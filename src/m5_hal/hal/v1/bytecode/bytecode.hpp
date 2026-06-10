#ifndef M5_HAL_BYTECODE_BYTECODE_HPP_
#define M5_HAL_BYTECODE_BYTECODE_HPP_

#include "../data.hpp"
#include "../data/memory.hpp"
#include "../gpio/group.hpp"
#include "../i2c/i2c.hpp"
#include "../memory/allocator.hpp"
#include "../spi/spi.hpp"
#include "../types.hpp"
#include "../uart/uart.hpp"

#include <M5Utility.hpp>

#include <stddef.h>
#include <stdint.h>

// =============================================================================
// Bytecode: a compact, self-describing instruction stream that drives the
// v1 HAL (bus transfers, GPIO, delays). The authoritative spec lives in
// spec/design/bytecode.md.
//
// Primary use case: executing a script straight from a local byte array
// (e.g. a const init table) - `runner.run(ConstDataSpan{table, len})`.
// The same scripts travel over the frame codec for remote execution.
//
// Wire format ("M5HAL bytecode v1"):
//   instruction : [LenVar size][opcode:1][payload:size-1]   (size includes opcode)
//   terminator  : LenVar 0 (a single 0x00); end-of-input is also a clean end
//   LenVar (LE) : 0x00-0xFC = 1 byte | 0xFD + u16 LE | 0xFE + u32 LE | 0xFF reserved
//
// Every instruction is length-prefixed and therefore skippable: an
// unknown opcode with bit7 clear is skipped (forward compatibility);
// bit7 set marks a "critical" instruction the runner must understand,
// so an unknown critical opcode aborts with PROTOCOL_ERROR.
//
// Responses are bytecode too (symmetric pipeline): read results become
// `store_data` instructions, completion becomes `report_*`. The same
// BytecodeRunner executes both directions - on the device side it
// dispatches to the HAL, on the host side a response script just fills
// the store slots and records the reported status.
// =============================================================================

/*!
  @namespace m5::hal::v1::bytecode
  @brief Self-describing HAL instruction stream: encoder + runner.
 */
namespace m5::hal::v1::bytecode {

enum class OpCode : uint8_t {
    delay_ms      = 0x01,  ///< [ms:u32]
    bus_configure = 0x10,  ///< [kind:1][bus_id:1][cfg payload]
    bus_transfer  = 0x11,  ///< [kind:1][bus_id:1][store_id:1][rx_len:LenVar][meta_size:1][meta][tx...]
    // 0x12 bus_init / 0x13 bus_deinit : reserved (bus lifecycle stays with the host app for now)
    gpio_set_mode   = 0x20,  ///< [mode:1]([gpio_num:u16])*
    gpio_write_high = 0x21,  ///< ([gpio_num:u16])*
    gpio_write_low  = 0x22,  ///< ([gpio_num:u16])*
    gpio_read       = 0x23,  ///< [store_id:1]([gpio_num:u16])* -> bits packed LSB-first
    // 0x24 gpio_subscribe / 0x25 gpio_unsubscribe : reserved (event machinery)
    store_data      = 0x40,  ///< [store_id:1][data...]          (response)
    report_error    = 0x41,  ///< [error:i8][offset:LenVar]      (response)
    report_complete = 0x42,  ///< [status:i8]                    (response)
    // 0x60 evt_gpio_state : reserved (push events)
};

/*! @brief Opcode bit7: an unknown critical opcode aborts instead of being skipped. */
constexpr uint8_t kCriticalOpcodeBit = 0x80;

/*! @brief `store_id` that discards the read data instead of storing it. */
constexpr uint8_t kDiscardStoreId = 0xFF;

constexpr size_t kMaxStoreSlots  = 8;  ///< Labeled response slots per runner.
constexpr size_t kMaxBusBindings = 4;  ///< Registered accessors per bus kind.

/*!
  @brief Decoded little-endian length prefix.

  `consumed == 0` means `src` is too short to decode; `valid == false`
  means the reserved marker byte (0xFF) was encountered.
 */
struct LenVar {
    size_t value    = 0;
    size_t consumed = 0;
    bool valid      = true;
};

LenVar decodeLenVar(data::ConstDataSpan src);

/*! @brief Encoded byte count of a LenVar for `value` (1, 3, or 5). */
constexpr size_t lenVarSize(size_t value)
{
    return value <= 0xFC ? 1 : (value <= 0xFFFF ? 3 : 5);
}

/*! @brief Write a LenVar; `dst` must hold `lenVarSize(value)` bytes. Returns bytes written. */
size_t encodeLenVar(uint8_t* dst, size_t value);

/*!
  @brief Builds bytecode instructions into a `data::Sink`.

  One Sink target covers both shapes: a `MemorySink` builds a local
  byte array, a `StreamSink` (over a UART TX accessor) streams the
  script out as it is encoded. Each instruction is reserved, built in
  place, and committed as a unit; a Sink that cannot lend the whole
  instruction yields `CLOSED` (sink closed) or `BUFFER_OVERFLOW`.
 */
class BytecodeEncoder {
public:
    explicit BytecodeEncoder(data::Sink& out) : _sink{&out}
    {
    }

    m5::stl::expected<void, m5::hal::v1::error::error_t> delayMs(uint32_t ms);

    m5::stl::expected<void, m5::hal::v1::error::error_t> configure(uint8_t bus_id,
                                                                   const i2c::I2CMasterAccessConfig& cfg);
    m5::stl::expected<void, m5::hal::v1::error::error_t> configure(uint8_t bus_id,
                                                                   const spi::SPIMasterAccessConfig& cfg);
    m5::stl::expected<void, m5::hal::v1::error::error_t> configure(uint8_t bus_id, const uart::UARTAccessConfig& cfg);

    m5::stl::expected<void, m5::hal::v1::error::error_t> transfer(uint8_t bus_id, const i2c::TransferDesc& desc,
                                                                  data::ConstDataSpan tx, size_t rx_len,
                                                                  uint8_t store_id = kDiscardStoreId);
    m5::stl::expected<void, m5::hal::v1::error::error_t> transfer(uint8_t bus_id, const spi::TransferDesc& desc,
                                                                  data::ConstDataSpan tx, size_t rx_len,
                                                                  uint8_t store_id = kDiscardStoreId);
    /*! @brief UART transfer: write `tx`, then read up to `rx_len` bytes. */
    m5::stl::expected<void, m5::hal::v1::error::error_t> uartTransfer(uint8_t bus_id, data::ConstDataSpan tx,
                                                                      size_t rx_len,
                                                                      uint8_t store_id = kDiscardStoreId);

    m5::stl::expected<void, m5::hal::v1::error::error_t> gpioSetMode(types::gpio_mode_t mode,
                                                                     const types::gpio_number_t* pins, size_t count);
    m5::stl::expected<void, m5::hal::v1::error::error_t> gpioWriteHigh(const types::gpio_number_t* pins, size_t count);
    m5::stl::expected<void, m5::hal::v1::error::error_t> gpioWriteLow(const types::gpio_number_t* pins, size_t count);
    m5::stl::expected<void, m5::hal::v1::error::error_t> gpioRead(uint8_t store_id, const types::gpio_number_t* pins,
                                                                  size_t count);

    m5::stl::expected<void, m5::hal::v1::error::error_t> storeData(uint8_t store_id, data::ConstDataSpan bytes);
    m5::stl::expected<void, m5::hal::v1::error::error_t> reportError(m5::hal::v1::error::error_t err, size_t offset);
    m5::stl::expected<void, m5::hal::v1::error::error_t> reportComplete(m5::hal::v1::error::error_t status);

    /*! @brief Write the explicit script terminator (LenVar 0). */
    m5::stl::expected<void, m5::hal::v1::error::error_t> end(void);

private:
    // Reserve one whole instruction, write the size prefix + opcode,
    // and expose the payload area. Committed by emit().
    m5::stl::expected<data::DataSpan, m5::hal::v1::error::error_t> beginInstruction(OpCode opcode, size_t payload_size);
    m5::stl::expected<void, m5::hal::v1::error::error_t> emit(void);

    data::Sink* _sink  = nullptr;
    size_t _instr_size = 0;
};

/*!
  @brief Executes a bytecode script against registered v1 HAL targets.

  Local execution is the primary path: `run(ConstDataSpan)` executes a
  script straight out of a byte array (it wraps a `MemorySource`
  internally, so byte arrays and streams share one implementation).
  `run(data::Source&)` executes from any Source - a `StreamSource`
  over a UART RX accessor, a frame payload, a file replay, ...

  Dispatch targets are registered up front (`registerI2C` /
  `registerSPI` / `registerUART` with a bus_id, `setGPIOGroup` for the
  unified `gpio_number_t` space). An instruction addressing an
  unregistered target fails with `INVALID_ARGUMENT`.

  Read results land in labeled store slots (`kMaxStoreSlots`, backed
  by `memory::TempBuffer`): `storedData(store_id)` after `run` returns
  the bytes. Slots are cleared at the start of each `run`.

  The same runner executes response scripts (symmetric pipeline):
  `store_data` fills a slot, `report_error` / `report_complete` are
  recorded and readable via `statusReported()` / `reportedStatus()`.
  `writeResponse(sink, status)` does the reverse - it emits the
  current slots and a final report as a response script.

  Error policy: the runner stops at the first failing instruction and
  returns its error; `lastOffset()` is the byte offset of that
  instruction. Unknown non-critical opcodes are skipped and counted
  (`unknownSkipped()`); unknown critical opcodes are PROTOCOL_ERROR.

  Streaming caveat: an instruction must be lendable by the Source in
  one `peek` - with a `StreamSource` the scratch must hold the largest
  instruction in the script. A stalled stream mid-instruction returns
  `BUFFER_UNDERFLOW` (the runner is not resumable mid-script).
 */
class BytecodeRunner {
public:
    using delay_fn_t = void (*)(uint32_t ms);

    explicit BytecodeRunner(memory::Allocator& alloc = memory::defaultAllocator()) : _alloc{&alloc}
    {
    }

    m5::stl::expected<void, m5::hal::v1::error::error_t> registerI2C(uint8_t bus_id, i2c::I2CMasterAccessor& acc);
    m5::stl::expected<void, m5::hal::v1::error::error_t> registerSPI(uint8_t bus_id, spi::SPIMasterAccessor& acc);
    m5::stl::expected<void, m5::hal::v1::error::error_t> registerUART(uint8_t bus_id, uart::UARTAccessor& acc);
    void setGPIOGroup(gpio::GPIOGroup& group)
    {
        _gpio_group = &group;
    }
    /*! @brief Replace the delay backend (default: `m5::utility::delay`). */
    void setDelayFn(delay_fn_t fn)
    {
        _delay_fn = fn;
    }

    /*! @brief Execute from any Source. Returns the consumed byte count. */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> run(data::Source& script);
    /*! @brief Execute straight from a local byte array. */
    m5::stl::expected<size_t, m5::hal::v1::error::error_t> run(data::ConstDataSpan script);

    /*! @brief Stored bytes for `store_id`; empty span when the slot is unused. */
    data::ConstDataSpan storedData(uint8_t store_id) const;
    size_t storedCount(void) const;
    /*! @brief store_id of the i-th used slot (i < storedCount()). */
    uint8_t storedIdAt(size_t index) const;
    void clearStored(void);

    /*! @brief True when the script carried a `report_error` / `report_complete`. */
    bool statusReported(void) const
    {
        return _status_reported;
    }
    m5::hal::v1::error::error_t reportedStatus(void) const
    {
        return _reported_status;
    }
    /*! @brief Offset carried by `report_error` (0 for `report_complete`). */
    size_t reportedOffset(void) const
    {
        return _reported_offset;
    }

    /*! @brief Byte offset of the most recently dispatched (or failed) instruction. */
    size_t lastOffset(void) const
    {
        return _last_offset;
    }
    /*! @brief Unknown non-critical instructions skipped by the last run. */
    size_t unknownSkipped(void) const
    {
        return _unknown_skipped;
    }

    /*!
      @brief Emit the current store slots and a final report as a
             response script into `out` (terminator included).
     */
    m5::stl::expected<void, m5::hal::v1::error::error_t> writeResponse(data::Sink& out,
                                                                       m5::hal::v1::error::error_t status);

private:
    struct Slot {
        uint8_t id = kDiscardStoreId;
        size_t len = 0;
        memory::TempBuffer buf;
    };

    m5::stl::expected<Slot*, m5::hal::v1::error::error_t> allocStore(uint8_t store_id, size_t size);

    m5::stl::expected<void, m5::hal::v1::error::error_t> dispatch(uint8_t opcode, data::ConstDataSpan payload);
    m5::stl::expected<void, m5::hal::v1::error::error_t> opDelay(data::ConstDataSpan payload);
    m5::stl::expected<void, m5::hal::v1::error::error_t> opBusConfigure(data::ConstDataSpan payload);
    m5::stl::expected<void, m5::hal::v1::error::error_t> opBusTransfer(data::ConstDataSpan payload);
    m5::stl::expected<void, m5::hal::v1::error::error_t> opGpio(uint8_t opcode, data::ConstDataSpan payload);
    m5::stl::expected<void, m5::hal::v1::error::error_t> opStoreData(data::ConstDataSpan payload);
    m5::stl::expected<void, m5::hal::v1::error::error_t> opReport(uint8_t opcode, data::ConstDataSpan payload);

    memory::Allocator* _alloc                     = nullptr;
    i2c::I2CMasterAccessor* _i2c[kMaxBusBindings] = {};
    spi::SPIMasterAccessor* _spi[kMaxBusBindings] = {};
    uart::UARTAccessor* _uart[kMaxBusBindings]    = {};
    gpio::GPIOGroup* _gpio_group                  = nullptr;
    delay_fn_t _delay_fn                          = nullptr;  // nullptr -> m5::utility::delay

    Slot _slots[kMaxStoreSlots];
    bool _status_reported                        = false;
    m5::hal::v1::error::error_t _reported_status = m5::hal::v1::error::error_t::OK;
    size_t _reported_offset                      = 0;
    size_t _last_offset                          = 0;
    size_t _unknown_skipped                      = 0;
};

}  // namespace m5::hal::v1::bytecode

#endif
