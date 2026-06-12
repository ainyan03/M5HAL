// SPDX-License-Identifier: MIT
#ifndef M5_HAL_BYTECODE_BYTECODE_HPP_
#define M5_HAL_BYTECODE_BYTECODE_HPP_

#include "../data.hpp"
#include "../data/memory.hpp"
#include "../gpio/group.hpp"
#include "../i2c/i2c.hpp"
#include "../i2s/i2s.hpp"
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
    gpio_set_mode     = 0x20,  ///< [mode:1]([gpio_num:u16])*
    gpio_write_high   = 0x21,  ///< ([gpio_num:u16])*
    gpio_write_low    = 0x22,  ///< ([gpio_num:u16])*
    gpio_read         = 0x23,  ///< [store_id:1]([gpio_num:u16])* -> bits packed LSB-first
    gpio_subscribe    = 0x24,  ///< ([gpio_num:u16])*              (change-notification machinery)
    gpio_unsubscribe  = 0x25,  ///< ([gpio_num:u16])*; empty = all
    store_data        = 0x40,  ///< [store_id:1][data...]          (response)
    report_error      = 0x41,  ///< [error:i8][offset:LenVar]      (response)
    report_complete   = 0x42,  ///< [status:i8]                    (response)
    evt_gpio_state    = 0x60,  ///< ([gpio_num:u16][level:u8])*    (event)
    evt_stream_credit = 0x61,  ///< [kind:1][bus_id:1][free:u32][submitted:u32]   (event) stream credit snapshot
    bus_write_stream  = 0xB0,  ///< [kind:1][bus_id:1][data...]                   (critical) write to a stream bus (I2S)
    bus_stream_status = 0xB1,  ///< [kind:1][bus_id:1][store_id:1] -> [free:u32][submitted:u32] (critical)
};

/*! @brief Opcode bit7: an unknown critical opcode aborts instead of being skipped. */
constexpr uint8_t kCriticalOpcodeBit = 0x80;

/*! @brief `store_id` that discards the read data instead of storing it. */
constexpr uint8_t kDiscardStoreId = 0xFF;

constexpr size_t kMaxStoreSlots  = 8;  ///< Labeled response slots per runner.
constexpr size_t kMaxBusBindings = 4;  ///< Registered accessors per bus kind.

// ---- bus_configure config payload layout (wire format) ---------------------
//
// Per-kind config blob sizes (the encoder's; decode is tolerant) and
// the byte offsets of the u32-LE timeout fields inside each blob. The
// offsets are the single source shared by `encodeConfig` (bytecode.inl)
// and the server's prescan policy checks (`Server::prescan`,
// remote.inl) — both reference these names, never raw numbers.
constexpr size_t kI2CConfigSize  = 12;
constexpr size_t kSPIConfigSize  = 16;
constexpr size_t kUARTConfigSize = 24;
constexpr size_t kI2SConfigSize  = 14;

constexpr size_t kI2CConfigTimeoutOffset           = 4;   ///< timeout_ms
constexpr size_t kSPIConfigTimeoutOffset           = 6;   ///< timeout_ms
constexpr size_t kUARTConfigTimeoutOffset          = 4;   ///< timeout_ms
constexpr size_t kUARTConfigFirstByteTimeoutOffset = 8;   ///< first_byte_timeout_ms
constexpr size_t kUARTConfigInterByteTimeoutOffset = 12;  ///< inter_byte_timeout_ms
constexpr size_t kUARTConfigWriteTimeoutOffset     = 16;  ///< write_timeout_ms
constexpr size_t kI2SConfigTimeoutOffset           = 4;   ///< timeout_ms
constexpr size_t kI2SConfigWriteTimeoutOffset      = 8;   ///< write_timeout_ms

static_assert(kI2CConfigTimeoutOffset + 4 <= kI2CConfigSize, "i2c timeout field must fit the config blob");
static_assert(kSPIConfigTimeoutOffset + 4 <= kSPIConfigSize, "spi timeout field must fit the config blob");
static_assert(kUARTConfigWriteTimeoutOffset + 4 <= kUARTConfigSize, "uart timeout fields must fit the config blob");
static_assert(kI2SConfigWriteTimeoutOffset + 4 <= kI2SConfigSize, "i2s timeout fields must fit the config blob");

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

/*!
  @brief Write a LenVar; `dst` must hold `lenVarSize(value)` bytes. Returns bytes written.

  The valid range is [0, 0xFFFFFFFE] (i.e. the full u32 space minus the
  reserved marker 0xFF). On a 64-bit host, passing a value above 0xFFFFFFFF
  silently truncates to the low 32 bits — this is a caller-contract violation.
  The caller is responsible for ensuring the value fits in u32 before calling.
 */
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

    m5::hal::v1::result_t<void> delayMs(uint32_t ms);

    m5::hal::v1::result_t<void> configure(uint8_t bus_id, const i2c::I2CMasterAccessConfig& cfg);
    m5::hal::v1::result_t<void> configure(uint8_t bus_id, const spi::SPIMasterAccessConfig& cfg);
    m5::hal::v1::result_t<void> configure(uint8_t bus_id, const uart::UARTAccessConfig& cfg);
    /*! @brief Configure a registered I2S accessor (spec §stream credit). */
    m5::hal::v1::result_t<void> i2sConfig(uint8_t bus_id, const i2s::I2SAccessConfig& cfg);

    m5::hal::v1::result_t<void> transfer(uint8_t bus_id, const i2c::TransferDesc& desc, data::ConstDataSpan tx,
                                         size_t rx_len, uint8_t store_id = kDiscardStoreId);
    m5::hal::v1::result_t<void> transfer(uint8_t bus_id, const spi::TransferDesc& desc, data::ConstDataSpan tx,
                                         size_t rx_len, uint8_t store_id = kDiscardStoreId);
    /*! @brief UART transfer: write `tx`, then read up to `rx_len` bytes. */
    m5::hal::v1::result_t<void> uartTransfer(uint8_t bus_id, data::ConstDataSpan tx, size_t rx_len,
                                             uint8_t store_id = kDiscardStoreId);

    m5::hal::v1::result_t<void> gpioSetMode(types::gpio_mode_t mode, const types::gpio_number_t* pins, size_t count);
    m5::hal::v1::result_t<void> gpioWriteHigh(const types::gpio_number_t* pins, size_t count);
    m5::hal::v1::result_t<void> gpioWriteLow(const types::gpio_number_t* pins, size_t count);
    m5::hal::v1::result_t<void> gpioRead(uint8_t store_id, const types::gpio_number_t* pins, size_t count);
    m5::hal::v1::result_t<void> gpioSubscribe(const types::gpio_number_t* pins, size_t count);
    /*! @brief `count == 0` encodes "unsubscribe all". */
    m5::hal::v1::result_t<void> gpioUnsubscribe(const types::gpio_number_t* pins, size_t count);
    /*! @brief Event payload: changed pins with their new levels (parallel arrays). */
    m5::hal::v1::result_t<void> evtGpioState(const types::gpio_number_t* pins, const bool* levels, size_t count);

    /*! @name Stream credit (spec §stream credit). @{ */
    /*! @brief Write `data` to a stream bus (currently I2S). data length is self-described. */
    m5::hal::v1::result_t<void> busWriteStream(types::bus_kind_t kind, uint8_t bus_id, const uint8_t* data, size_t len);
    /*! @brief Query the stream bus credit ([free:u32][submitted:u32] into the slot). */
    m5::hal::v1::result_t<void> busStreamStatus(types::bus_kind_t kind, uint8_t bus_id, uint8_t store_id);
    /*! @brief Event: device-side credit snapshot (free + cumulative submitted). */
    m5::hal::v1::result_t<void> evtStreamCredit(types::bus_kind_t kind, uint8_t bus_id, uint32_t free,
                                                uint32_t submitted);
    /*! @} */

    m5::hal::v1::result_t<void> storeData(uint8_t store_id, data::ConstDataSpan bytes);
    m5::hal::v1::result_t<void> reportError(m5::hal::v1::error::error_t err, size_t offset);
    m5::hal::v1::result_t<void> reportComplete(m5::hal::v1::error::error_t status);

    /*! @brief Write the explicit script terminator (LenVar 0). */
    m5::hal::v1::result_t<void> end(void);

private:
    // Reserve one whole instruction, write the size prefix + opcode,
    // and expose the payload area. Committed by emit().
    m5::hal::v1::result_t<data::DataSpan> beginInstruction(OpCode opcode, size_t payload_size);
    m5::hal::v1::result_t<void> emit(void);

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

    m5::hal::v1::result_t<void> registerI2C(uint8_t bus_id, i2c::I2CMasterAccessor& acc);
    m5::hal::v1::result_t<void> registerSPI(uint8_t bus_id, spi::SPIMasterAccessor& acc);
    m5::hal::v1::result_t<void> registerUART(uint8_t bus_id, uart::UARTAccessor& acc);
    /*!
      @brief Register an I2S TX accessor as a stream bus (spec §stream credit).

      Carries a per-binding `submitted` counter — the cumulative bytes
      accepted by `bus_write_stream` (mod 2^32). The counter lives with the
      runner binding, so the absolute-value credit scheme holds for local
      byte-array execution too (symmetric design).
     */
    m5::hal::v1::result_t<void> registerI2S(uint8_t bus_id, i2s::I2STxAccessor& acc);
    void setGPIOGroup(gpio::GPIOGroup& group)
    {
        _gpio_group = &group;
    }
    /*! @brief Replace the delay backend (default: `m5::utility::delay`). */
    void setDelayFn(delay_fn_t fn)
    {
        _delay_fn = fn;
    }

    /*!
      @name Subscription / event hooks (the remote push machinery).

      The runner carries no subscription state itself; it only routes
      the opcodes. `gpio_subscribe` / `gpio_unsubscribe` dispatch to the
      subscribe handler (`subscribe` argument tells which; pins may be
      empty for "unsubscribe all") and fail with `UNSUPPORTED` when no
      handler is installed — the correct semantics for a local-only
      runner. `evt_gpio_state` calls the event handler once per
      (pin, level) entry and is silently ignored when none is installed.
      @{
     */
    using gpio_subscribe_fn_t = m5::hal::v1::result_t<void> (*)(void* ctx, bool subscribe,
                                                                const types::gpio_number_t* pins, size_t count);
    using gpio_event_fn_t     = void (*)(void* ctx, types::gpio_number_t pin, bool level);

    void setGpioSubscribeHandler(gpio_subscribe_fn_t fn, void* ctx)
    {
        _gpio_subscribe_fn  = fn;
        _gpio_subscribe_ctx = ctx;
    }
    void setGpioEventHandler(gpio_event_fn_t fn, void* ctx)
    {
        _gpio_event_fn  = fn;
        _gpio_event_ctx = ctx;
    }

    /*!
      @brief Receive-side dispatch for `evt_stream_credit` (spec §stream
             credit). Routed to the handler once per event; silently
             ignored when none is installed (same as `evt_gpio_state`).
     */
    using stream_credit_fn_t = void (*)(void* ctx, types::bus_kind_t kind, uint8_t bus_id, uint32_t free,
                                        uint32_t submitted);
    void setStreamCreditHandler(stream_credit_fn_t fn, void* ctx)
    {
        _stream_credit_fn  = fn;
        _stream_credit_ctx = ctx;
    }
    /*! @brief Context registered with the stream-credit handler (`nullptr` when unset).
        Lets a registrant unregister itself only while it still owns the slot. */
    void* streamCreditHandlerCtx() const
    {
        return _stream_credit_ctx;
    }
    /*! @} */

    /*! @brief Result of `i2sStreamStatus` — the device-side credit snapshot. */
    struct StreamStatus {
        uint32_t free      = 0;  ///< Bytes writable now without blocking (writableBytes()).
        uint32_t submitted = 0;  ///< Cumulative bytes accepted by this binding (mod 2^32).
    };
    /*!
      @brief Non-script credit query for the server poll path: reads the
             registered I2S accessor's `writableBytes()` and the binding's
             cumulative `submitted`. Fails with `INVALID_ARGUMENT` when the
             bus_id is out of range or has no I2S binding.
     */
    m5::hal::v1::result_t<StreamStatus> i2sStreamStatus(uint8_t bus_id);

    /*! @brief Execute from any Source. Returns the consumed byte count. */
    m5::hal::v1::result_t<size_t> run(data::Source& script);
    /*! @brief Execute straight from a local byte array. */
    m5::hal::v1::result_t<size_t> run(data::ConstDataSpan script);

    /*!
      @brief Execute an EVENT script without disturbing request state.

      Unlike `run`, this neither clears the stored slots nor resets the
      report state, and `store_data` / `report_*` inside the script are
      ignored — a poll-path event must not clobber the response data the
      caller is still reading (S16 D8). Event dispatch (`evt_*` handlers)
      works as in `run`.
     */
    m5::hal::v1::result_t<size_t> runEvent(data::ConstDataSpan script);

    /*!
      @brief Restrict dispatch to receive-side opcodes.

      With receive-only set, executable opcodes (`delay_ms`, `bus_*`,
      `gpio_*`) are rejected with `PROTOCOL_ERROR`: a script received
      FROM a peer must fill in data and report status, never drive the
      local buses, pins, or clock (S16 D8 — trust is symmetric; a buggy
      or hostile peer must not stall or actuate this side). Unknown
      non-critical opcodes still skip for forward compatibility. The
      host-side `RemoteSession` enables this on its runner; the
      server's executing runner keeps the full set.
     */
    void setReceiveOnly(bool receive_only)
    {
        _receive_only = receive_only;
    }
    bool receiveOnly(void) const
    {
        return _receive_only;
    }

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
    m5::hal::v1::result_t<void> writeResponse(data::Sink& out, m5::hal::v1::error::error_t status);

private:
    struct Slot {
        uint8_t id = kDiscardStoreId;
        size_t len = 0;
        memory::TempBuffer buf;
    };

    m5::hal::v1::result_t<Slot*> allocStore(uint8_t store_id, size_t size);

    m5::hal::v1::result_t<void> dispatch(uint8_t opcode, data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opDelay(data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opBusConfigure(data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opBusTransfer(data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opGpio(uint8_t opcode, data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opGpioSubscribe(uint8_t opcode, data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opEvtGpioState(data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opBusWriteStream(data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opBusStreamStatus(data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opEvtStreamCredit(data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opStoreData(data::ConstDataSpan payload);
    m5::hal::v1::result_t<void> opReport(uint8_t opcode, data::ConstDataSpan payload);

    // Shared instruction loop behind run()/runEvent(); state resets stay
    // in the entry points.
    m5::hal::v1::result_t<size_t> runLoop(data::Source& script);

    memory::Allocator* _alloc                     = nullptr;
    i2c::I2CMasterAccessor* _i2c[kMaxBusBindings] = {};
    spi::SPIMasterAccessor* _spi[kMaxBusBindings] = {};
    uart::UARTAccessor* _uart[kMaxBusBindings]    = {};
    i2s::I2STxAccessor* _i2s[kMaxBusBindings]     = {};
    // Per-binding cumulative bytes accepted by bus_write_stream (mod 2^32).
    uint32_t _stream_submitted[kMaxBusBindings] = {};
    gpio::GPIOGroup* _gpio_group                = nullptr;
    delay_fn_t _delay_fn                        = nullptr;  // nullptr -> m5::utility::delay
    gpio_subscribe_fn_t _gpio_subscribe_fn      = nullptr;
    void* _gpio_subscribe_ctx                   = nullptr;
    gpio_event_fn_t _gpio_event_fn              = nullptr;
    void* _gpio_event_ctx                       = nullptr;
    stream_credit_fn_t _stream_credit_fn        = nullptr;
    void* _stream_credit_ctx                    = nullptr;

    Slot _slots[kMaxStoreSlots];
    bool _status_reported                        = false;
    m5::hal::v1::error::error_t _reported_status = m5::hal::v1::error::error_t::OK;
    size_t _reported_offset                      = 0;
    size_t _last_offset                          = 0;
    size_t _unknown_skipped                      = 0;
    bool _receive_only                           = false;  // reject executable opcodes (S16 D8)
    bool _event_mode                             = false;  // runEvent(): store/report are ignored
};

}  // namespace m5::hal::v1::bytecode

#endif
