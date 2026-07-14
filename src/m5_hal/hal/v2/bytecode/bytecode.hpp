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
// v2 HAL (bus transfers, GPIO, delays). The authoritative spec lives in
// spec/design/bytecode.md.
//
// Primary use case: executing a script straight from a local byte array
// (e.g. a const init table) - `runner.run(ConstDataSpan{table, len})`.
// The same scripts travel over the frame codec for remote execution.
//
// Wire format ("M5HAL bytecode v2"):
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
// `StoreData` instructions, completion becomes `Report*`. The same
// BytecodeRunner executes both directions - on the device side it
// dispatches to the HAL, on the host side a response script just fills
// the store slots and records the reported status.
// =============================================================================

/*!
  @namespace m5::hal::v2::bytecode
  @brief Self-describing HAL instruction stream: encoder + runner.
 */
namespace m5::hal::v2::bytecode {

enum class OpCode : uint8_t {
    DelayMs             = 0x01,  ///< [ms:u32]
    BusConfigure        = 0x10,  ///< [kind:1][bus_id:1][cfg payload]
    BusTransfer         = 0x11,  ///< [kind:1][bus_id:1][store_id:1][rx_len:LenVar][meta_size:1][meta][tx...]
    BusStreamTransfer   = 0x12,  ///< [kind:1][bus_id:1][stream_id:1][meta_size:1][tx_len:4LE][rx_len:4LE][meta]
    GpioSetMode         = 0x20,  ///< [mode:1]([gpio_num:u16])*
    GpioWriteHigh       = 0x21,  ///< ([gpio_num:u16])*
    GpioWriteLow        = 0x22,  ///< ([gpio_num:u16])*
    GpioRead            = 0x23,  ///< [store_id:1]([gpio_num:u16])* -> bits packed LSB-first
    GpioSubscribe       = 0x24,  ///< ([gpio_num:u16])*              (change-notification machinery)
    GpioUnsubscribe     = 0x25,  ///< ([gpio_num:u16])*; empty = all
    GpioAllowlist       = 0x26,  ///< ([pin:u8])*                    (remote GPIO pin allowlist)
    GpioPortRead        = 0x27,  ///< [store_id:1][slot:1][port_index:1] -> u32 LE (denied bits = 0)
    GpioPortWrite       = 0x28,  ///< [slot:1][port_index:1][set_mask:u32 LE][clear_mask:u32 LE]
    StoreData           = 0x40,  ///< [store_id:1][data...]          (response)
    ReportError         = 0x41,  ///< [error:i8][offset:LenVar]      (response)
    ReportComplete      = 0x42,  ///< [status:i8]                    (response)
    EvtGpioState        = 0x60,  ///< ([gpio_num:u16][level:u8])*    (event)
    BusCreate           = 0x92,  ///< [kind:1][bus_id:1][store_id:1][pin_config...] (critical) dynamic bus creation
    BusRelease          = 0x93,  ///< [kind:1][bus_id:1]                           (critical) dynamic bus teardown
    BusBeginTransaction = 0xB4,  ///< [kind:1][bus_id:1]                       (critical) CS assert
    BusEndTransaction   = 0xB5,  ///< [kind:1][bus_id:1]                       (critical) CS deassert
};

/*! @brief Opcode bit7: an unknown critical opcode aborts instead of being skipped. */
constexpr uint8_t kCriticalOpcodeBit = 0x80;

/*! @brief `store_id` that discards the read data instead of storing it. */
constexpr uint8_t kDiscardStoreId = 0xFF;

constexpr size_t kMaxStoreSlots  = 8;  ///< Labeled response slots per runner.
constexpr size_t kMaxBusBindings = 4;  ///< Registered accessors per bus kind.

// ---- BusConfigure config payload layout (wire format) ---------------------
//
// Per-kind config blob sizes (the encoder's; decode is tolerant) and
// the byte offsets of the u32-LE timeout fields inside each blob. The
// offsets are the single source shared by `encodeConfig` (bytecode.inl)
// and the server's prescan policy checks (`Server::prescan`,
// remote.inl) — both reference these names, never raw numbers.
// Lock-acquisition timeouts are NOT part of the wire format: they are a
// call-context property of the host-side API, and the device
// serves one session at a time, so its local bus locks are uncontended.
// The timeouts that remain on the wire all bound real wire/driver time.
constexpr size_t kI2CConfigSize  = 12;
constexpr size_t kSPIConfigSize  = 14;
constexpr size_t kUARTConfigSize = 20;
constexpr size_t kI2SConfigSize  = 14;

constexpr size_t kI2CConfigWireTimeoutOffset       = 4;   ///< wire_timeout_ms
constexpr size_t kUARTConfigFirstByteTimeoutOffset = 4;   ///< first_byte_timeout_ms
constexpr size_t kUARTConfigInterByteTimeoutOffset = 8;   ///< inter_byte_timeout_ms
constexpr size_t kUARTConfigWriteTimeoutOffset     = 12;  ///< write_timeout_ms
constexpr size_t kI2SConfigWriteTimeoutOffset      = 4;   ///< write_timeout_ms
constexpr size_t kI2SConfigReadTimeoutOffset       = 10;  ///< read_timeout_ms

static_assert(kI2CConfigWireTimeoutOffset + 4 <= kI2CConfigSize, "i2c timeout field must fit the config blob");
static_assert(kUARTConfigWriteTimeoutOffset + 4 <= kUARTConfigSize, "uart timeout fields must fit the config blob");
static_assert(kI2SConfigWriteTimeoutOffset + 4 <= kI2SConfigSize, "i2s timeout fields must fit the config blob");
static_assert(kI2SConfigReadTimeoutOffset + 4 <= kI2SConfigSize, "i2s timeout fields must fit the config blob");

namespace detail {

struct RunnerBusBinding;

struct RunnerBusOps {
    m5::hal::v2::result_t<void> (*configure)(RunnerBusBinding& binding, data::ConstDataSpan cfg) = nullptr;
    m5::hal::v2::result_t<void> (*transfer)(RunnerBusBinding& binding, data::ConstDataSpan meta,
                                            data::ConstDataSpan src, data::DataSpan dst, size_t& actual_tx_len,
                                            size_t& actual_rx_len)                               = nullptr;
    m5::hal::v2::result_t<void> (*begin_transaction)(RunnerBusBinding& binding)                  = nullptr;
    m5::hal::v2::result_t<void> (*end_transaction)(RunnerBusBinding& binding)                    = nullptr;
};

struct RunnerBusBinding {
    types::bus_kind_t kind  = types::bus_kind_t::Unknown;
    uint8_t bus_id          = 0;
    void* main              = nullptr;
    void* tx                = nullptr;
    void* rx                = nullptr;
    const RunnerBusOps* ops = nullptr;

    bool registered() const
    {
        return ops != nullptr;
    }
    bool matches(types::bus_kind_t k, uint8_t b) const
    {
        return registered() && kind == k && bus_id == b;
    }
    void clear()
    {
        kind   = types::bus_kind_t::Unknown;
        bus_id = 0;
        main   = nullptr;
        tx     = nullptr;
        rx     = nullptr;
        ops    = nullptr;
    }
};

// Canonical per-kind config-blob serializers (wire format). The packing
// itself lives in bytecode.inl next to each decode counterpart; these
// entry points exist so the remote host proxy reuses that single
// definition instead of re-implementing the bit layout.
void encodeConfig(uint8_t* dst, const i2c::MasterAccessConfig& cfg);
void encodeConfig(uint8_t* dst, const spi::MasterAccessConfig& cfg);
void encodeConfig(uint8_t* dst, const uart::AccessConfig& cfg);
void encodeConfig(uint8_t* dst, const i2s::AccessConfig& cfg);

}  // namespace detail

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

  The valid range is [0, 0xFFFFFFFF] (the full u32 space). The reserved marker
  0xFF is a prefix byte, not a reserved decoded value. On a 64-bit host,
  passing a value above 0xFFFFFFFF
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
    explicit BytecodeEncoder(data::Sink& dst) : _sink{&dst}
    {
    }

    m5::hal::v2::result_t<void> delayMs(uint32_t ms);

    m5::hal::v2::result_t<void> configure(uint8_t bus_id, const i2c::MasterAccessConfig& cfg);
    m5::hal::v2::result_t<void> configure(uint8_t bus_id, const spi::MasterAccessConfig& cfg);
    m5::hal::v2::result_t<void> configure(uint8_t bus_id, const uart::AccessConfig& cfg);
    /*! @brief Raw form: `cfg_bytes` is an already-serialized config blob for `kind`. */
    m5::hal::v2::result_t<void> configure(types::bus_kind_t kind, uint8_t bus_id, data::ConstDataSpan cfg_bytes);
    /*! @brief Configure registered I2S stream accessors (TX/RX). */
    m5::hal::v2::result_t<void> i2sConfig(uint8_t bus_id, const i2s::AccessConfig& cfg);

    m5::hal::v2::result_t<void> transfer(uint8_t bus_id, const i2c::TransferDesc& desc, data::ConstDataSpan src,
                                         size_t rx_len, uint8_t store_id = kDiscardStoreId);
    m5::hal::v2::result_t<void> transfer(uint8_t bus_id, const spi::TransferDesc& desc, data::ConstDataSpan src,
                                         size_t rx_len, uint8_t store_id = kDiscardStoreId);
    /*! @brief UART transfer: write `src`, then read up to `rx_len` bytes. */
    m5::hal::v2::result_t<void> uartTransfer(uint8_t bus_id, data::ConstDataSpan src, size_t rx_len,
                                             uint8_t store_id = kDiscardStoreId);

    m5::hal::v2::result_t<void> streamTransfer(types::bus_kind_t kind, uint8_t bus_id, uint8_t stream_id,
                                               uint32_t tx_len, uint32_t rx_len, data::ConstDataSpan meta);

    m5::hal::v2::result_t<void> gpioSetMode(types::gpio_mode_t mode, const types::gpio_number_t* pins, size_t count);
    m5::hal::v2::result_t<void> gpioWriteHigh(const types::gpio_number_t* pins, size_t count);
    m5::hal::v2::result_t<void> gpioWriteLow(const types::gpio_number_t* pins, size_t count);
    m5::hal::v2::result_t<void> gpioRead(uint8_t store_id, const types::gpio_number_t* pins, size_t count);
    m5::hal::v2::result_t<void> gpioPortRead(uint8_t store_id, types::gpio_slot_t slot, uint8_t port_index);
    m5::hal::v2::result_t<void> gpioPortWrite(types::gpio_slot_t slot, uint8_t port_index, uint32_t set_mask,
                                              uint32_t clear_mask);
    m5::hal::v2::result_t<void> gpioSubscribe(const types::gpio_number_t* pins, size_t count);
    /*! @brief `count == 0` encodes "unsubscribe all". */
    m5::hal::v2::result_t<void> gpioUnsubscribe(const types::gpio_number_t* pins, size_t count);
    /*! @brief Event payload: changed pins with their new levels (parallel arrays). */
    m5::hal::v2::result_t<void> evtGpioState(const types::gpio_number_t* pins, const bool* levels, size_t count);

    m5::hal::v2::result_t<void> busCreate(types::bus_kind_t kind, uint8_t bus_id, uint8_t store_id,
                                          data::ConstDataSpan pin_config);
    m5::hal::v2::result_t<void> busRelease(types::bus_kind_t kind, uint8_t bus_id);

    m5::hal::v2::result_t<void> busBeginTransaction(types::bus_kind_t kind, uint8_t bus_id);
    m5::hal::v2::result_t<void> busEndTransaction(types::bus_kind_t kind, uint8_t bus_id);

    m5::hal::v2::result_t<void> storeData(uint8_t store_id, data::ConstDataSpan bytes);
    m5::hal::v2::result_t<void> reportError(m5::hal::v2::error::error_t err, size_t offset);
    m5::hal::v2::result_t<void> reportComplete(m5::hal::v2::error::error_t status);

    /*! @brief Write the explicit script terminator (LenVar 0). */
    m5::hal::v2::result_t<void> end(void);

private:
    // Reserve one whole instruction, write the size prefix + opcode,
    // and expose the payload area. Committed by emit().
    m5::hal::v2::result_t<data::DataSpan> beginInstruction(OpCode opcode, size_t payload_size);
    m5::hal::v2::result_t<void> emit(void);

    data::Sink* _sink  = nullptr;
    size_t _instr_size = 0;
};

/*!
  @brief Executes a bytecode script against registered v2 HAL targets.

  Local execution is the primary path: `run(ConstDataSpan)` executes a
  script straight out of a byte array (it wraps a `MemorySource`
  internally, so byte arrays and streams share one implementation).
  `run(data::Source&)` executes from any Source - a `StreamSource`
  over a UART RX accessor, a frame payload, a file replay, ...

  Dispatch targets are registered up front (`registerI2C` /
  `registerSPI` / `registerUART` with a bus_id, `setGPIOGroup` for the
  unified `gpio_number_t` space). An instruction addressing an
  unregistered target fails with `INVALID_STATE`.

  Read results land in labeled store slots (`kMaxStoreSlots`, backed
  by `memory::TempBuffer`): `storedData(store_id)` after `run` returns
  the bytes. Slots are cleared at the start of each `run`.

  The same runner executes response scripts (symmetric pipeline):
  `StoreData` fills a slot, `ReportError` / `ReportComplete` are
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

    m5::hal::v2::result_t<void> registerI2C(uint8_t bus_id, i2c::MasterAccessor& acc);
    m5::hal::v2::result_t<void> registerSPI(uint8_t bus_id, spi::MasterAccessor& acc);
    m5::hal::v2::result_t<void> registerUART(uint8_t bus_id, uart::Accessor& acc);
    m5::hal::v2::result_t<void> registerI2S(uint8_t bus_id, i2s::Accessor& acc);
    m5::hal::v2::result_t<void> registerI2S(uint8_t bus_id, i2s::TxAccessor& acc);
    m5::hal::v2::result_t<void> registerI2S(uint8_t bus_id, i2s::RxAccessor& acc);
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
      the opcodes. `GpioSubscribe` / `GpioUnsubscribe` dispatch to the
      subscribe handler (`subscribe` argument tells which; pins may be
      empty for "unsubscribe all") and fail with `UNSUPPORTED` when no
      handler is installed — the correct semantics for a local-only
      runner. `EvtGpioState` calls the event handler once per
      (pin, level) entry and is silently ignored when none is installed.
      @{
     */
    using gpio_subscribe_fn_t = m5::hal::v2::result_t<void> (*)(void* ctx, bool subscribe,
                                                                const types::gpio_number_t* pins, size_t count);
    using gpio_event_fn_t     = void (*)(void* ctx, types::gpio_number_t pin, bool level);
    using gpio_mode_fn_t      = m5::hal::v2::result_t<void> (*)(void* ctx, types::gpio_number_t pin,
                                                           types::gpio_mode_t mode);

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
    void setGpioModeHandler(gpio_mode_fn_t fn, void* ctx)
    {
        _gpio_mode_fn  = fn;
        _gpio_mode_ctx = ctx;
    }

    /*!
      @name Dynamic bus lifecycle hooks (BusCreate / BusRelease opcodes).

      The handler receives `create=true` for BusCreate (pin_config carries
      the kind-specific payload) and `create=false` for BusRelease
      (pin_config is empty). Returns `UNSUPPORTED` when no handler is
      installed. The handler is expected to call `registerI2C` / etc. on
      success (and `unregisterI2C` / etc. on release).
      @{
     */
    using bus_create_fn_t = m5::hal::v2::result_t<void> (*)(void* ctx, bool create, types::bus_kind_t kind,
                                                            uint8_t bus_id, data::ConstDataSpan pin_config);

    void setBusCreateHandler(bus_create_fn_t fn, void* ctx)
    {
        _bus_create_fn  = fn;
        _bus_create_ctx = ctx;
    }

    struct StreamTransferDesc {
        types::bus_kind_t kind;
        uint8_t bus_id;
        uint8_t stream_id;
        uint32_t tx_len;
        uint32_t rx_len;
        data::ConstDataSpan meta;
    };

    using stream_transfer_fn_t = m5::hal::v2::result_t<void> (*)(void* ctx, const StreamTransferDesc& desc);

    void setStreamTransferHandler(stream_transfer_fn_t fn, void* ctx)
    {
        _stream_transfer_fn  = fn;
        _stream_transfer_ctx = ctx;
    }

    using gpio_allowlist_fn_t = m5::hal::v2::result_t<void> (*)(void* ctx, const uint8_t* pins, size_t count);

    void setGpioAllowlistHandler(gpio_allowlist_fn_t fn, void* ctx)
    {
        _gpio_allowlist_fn  = fn;
        _gpio_allowlist_ctx = ctx;
    }
    /*! @} */

    void unregisterI2C(uint8_t bus_id);
    void unregisterSPI(uint8_t bus_id);
    void unregisterUART(uint8_t bus_id);
    void unregisterI2S(uint8_t bus_id);

    /*! @brief Execute from any Source. Returns the consumed byte count. */
    m5::hal::v2::result_t<size_t> run(data::Source& script);
    /*! @brief Execute straight from a local byte array. */
    m5::hal::v2::result_t<size_t> run(data::ConstDataSpan script);

    /*!
      @brief Execute an EVENT script without disturbing request state.

      Unlike `run`, this neither clears the stored slots nor resets the
      report state, and `StoreData` / `Report*` inside the script are
      ignored — a poll-path event must not clobber the response data the
      caller is still reading. Event dispatch (`evt_*` handlers)
      works as in `run`.
     */
    m5::hal::v2::result_t<size_t> runEvent(data::ConstDataSpan script);

    /*!
      @brief Restrict dispatch to receive-side opcodes.

      With receive-only set, executable opcodes (`DelayMs`, `Bus*`,
      `Gpio*`) are rejected with `PROTOCOL_ERROR`: a script received
      FROM a peer must fill in data and report status, never drive the
      local buses, pins, or clock (trust is symmetric; a buggy
      or hostile peer must not stall or actuate this side). Unknown
      non-critical opcodes still skip for forward compatibility. Hosts
      that decode peer responses enable this; device-side script
      execution keeps the full set.
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

    /*! @brief True when the script carried a `ReportError` / `ReportComplete`. */
    bool statusReported(void) const
    {
        return _status_reported;
    }
    m5::hal::v2::error::error_t reportedStatus(void) const
    {
        return _reported_status;
    }
    /*! @brief Offset carried by `ReportError` (0 for `ReportComplete`). */
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
    m5::hal::v2::result_t<void> writeResponse(data::Sink& dst, m5::hal::v2::error::error_t status);
    bool hasBinding(types::bus_kind_t kind, uint8_t bus_id) const;
    m5::hal::v2::result_t<void> streamTransferChunk(types::bus_kind_t kind, uint8_t bus_id, data::ConstDataSpan meta,
                                                    data::ConstDataSpan src, data::DataSpan dst, size_t& actual_tx_len,
                                                    size_t& actual_rx_len);

    static constexpr size_t kMaxBindingSlots = 8;

private:
    struct Slot {
        uint8_t id = kDiscardStoreId;
        size_t len = 0;
        memory::TempBuffer buf;
    };

    m5::hal::v2::result_t<Slot*> allocStore(uint8_t store_id, size_t size);

    m5::hal::v2::result_t<void> dispatch(uint8_t opcode, data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opDelay(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opBusConfigure(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opBusTransfer(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opBusStreamTransfer(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opGpio(uint8_t opcode, data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opGpioSubscribe(uint8_t opcode, data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opGpioAllowlist(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opGpioPortRead(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opGpioPortWrite(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opBusCreate(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opBusRelease(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opEvtGpioState(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opBusBeginTransaction(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opBusEndTransaction(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opStoreData(data::ConstDataSpan payload);
    m5::hal::v2::result_t<void> opReport(uint8_t opcode, data::ConstDataSpan payload);

    // Shared instruction loop behind run()/runEvent(); state resets stay
    // in the entry points.
    m5::hal::v2::result_t<size_t> runLoop(data::Source& script);
    detail::RunnerBusBinding* binding(types::bus_kind_t kind, uint8_t bus_id);
    const detail::RunnerBusBinding* binding(types::bus_kind_t kind, uint8_t bus_id) const;
    m5::hal::v2::result_t<void> registerBinding(types::bus_kind_t kind, uint8_t bus_id, const detail::RunnerBusOps& ops,
                                                void* main, void* tx, void* rx);
    void unregisterBinding(types::bus_kind_t kind, uint8_t bus_id);

    memory::Allocator* _alloc                            = nullptr;
    detail::RunnerBusBinding _bindings[kMaxBindingSlots] = {};
    gpio::GPIOGroup* _gpio_group                         = nullptr;
    delay_fn_t _delay_fn                                 = nullptr;  // nullptr -> m5::utility::delay
    gpio_subscribe_fn_t _gpio_subscribe_fn               = nullptr;
    void* _gpio_subscribe_ctx                            = nullptr;
    gpio_event_fn_t _gpio_event_fn                       = nullptr;
    void* _gpio_event_ctx                                = nullptr;
    gpio_mode_fn_t _gpio_mode_fn                         = nullptr;
    void* _gpio_mode_ctx                                 = nullptr;

    bus_create_fn_t _bus_create_fn           = nullptr;
    void* _bus_create_ctx                    = nullptr;
    stream_transfer_fn_t _stream_transfer_fn = nullptr;
    void* _stream_transfer_ctx               = nullptr;
    gpio_allowlist_fn_t _gpio_allowlist_fn   = nullptr;
    void* _gpio_allowlist_ctx                = nullptr;

    Slot _slots[kMaxStoreSlots];
    bool _status_reported                        = false;
    m5::hal::v2::error::error_t _reported_status = m5::hal::v2::error::error_t::OK;
    size_t _reported_offset                      = 0;
    size_t _last_offset                          = 0;
    size_t _unknown_skipped                      = 0;
    bool _receive_only                           = false;  // reject executable opcodes
    bool _event_mode                             = false;  // runEvent(): store/report are ignored
};

}  // namespace m5::hal::v2::bytecode

#endif
