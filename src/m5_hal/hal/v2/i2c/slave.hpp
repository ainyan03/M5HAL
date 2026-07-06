// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_I2C_SLAVE_HPP_
#define M5_HAL_HAL_V2_I2C_SLAVE_HPP_

#include "../bus/bus.hpp"
#include "../data.hpp"
#include "../data/memory.hpp"
#include "../data/stream.hpp"
#include "../error.hpp"
#include "../runtime/runtime.hpp"
#include "../service/service.hpp"
#include "../types.hpp"

#include <M5Utility.hpp>

#include <algorithm>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace m5::hal::v2::i2c {

enum class TxUnderrun : uint8_t { Fill, Stretch };

struct SlaveBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_scl = -1;
    types::gpio_number_t pin_sda = -1;
    uint16_t address             = 0;
    bool address_is_10bit        = false;
    uint32_t timeout_ms          = 1000;
    TxUnderrun tx_underrun       = TxUnderrun::Fill;
    uint8_t tx_fill_byte         = 0xFF;
    uint32_t stretch_timeout_ms  = 100;

    constexpr SlaveBusConfig(void) : bus::IBusConfig{types::bus_kind_t::I2C}
    {
    }
};

class SlaveLineDriver {
public:
    virtual ~SlaveLineDriver() = default;

    virtual bool readScl() const           = 0;
    virtual bool readSda() const           = 0;
    virtual void pullSdaLow(bool pull_low) = 0;
    virtual void pullSclLow(bool pull_low) = 0;
};

struct ISlaveBus : public bus::IBus {
    const SlaveBusConfig &getConfig(void) const override
    {
        return _config;
    }

    virtual result_t<void> init(const SlaveBusConfig &cfg)                                                       = 0;
    virtual result_t<void> beginTransaction(bus::IAccessor *owner, uint32_t timeout_ms = types::TIMEOUT_FOREVER) = 0;
    virtual result_t<void> endTransaction(bus::IAccessor *owner)                                                 = 0;
    virtual result_t<size_t> read(bus::IAccessor *owner, data::DataSpan dst)                                     = 0;
    virtual result_t<size_t> write(bus::IAccessor *owner, data::ConstDataSpan src)                               = 0;
    virtual result_t<size_t> readableBytes(bus::IAccessor *owner)                                                = 0;
    virtual result_t<bool> transactionComplete(bus::IAccessor *owner)                                            = 0;
    virtual service::IService *service()                                                                         = 0;

    // Block the calling consumer until the backend has activity for `owner`'s open
    // transaction (RX bytes became readable, the transaction completed, or TX reply
    // room opened), or `timeout_ms` elapses. This is the event-driven seam serve()
    // uses instead of a fixed poll delay: a backend with an ISR overrides it to wake
    // on a notification, so a fast master is drained promptly (the RX ring does not
    // stay full into the STOP, which would drop the final FIFO load). The default is
    // a 1 ms poll, preserving the prior polling behavior for backends without an ISR
    // wake (software / v2 driver). Never blocks the bus: the backend keeps advancing
    // under its ISR / service tick while the consumer waits here.
    //
    // Returns `true` when the backend confirmed an activity wake before the timeout,
    // `false` when the wait elapsed with no confirmed signal. The poll fallback cannot
    // observe a wake reason, so it always reports `false`; either way the caller must
    // re-check state (readableBytes / transactionComplete) -- the bool is an advisory
    // hint, not a substitute for that check.
    virtual result_t<bool> waitForActivity(bus::IAccessor *owner, uint32_t timeout_ms)
    {
        (void)owner;
        (void)timeout_ms;
        runtime::delayMs(1);
        return false;
    }

protected:
    SlaveBusConfig _config;
};

class SlaveStreamAccessor : public bus::IAccessor, public data::StreamReader, public data::StreamWriter {
public:
    SlaveStreamAccessor(ISlaveBus &bus) : bus::IAccessor{bus}
    {
    }

    const bus::IAccessConfig &getConfig(void) const override
    {
        return _access_config;
    }
    ISlaveBus &getBus(void) const
    {
        return static_cast<ISlaveBus &>(bus::IAccessor::getBus());
    }

    result_t<void> beginTransaction(uint32_t timeout_ms = types::TIMEOUT_FOREVER);

    result_t<void> endTransaction(void);

    result_t<size_t> read(data::DataSpan dst) override;

    result_t<size_t> write(data::ConstDataSpan src) override;

    result_t<size_t> readableBytes(void) override;

    // True once the master has finished the open transaction (issued its STOP).
    // Servicing a read means composing the reply with write() and then keeping
    // the transaction open until it completes: the backend streams the reply out
    // of the open transaction across the read (refilling the TX FIFO under clock
    // stretch), so ending early would discard the reply mid-read. Poll this after
    // write() before endTransaction(). A pure write reports complete on its STOP.
    result_t<bool> transactionComplete(void);

    // Park until the backend signals activity (RX/TX/STOP) for this accessor, or the
    // timeout elapses -- the event-driven replacement for a fixed poll delay in a
    // serve/drain loop. On a backend with an ISR (the espidf LL stretch path) this
    // wakes on the next interrupt, so a serve loop drains promptly enough that the RX
    // ring is not full at the master's STOP (the small-write tail-drop fix). On
    // backends without an ISR wake it falls back to a short poll (see the base).
    //
    // Returns `true` on a confirmed activity wake, `false` on timeout (and always
    // `false` on the poll fallback); `INVALID_ARGUMENT` when this accessor is not
    // bound. The bool is advisory -- re-check readableBytes / transactionComplete
    // regardless (a custom serve loop uses it only to tell "woke early" from "timed
    // out", e.g. to decide whether to keep waiting).
    result_t<bool> waitForActivity(uint32_t timeout_ms)
    {
        if (!isBound()) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        return getBus().waitForActivity(this, timeout_ms);
    }

    // Serve exactly one transaction with the SPI-`transfer`-style Source/Sink model:
    // received (write) bytes are pushed into `dst` (a Sink), reply (read) bytes are
    // pulled from `src` (a Source). Blocks until the master completes the transaction
    // (its STOP), or `timeout_ms` elapses with no progress -- whether waiting for the
    // transaction to start OR for a stuck Sink/Source mid-transaction (the stall
    // escape below); returns the number of received bytes (what the master wrote into
    // `dst`); a read-only transaction returns 0. Either argument may be null (write-only / read-only).
    // This is the slave counterpart of MasterAccessor::transfer(Source*, Sink*); the
    // Source/Sink virtuals ARE the (internal) callbacks -- no separate callback API.
    //
    // Back-pressure is automatic and lives in the backend, not here: when the Sink is
    // full (reserve() returns a short/zero span) this loop stops draining, the RX ring
    // backs up, and the backend clock-stretches the master until space frees; when the
    // Source lags the TX ring empties and the backend stretches on TX underrun. The
    // loop just keeps draining/filling -- no byte is dropped while a Sink/Source can
    // eventually make progress (unless a finite timeout escapes a stall, below).
    // Execution model matches RegMap serve(): it blocks (poll + delayMs / event wait)
    // for an app loop, NOT a single-thread ServiceRunner tick.
    //
    // CONTRACT / footguns:
    //   * The `dst` Sink must accept what the master writes, OR a finite timeout_ms must
    //     be given. A bounded Sink that fills mid-write (or a null `dst` while the master
    //     writes) makes reserve() return 0, so this loop stops draining and the backend
    //     holds the master under the RX_FULL stretch. With timeout_ms == TIMEOUT_FOREVER
    //     (the default) that hold is indefinite -- zero-loss, but a too-small Sink wedges
    //     the bus until it frees / re-init. With a FINITE timeout_ms it is a no-progress
    //     (stall) deadline: if nothing is drained/filled for that long serve() ESCAPES --
    //     it drains the rest of the write into a throwaway buffer so the stretch lifts and
    //     the master finishes, then returns TIMEOUT_ERROR (the bytes that fit are already
    //     in the Sink; the dropped tail is also counted by the backend rxOverflowCount).
    //     Size the Sink to the largest write for zero-loss, or pass a finite timeout to
    //     fail-and-recover instead of wedging the bus.
    //   * On the espidf LL backend serve() parks (via waitForActivity) on a DEDICATED
    //     binary semaphore owned by the backend -- NOT the calling task's direct
    //     notification -- so an app may use xTaskNotify on its own task concurrently
    //     with serve() without the wakeups racing (no shared notification slot).
    result_t<size_t> serve(data::Source *src, data::Sink *dst, uint32_t timeout_ms = types::TIMEOUT_FOREVER);

private:
    struct AccessConfig : public bus::IAccessConfig {
        constexpr AccessConfig(void) : bus::IAccessConfig{types::bus_kind_t::I2C}
        {
        }
    } _access_config;
};

// Register-file slave accessor: the common "emulate a register device" policy
// composed on top of SlaveStreamAccessor.
//
// Holds an application-owned register file (data::DataSpan), an 8-bit register
// pointer with auto-increment, and optional onRead / onWrite hooks. The wire
// semantics it services (validated on 2-board HIL across WTR / SPLIT / WTEST x
// {100,400,800} kHz):
//   - The FIRST byte of a transaction is the register pointer. It persists
//     across transactions, so a SPLIT read (register write, STOP, then a pure
//     read) resolves against the pointer set by the preceding write.
//   - Subsequent write bytes store into reg_file[pointer + 0], [pointer + 1],
//     ... (auto-increment) and fire onWrite(reg, val).
//   - A read returns reg_file[pointer + 0], [pointer + 1], ... (auto-increment,
//     8-bit wrap); onRead(reg) overrides the byte for live / computed values
//     (generated just-in-time while the master is held under clock stretch --
//     the differentiating point of an M5HAL stretch-capable slave).
//
// Execution model: serve() blocks (poll + delayMs) and is meant for an espidf-
// style app loop ("while (running) acc.serve();") where the bus advances under
// the backend ISR while serve() waits. Do NOT call serve() from a single-thread
// ServiceRunner tick (the bus would never advance -- deadlock). For that model,
// drive the transaction-step API (beginExchange / ingest / composeReply) from
// your own tick loop instead; serve() merely composes those steps in the
// validated order.
class SlaveRegMapAccessor {
public:
    using OnReadFn  = uint8_t (*)(uint8_t reg, void *ctx);
    using OnWriteFn = void (*)(uint8_t reg, uint8_t value, void *ctx);

    // Composing a full register window per read keeps the held read stretch
    // short; 64 matches the backend's TX FIFO staging capacity (kTxCapacity).
    static constexpr size_t kReplyWindowBytes = 64;
    // One read() drains this many bytes at a time; a multi-byte write is drained
    // across several reads in serve()'s loop, so it is not capped by THIS chunk.
    // NOTE: the backend RX is a kRxCapacity-deep power-of-two ring (64 on the
    // software backend; 32 on the espidf LL backend). It bounds the UNREAD backlog,
    // not the per-transaction total -- as long as serve() keeps draining reads, a single
    // transaction may write far more than kRxCapacity bytes (e.g. a full 256-byte
    // register file). A backend rxOverflowCount() increment means the consumer fell
    // behind and a byte was dropped; serve()'s drain loop keeps the backlog small.
    static constexpr size_t kReadChunkBytes = 64;

    SlaveRegMapAccessor(ISlaveBus &bus, data::DataSpan reg_file) : _stream{bus}, _reg_file{reg_file}
    {
    }

    // --- Application-side register access (independent of the wire). ---------
    // getRegister / setRegister read and write the backing store directly; they
    // do NOT fire the hooks (the hooks model the master's view of the wire).
    uint8_t getRegister(uint8_t reg) const
    {
        return (_reg_file.data != nullptr && reg < _reg_file.size) ? _reg_file.data[reg] : uint8_t{0};
    }
    void setRegister(uint8_t reg, uint8_t value)
    {
        if (_reg_file.data != nullptr && reg < _reg_file.size) {
            _reg_file.data[reg] = value;
        }
    }
    uint8_t pointer(void) const
    {
        return _pointer;
    }

    // --- Hooks (function pointer + void* ctx; no std::function, house style). -
    // onRead(reg) supplies a just-in-time byte for the read window (live /
    // computed values). onWrite(reg, val) is the master-write side-effect
    // (command registers); it fires after the byte is stored.
    void setOnRead(OnReadFn cb, void *ctx)
    {
        _on_read     = cb;
        _on_read_ctx = ctx;
    }
    void setOnWrite(OnWriteFn cb, void *ctx)
    {
        _on_write     = cb;
        _on_write_ctx = ctx;
    }

    // --- Blocking convenience: serve exactly one transaction. ----------------
    // Returns OK once the transaction is served and closed. A hard (unexpected)
    // error from any backend poll stops the exchange early but still closes the
    // transaction, and is returned -- the happy path is unchanged, but a real
    // failure (owner mismatch, bus released) is no longer reported as success. A
    // short read (0 bytes, the normal stream short-read) is not an error.
    result_t<void> serve(uint32_t timeout_ms = types::TIMEOUT_FOREVER);

    // --- Transaction-step API (pure register-map logic; no bus I/O). ---------
    // serve() composes these in the validated order. They are also the building
    // blocks for a tick-driven ServiceRunner loop (where serve()'s blocking
    // poll cannot be used) and the seam exercised by the native unit tests.

    // Begin a fresh transaction window: the next ingested byte becomes the
    // register pointer. The pointer value itself persists from the prior
    // transaction (SPLIT relies on this).
    void beginExchange(void);

    // Feed received bytes (write phase) in wire order. The first byte of the
    // exchange sets the register pointer; each subsequent byte stores into
    // reg_file[pointer + offset] (auto-increment) and fires onWrite.
    void ingest(data::ConstDataSpan src);

    // Compose up to dst.size reply bytes from the register pointer window
    // (auto-increment with 8-bit wrap; onRead overrides per byte). Returns the
    // number of bytes composed (== dst.size when dst.data is non-null).
    size_t composeReply(data::DataSpan dst);

    // Access to the composed stream accessor (advanced: custom serve loops).
    SlaveStreamAccessor &stream(void);

private:
    uint8_t readByte(uint8_t reg) const;

    void writeByte(uint8_t reg, uint8_t value);

    SlaveStreamAccessor _stream;
    data::DataSpan _reg_file;
    uint8_t _pointer      = 0;
    bool _pointer_set     = false;
    uint8_t _write_offset = 0;
    OnReadFn _on_read     = nullptr;
    void *_on_read_ctx    = nullptr;
    OnWriteFn _on_write   = nullptr;
    void *_on_write_ctx   = nullptr;
};

class ScopedSlaveTransaction {
public:
    explicit ScopedSlaveTransaction(SlaveStreamAccessor &accessor, uint32_t timeout_ms = types::TIMEOUT_FOREVER)
        : _accessor{&accessor}
    {
        auto r = _accessor->beginTransaction(timeout_ms);
        if (!r.has_value()) {
            _error    = r.error();
            _accessor = nullptr;
        }
    }
    ~ScopedSlaveTransaction()
    {
        if (_accessor != nullptr) {
            (void)_accessor->endTransaction();
        }
    }
    ScopedSlaveTransaction(const ScopedSlaveTransaction &)            = delete;
    ScopedSlaveTransaction &operator=(const ScopedSlaveTransaction &) = delete;
    ScopedSlaveTransaction(ScopedSlaveTransaction &&)                 = delete;
    ScopedSlaveTransaction &operator=(ScopedSlaveTransaction &&)      = delete;

    bool has_error(void) const
    {
        return _accessor == nullptr;
    }
    /*! @brief Success view: `true` when the scope acquired (== `!has_error()`). */
    bool ok(void) const
    {
        return !has_error();
    }
    error::error_t error(void) const
    {
        return _error;
    }

private:
    SlaveStreamAccessor *_accessor = nullptr;
    error::error_t _error          = error::error_t::OK;
};

class ScopedSlaveServiceRegistration {
public:
    ScopedSlaveServiceRegistration() = default;
    ScopedSlaveServiceRegistration(service::ServiceRunner &runner, ISlaveBus &driver)
    {
        (void)registerTo(runner, driver);
    }
    ~ScopedSlaveServiceRegistration()
    {
        release();
    }

    ScopedSlaveServiceRegistration(const ScopedSlaveServiceRegistration &)            = delete;
    ScopedSlaveServiceRegistration &operator=(const ScopedSlaveServiceRegistration &) = delete;

    ScopedSlaveServiceRegistration(ScopedSlaveServiceRegistration &&other) noexcept
        : _runner{other._runner}, _service{other._service}
    {
        other._runner  = nullptr;
        other._service = nullptr;
    }
    ScopedSlaveServiceRegistration &operator=(ScopedSlaveServiceRegistration &&other) noexcept
    {
        if (this != &other) {
            release();
            _runner        = other._runner;
            _service       = other._service;
            other._runner  = nullptr;
            other._service = nullptr;
        }
        return *this;
    }

    result_t<void> registerTo(service::ServiceRunner &runner, ISlaveBus &driver);

    void release();

    bool registered() const;

private:
    service::ServiceRunner *_runner = nullptr;
    service::IService *_service     = nullptr;
};

class SlaveBus_software : public ISlaveBus, public service::IService {
public:
    static constexpr size_t kMaxObservedMasterAcks = 16;
    static constexpr size_t kMaxTransactions       = 4;
    static constexpr size_t kRxCapacity            = 64;
    static constexpr size_t kTxCapacity            = 64;
    // rx[] and tx[] are indexed as rings via (idx & (cap - 1)); require powers of two.
    static_assert((kRxCapacity & (kRxCapacity - 1)) == 0, "kRxCapacity must be a power of two");
    static_assert((kTxCapacity & (kTxCapacity - 1)) == 0, "kTxCapacity must be a power of two");

    SlaveBus_software() = default;

    result_t<void> init(SlaveLineDriver &lines, const SlaveBusConfig &config);

    result_t<void> init(const SlaveBusConfig &cfg) override;

    result_t<void> release(void) override;

    service::IService *service() override;

    result_t<void> beginTransaction(bus::IAccessor *owner, uint32_t timeout_ms = 0) override;

    result_t<void> endTransaction(bus::IAccessor *owner) override;

    result_t<size_t> read(bus::IAccessor *owner, data::DataSpan dst) override;

    result_t<size_t> write(bus::IAccessor *owner, data::ConstDataSpan src) override;

    result_t<size_t> readableBytes(bus::IAccessor *owner) override;

    result_t<bool> transactionComplete(bus::IAccessor *owner) override;

    void setMaxAckedWriteBytes(size_t count);

    size_t maxAckedWriteBytes() const;

    size_t masterAckCount() const;

    bool masterAckAt(size_t index) const;

    size_t stopCount() const;

    // Number of received write bytes dropped because the RX ring's UNREAD backlog
    // reached kRxCapacity, i.e. the consumer did not drain reads fast enough and a
    // further byte would have overwritten the oldest unread one. With timely reads a
    // single transaction can receive far more than kRxCapacity bytes total; non-zero
    // here means consumer lag (or no read() at all), not a hard per-transaction cap.
    size_t rxOverflowCount() const;

    service::ServicePoll serviceImpl(const service::ServiceContext &ctx) override;

private:
    enum class State : uint8_t { Idle, Receive, AckSetup, Ack, Transmit, ReadMasterAck, WaitTx, Ignore };

    struct Transaction {
        bool in_use             = false;
        bool complete           = false;
        bool opened             = false;
        uint8_t rx[kRxCapacity] = {};
        size_t rx_size          = 0;
        size_t rx_read          = 0;
        uint8_t tx[kTxCapacity] = {};
        size_t tx_size          = 0;
        size_t tx_read          = 0;
        uint32_t seq            = 0;
    };

    void resetProtocol();

    void startCondition();

    void stopCondition();

    Transaction *allocateTransaction();

    Transaction *oldestOpenableTransaction();

    void discardTransaction(Transaction &txn);

    bool isOpenOwner(bus::IAccessor *owner) const;

    static size_t readableBytesOf(const Transaction &txn);

    size_t currentRxSize() const;

    bool storeReceivedByte(uint8_t value);

    void storeMasterAck(bool ack);

    bool txAvailableForCurrent() const;

    uint8_t nextTxByte();

    void driveTxBit();

    void beginStretch(service::fast_tick_t now_tick);

    bool stretchExpired(service::fast_tick_t now_tick) const;

    void resumeStretchIfReady();

    SlaveLineDriver *_lines = nullptr;
    Transaction _transactions[kMaxTransactions];
    Transaction *_current                     = nullptr;
    Transaction *_open                        = nullptr;
    bus::IAccessor *_open_owner               = nullptr;
    State _state                              = State::Idle;
    bool _prev_scl                            = true;
    bool _prev_sda                            = true;
    bool _is_address                          = true;
    bool _matched                             = false;
    bool _read_phase                          = false;
    bool _drive_ack                           = false;
    bool _master_ack                          = false;
    bool _stretching                          = false;
    uint8_t _byte                             = 0;
    uint8_t _tx_byte                          = 0xFF;
    uint8_t _bit_count                        = 0;
    size_t _stop_count                        = 0;
    size_t _rx_overflow_count                 = 0;
    size_t _max_acked_write_bytes             = static_cast<size_t>(-1);
    size_t _master_ack_count                  = 0;
    bool _master_acks[kMaxObservedMasterAcks] = {};
    uint32_t _next_seq                        = 1;
    service::fast_tick_t _stretch_start       = 0;
    service::fast_tick_t _stretch_budget      = 0;
};

}  // namespace m5::hal::v2::i2c

#endif  // M5_HAL_HAL_V2_I2C_SLAVE_HPP_
