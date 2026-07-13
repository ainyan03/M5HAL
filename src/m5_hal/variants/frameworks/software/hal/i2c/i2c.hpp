// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_I2C_I2C_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_I2C_I2C_HPP

#include "../../../../../hal/v2/gpio/port.hpp"
#include "../../../../../hal/v2/i2c/i2c.hpp"
#include "../../../../../hal/v2/m5_hal.hpp"
#include "../../../../../hal/v2/service/completion_gate.hpp"
#include "../../../../../hal/v2/service/service.hpp"

#include <atomic>

// I2C bit-bang implementation. Drives SCL / SDA through the
// `m5::hal::v2::gpio::Pin` value type; callers populate
// `IBusConfig::pin_scl` / `pin_sda` with `gpio_number_t` values.
// `Bus::init` resolves them via `m5::hal::v2::M5_Hal.Gpio.getPin(num)`
// (global lookup through the `M5HALCore` singleton) and stores the
// resulting `IPort` / `Pin` in members. Any `gpio_number_t` is
// accepted, including pins behind an I/O expander.
namespace m5::variants::frameworks::software::hal::v2::i2c {

using namespace ::m5::hal::v2;  // resolve unqualified types::/bus:: refs

namespace detail {

class MasterLineDriver {
public:
    virtual ~MasterLineDriver() = default;

    virtual void writeSclHigh()      = 0;
    virtual void writeSclLow()       = 0;
    virtual void writeSda(bool high) = 0;
    virtual bool readScl() const     = 0;
    virtual bool readSda() const     = 0;
};

// Timing pair for the detail services. fromConfig() stores mixed
// duration units explicitly: `half_period` is nanoseconds for sub-us
// hardware-clock precision, while `timeout` is microseconds so
// millisecond-scale wire timeouts fit in uint32_t. The synchronous
// transfer path converts both once with serviceTimingToTicks() and pumps
// raw fastTick() counts instead. Unit tests that drive detail services
// directly may still fill both fields with arbitrary comparable ticks.
struct MasterTiming {
    ::m5::hal::v2::service::fast_tick_t half_period = 5000;
    ::m5::hal::v2::service::fast_tick_t timeout     = 1000000u;

    static constexpr uint32_t kNsecPerSec  = 1000000000u;
    static constexpr uint32_t kUsecPerMsec = 1000u;
    static constexpr uint32_t kMaxUsec     = 0xFFFFFFFFu;

    static ::m5::hal::v2::result_t<MasterTiming> fromConfig(const ::m5::hal::v2::i2c::MasterAccessConfig& cfg);
};

class MasterServiceTiming {
public:
    enum class ClockWaitResult : uint8_t { Released, Waiting, Timeout };

    void reset(::m5::hal::v2::service::fast_tick_t now_tick);

    ::m5::hal::v2::service::fast_tick_t dueTick() const;
    template <typename State>
    void scheduleAfterHalfFromNow(const MasterTiming& timing, ::m5::hal::v2::service::fast_tick_t now_tick,
                                  State& state, State next)
    {
        _due  = now_tick + timing.half_period;
        state = next;
    }
    template <typename State>
    void scheduleNextHalf(const MasterTiming& timing, ::m5::hal::v2::service::fast_tick_t now_tick, State& state,
                          State next)
    {
        // Preserve the ideal clock phase for small service/GPIO overhead, but
        // resync after a large delay so the bus never emits catch-up bursts.
        const auto next_due = _due + timing.half_period;
        _due  = ::m5::hal::v2::service::hasReached(now_tick, next_due) ? now_tick + timing.half_period : next_due;
        state = next;
    }
    template <typename State>
    void scheduleNow(::m5::hal::v2::service::fast_tick_t now_tick, State& state, State next)
    {
        _due  = now_tick;
        state = next;
    }
    template <typename State>
    void beginClockStretch(::m5::hal::v2::service::fast_tick_t now_tick, State& state, State wait_state)
    {
        _stretch_start = now_tick;
        _due           = now_tick;
        state          = wait_state;
    }
    ClockWaitResult waitClockHigh(const MasterLineDriver& lines, const MasterTiming& timing,
                                  ::m5::hal::v2::service::fast_tick_t now_tick) const;

private:
    ::m5::hal::v2::service::fast_tick_t _due           = 0;
    ::m5::hal::v2::service::fast_tick_t _stretch_start = 0;
};

class StartConditionService {
public:
    enum class State : uint8_t { Idle, ReleaseScl, WaitClockHigh, PullSdaLow, PullSclLow, Done, Timeout };

    void begin(MasterLineDriver& lines, const MasterTiming& timing, ::m5::hal::v2::service::fast_tick_t now_tick);

    ::m5::hal::v2::service::ServiceResult service(::m5::hal::v2::service::fast_tick_t now_tick);

    State state() const;

    bool done() const;

    ::m5::hal::v2::service::fast_tick_t dueTick() const;

    ::m5::hal::v2::error::error_t error() const;

private:
    MasterLineDriver* _lines = nullptr;
    MasterTiming _timing;
    MasterServiceTiming _clock;
    ::m5::hal::v2::error::error_t _error = ::m5::hal::v2::error::error_t::OK;
    State _state                         = State::Idle;
};

class WriteByteService {
public:
    enum class State : uint8_t {
        RaiseClock    = 0x00,
        LowerClock    = 0x01,
        RaiseAckClock = 0x02,
        WaitClockHigh = 0x03,
        SampleAck     = 0x04,
        Idle          = 0x80,
        Done          = 0x81,
        Nack          = 0xC0,
        Timeout       = 0xC1,
    };

    void begin(MasterLineDriver& lines, const MasterTiming& timing, uint8_t byte,
               ::m5::hal::v2::service::fast_tick_t now_tick);

    void restart(uint8_t byte, ::m5::hal::v2::service::fast_tick_t now_tick);

    ::m5::hal::v2::service::ServiceResult service(::m5::hal::v2::service::fast_tick_t now_tick);

    State state() const;

    bool done() const;

    bool acked() const;

    ::m5::hal::v2::error::error_t error() const;

    uint8_t bitIndex() const;

    ::m5::hal::v2::service::fast_tick_t dueTick() const;

private:
    static constexpr uint8_t kTerminalStateMask = 0x80;
    static constexpr uint8_t kErrorStateMask    = 0x40;

    static constexpr ::m5::hal::v2::service::ServiceResult terminalStateResult(uint8_t state_value);

    bool bitValue() const;

    void scheduleAfterHalf(::m5::hal::v2::service::fast_tick_t now_tick, State next);

    void scheduleAfterHalfFromNow(::m5::hal::v2::service::fast_tick_t now_tick, State next);

    void waitClockHighOrSchedule(::m5::hal::v2::service::fast_tick_t now_tick, State next);

    MasterLineDriver* _lines = nullptr;
    MasterTiming _timing;
    MasterServiceTiming _clock;
    State _state                         = State::Idle;
    State _after_stretch                 = State::Idle;
    ::m5::hal::v2::error::error_t _error = ::m5::hal::v2::error::error_t::OK;
    uint8_t _byte                        = 0;
    uint8_t _bit_index                   = 0;
    uint8_t _bit_mask                    = 0x80;
    bool _acked                          = false;
};

class StopConditionService {
public:
    enum class State : uint8_t {
        Idle,
        PullSdaLow,
        RaiseClock,
        WaitClockHigh,
        ReleaseSda,
        VerifySdaHigh,
        Done,
        BusError,
        Timeout,
    };

    void begin(MasterLineDriver& lines, const MasterTiming& timing, ::m5::hal::v2::service::fast_tick_t now_tick);

    ::m5::hal::v2::service::ServiceResult service(::m5::hal::v2::service::fast_tick_t now_tick);

    State state() const;

    bool done() const;

    ::m5::hal::v2::error::error_t error() const;

    ::m5::hal::v2::service::fast_tick_t dueTick() const;

private:
    void scheduleAfterHalf(::m5::hal::v2::service::fast_tick_t now_tick, State next);

    void waitClockHighOrSchedule(::m5::hal::v2::service::fast_tick_t now_tick, State next);

    MasterLineDriver* _lines = nullptr;
    MasterTiming _timing;
    MasterServiceTiming _clock;
    State _state                         = State::Idle;
    State _after_stretch                 = State::Idle;
    ::m5::hal::v2::error::error_t _error = ::m5::hal::v2::error::error_t::OK;
};

class ReadByteService {
public:
    enum class State : uint8_t {
        RaiseClock    = 0x00,
        SampleBit     = 0x01,
        ReleaseSda    = 0x02,
        RaiseAckClock = 0x03,
        LowerAckClock = 0x04,
        WaitClockHigh = 0x05,
        Idle          = 0x80,
        Done          = 0x81,
        Timeout       = 0xC0,
    };

    void begin(MasterLineDriver& lines, const MasterTiming& timing, bool ack_after_read,
               ::m5::hal::v2::service::fast_tick_t now_tick);

    void restart(bool ack_after_read, ::m5::hal::v2::service::fast_tick_t now_tick);

    ::m5::hal::v2::service::ServiceResult service(::m5::hal::v2::service::fast_tick_t now_tick);

    State state() const;

    bool done() const;

    uint8_t byte() const;

    ::m5::hal::v2::error::error_t error() const;

    uint8_t bitIndex() const;

    ::m5::hal::v2::service::fast_tick_t dueTick() const;

private:
    static constexpr uint8_t kTerminalStateMask = 0x80;
    static constexpr uint8_t kErrorStateMask    = 0x40;

    static constexpr ::m5::hal::v2::service::ServiceResult terminalStateResult(uint8_t state_value);

    void scheduleAfterHalf(::m5::hal::v2::service::fast_tick_t now_tick, State next);

    void scheduleAfterHalfFromNow(::m5::hal::v2::service::fast_tick_t now_tick, State next);

    void waitClockHighOrSchedule(::m5::hal::v2::service::fast_tick_t now_tick, State next);

    MasterLineDriver* _lines = nullptr;
    MasterTiming _timing;
    MasterServiceTiming _clock;
    State _state                         = State::Idle;
    State _after_stretch                 = State::Idle;
    ::m5::hal::v2::error::error_t _error = ::m5::hal::v2::error::error_t::OK;
    uint8_t _byte                        = 0;
    uint8_t _bit_index                   = 0;
    uint8_t _bit_mask                    = 0x80;
    bool _ack_after_read                 = false;
};

class MasterTransactionService {
public:
    enum class Operation : uint8_t { Idle, Start, WriteByte, ReadByte, Stop, Address, WriteBuffer, ReadBuffer };
    enum class Phase : uint8_t { Idle, Start, Write, Read };

    void beginStart(MasterLineDriver& lines, const MasterTiming& timing, ::m5::hal::v2::service::fast_tick_t now_tick);

    void beginWriteByte(MasterLineDriver& lines, const MasterTiming& timing, uint8_t byte,
                        ::m5::hal::v2::service::fast_tick_t now_tick);

    void beginReadByte(MasterLineDriver& lines, const MasterTiming& timing, bool ack_after_read,
                       ::m5::hal::v2::service::fast_tick_t now_tick);

    void beginStop(MasterLineDriver& lines, const MasterTiming& timing, ::m5::hal::v2::service::fast_tick_t now_tick);

    void beginAddress(MasterLineDriver& lines, const MasterTiming& timing, uint8_t addr_byte,
                      ::m5::hal::v2::service::fast_tick_t now_tick);

    void beginWriteBuffer(MasterLineDriver& lines, const MasterTiming& timing, const uint8_t* data, size_t len,
                          ::m5::hal::v2::service::fast_tick_t now_tick);

    void beginReadBuffer(MasterLineDriver& lines, const MasterTiming& timing, uint8_t* data, size_t len, bool last_nack,
                         ::m5::hal::v2::service::fast_tick_t now_tick);

    ::m5::hal::v2::service::ServiceResult service(::m5::hal::v2::service::fast_tick_t now_tick);

    Operation operation() const;

    ::m5::hal::v2::error::error_t error() const;

    uint8_t byte() const;

    bool acked() const;

    size_t transferred() const;

    ::m5::hal::v2::service::fast_tick_t dueTick() const;

private:
    bool ackAfterRead(size_t index) const;

    ::m5::hal::v2::service::ServiceResult serviceAddress(::m5::hal::v2::service::fast_tick_t now_tick);

    ::m5::hal::v2::service::ServiceResult serviceWriteBuffer(::m5::hal::v2::service::fast_tick_t now_tick);

    ::m5::hal::v2::service::ServiceResult serviceReadBuffer(::m5::hal::v2::service::fast_tick_t now_tick);
    template <typename Service>
    ::m5::hal::v2::service::ServiceResult serviceActive(Service& service, ::m5::hal::v2::service::fast_tick_t now_tick)
    {
        auto result = service.service(now_tick);
        if (result == ::m5::hal::v2::service::ServiceResult::Done) {
            _operation = Operation::Idle;
        } else if (result == ::m5::hal::v2::service::ServiceResult::Error) {
            _error     = service.error();
            _operation = Operation::Idle;
        }
        return result;
    }
    template <typename Service>
    ::m5::hal::v2::service::ServiceResult serviceComposite(Service& service,
                                                           ::m5::hal::v2::service::fast_tick_t now_tick)
    {
        auto result = service.service(now_tick);
        if (result == ::m5::hal::v2::service::ServiceResult::Error) {
            _error     = service.error();
            _operation = Operation::Idle;
            _phase     = Phase::Idle;
        }
        return result;
    }

    Operation _operation                 = Operation::Idle;
    Phase _phase                         = Phase::Idle;
    MasterLineDriver* _lines             = nullptr;
    const MasterTiming* _timing          = nullptr;
    ::m5::hal::v2::error::error_t _error = ::m5::hal::v2::error::error_t::OK;
    StartConditionService _start;
    WriteByteService _write;
    ReadByteService _read;
    StopConditionService _stop;
    const uint8_t* _tx_data = nullptr;
    uint8_t* _rx_data       = nullptr;
    size_t _length          = 0;
    size_t _index           = 0;
    uint8_t _byte           = 0;
    bool _last_nack         = true;
};

}  // namespace detail

}  // namespace m5::variants::frameworks::software::hal::v2::i2c

// The user-facing surface (Bus_software / BusConfig_software) lives in the
// kind namespace per the variant naming rule (spec/design/variants.md);
// the bit-bang machinery above stays in the variant namespace as an
// implementation detail.
namespace m5::hal::v2::i2c {

// This variant needs no fields beyond the abstract kind config; the
// empty derivation still gives `init` a variant-owned type, so a
// sibling variant's config cannot be passed by accident.
struct BusConfig_software : public IBusConfig {
    using IBusConfig::IBusConfig;
};

class Bus_software : public IBus, private service::IService {
public:
    ~Bus_software() override;

    result_t<void> init(const BusConfig_software& config);
    result_t<void> release(void) override;

    result_t<void> transfer(bus::IAccessor* owner, const MasterAccessConfig& cfg, const TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override;
    result_t<bus::TransferTotals> waitTransfer(bus::IAccessor* owner, const MasterAccessConfig& cfg) override;
    bool transferBusy(bus::IAccessor* owner) override;

private:
    service::ServicePoll serviceImpl(const service::ServiceContext& ctx) override;
    service::ServiceResult serviceTransfer(const service::ServiceContext& ctx);
    void unregisterTransferService(void);
    void clearTransferState(void);

    gpio::Pin _pin_scl{};
    gpio::Pin _pin_sda{};
    void* _transfer_state           = nullptr;
    bus::IAccessor* _transfer_owner = nullptr;
    std::atomic<bool> _transfer_registered{false};
    service::CompletionGate _transfer_gate;
    error::error_t _transfer_error = error::error_t::OK;
    bus::TransferTotals _transfer_totals{};
};

// Facade backend selection: i2c::Bus::init(BusConfig_software) -> Bus_software.
template <>
struct BackendFor<BusConfig_software> {
    using type = Bus_software;
};

// software backend factory: builds a bit-bang Bus_software from a
// LogicalBusConfig's pins. M5HALCore wires this into i2c::BusView (the logical
// acquire path). The software variant is always present, so this is the
// universal software fallback used when a bus is not (or not yet) on hardware.
inline IBus* makeSoftwareBackendForI2C(const LogicalBusConfig& logical)
{
    auto* backend = new (std::nothrow) Bus_software();
    if (backend == nullptr) {
        return nullptr;
    }
    BusConfig_software cfg;
    cfg.pin_scl = logical.pin_scl;
    cfg.pin_sda = logical.pin_sda;
    auto r      = backend->init(cfg);
    if (!r.has_value()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

}  // namespace m5::hal::v2::i2c

#endif
