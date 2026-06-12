// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_I2C_I2C_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_I2C_I2C_HPP

#include "../../../../../hal/v1/gpio/port.hpp"
#include "../../../../../hal/v1/i2c/i2c.hpp"
#include "../../../../../hal/v1/m5_hal.hpp"
#include "../../../../../hal/v1/service/service.hpp"

// I2C bit-bang implementation. Drives SCL / SDA through the
// `m5::hal::v1::gpio::Pin` value type; callers populate
// `IBusConfig::pin_scl` / `pin_sda` with `gpio_number_t` values.
// `Bus::init` resolves them via `m5::hal::v1::M5_Hal.Gpio.getPin(num)`
// (global lookup through the `M5HALCore` singleton) and stores the
// resulting `IPort` / `Pin` in members. Any `gpio_number_t` is
// accepted, including pins behind an I/O expander.
namespace m5::variants::frameworks::software::hal::v1::i2c {

using namespace ::m5::hal::v1;  // resolve unqualified types::/bus:: refs

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

// Timing pair for the detail services. `half_period` / `timeout` are
// COMPARABLE-TICK quantities in whatever unit the caller feeds
// ServiceContext::now_tick with — the detail services only add them to
// and compare them against that field. fromConfig() builds the pair in
// nanoseconds (for callers that pump nanosecond contexts, e.g. the
// native tests); the synchronous transfer path converts it once with
// serviceTimingToTicks() and pumps raw fastTick() counts instead.
struct MasterTiming {
    ::m5::hal::v1::service::fast_tick_t half_period = 5000;
    ::m5::hal::v1::service::fast_tick_t timeout     = 1000000000u;

    static constexpr uint32_t kNsecPerSec  = 1000000000u;
    static constexpr uint32_t kNsecPerMsec = 1000000u;

    static ::m5::hal::v1::result_t<MasterTiming> fromConfig(const ::m5::hal::v1::i2c::MasterAccessConfig& cfg)
    {
        if (cfg.freq == 0 || cfg.wire_timeout_ms > (::m5::hal::v1::service::kMaxComparableDelayTicks / kNsecPerMsec)) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }

        MasterTiming timing;
        const uint64_t denom = static_cast<uint64_t>(cfg.freq) * 2ull;
        uint64_t half        = (static_cast<uint64_t>(kNsecPerSec) + (denom / 2ull)) / denom;
        if (half == 0) {
            half = 1;
        }
        if (half > ::m5::hal::v1::service::kMaxComparableDelayTicks) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
        timing.half_period = static_cast<::m5::hal::v1::service::fast_tick_t>(half);
        timing.timeout     = static_cast<::m5::hal::v1::service::fast_tick_t>(cfg.wire_timeout_ms * kNsecPerMsec);
        return timing;
    }
};

class MasterServiceTiming {
public:
    enum class ClockWaitResult : uint8_t { Released, Waiting, Timeout };

    void reset(::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _due           = now_tick;
        _stretch_start = now_tick;
    }
    ::m5::hal::v1::service::fast_tick_t dueTick() const
    {
        return _due;
    }
    template <typename State>
    void scheduleAfterHalfFromNow(const MasterTiming& timing, ::m5::hal::v1::service::fast_tick_t now_tick,
                                  State& state, State next)
    {
        _due  = now_tick + timing.half_period;
        state = next;
    }
    template <typename State>
    void scheduleNextHalf(const MasterTiming& timing, ::m5::hal::v1::service::fast_tick_t now_tick, State& state,
                          State next)
    {
        // Preserve the ideal clock phase for small service/GPIO overhead, but
        // resync after a large delay so the bus never emits catch-up bursts.
        const auto next_due = _due + timing.half_period;
        _due  = ::m5::hal::v1::service::hasReached(now_tick, next_due) ? now_tick + timing.half_period : next_due;
        state = next;
    }
    template <typename State>
    void scheduleNow(::m5::hal::v1::service::fast_tick_t now_tick, State& state, State next)
    {
        _due  = now_tick;
        state = next;
    }
    template <typename State>
    void beginClockStretch(::m5::hal::v1::service::fast_tick_t now_tick, State& state, State wait_state)
    {
        _stretch_start = now_tick;
        _due           = now_tick;
        state          = wait_state;
    }
    ClockWaitResult waitClockHigh(const MasterLineDriver& lines, const MasterTiming& timing,
                                  ::m5::hal::v1::service::fast_tick_t now_tick) const
    {
        if (lines.readScl()) {
            return ClockWaitResult::Released;
        }
        if (::m5::hal::v1::service::elapsedTicks(now_tick, _stretch_start) >= timing.timeout) {
            return ClockWaitResult::Timeout;
        }
        return ClockWaitResult::Waiting;
    }

private:
    ::m5::hal::v1::service::fast_tick_t _due           = 0;
    ::m5::hal::v1::service::fast_tick_t _stretch_start = 0;
};

class StartConditionService : public ::m5::hal::v1::service::IService {
public:
    enum class State : uint8_t { Idle, ReleaseScl, WaitClockHigh, PullSdaLow, PullSclLow, Done, Timeout };

    void begin(MasterLineDriver& lines, const MasterTiming& timing, ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _lines  = &lines;
        _timing = timing;
        _error  = ::m5::hal::v1::error::error_t::OK;
        _state  = State::ReleaseScl;
        _clock.reset(now_tick);
    }

    ::m5::hal::v1::service::ServiceResult service(const ::m5::hal::v1::service::ServiceContext& ctx) override
    {
        using ::m5::hal::v1::service::hasReached;
        if (_lines == nullptr || _state == State::Idle) {
            return ::m5::hal::v1::service::ServiceResult::Idle;
        }
        if (_state == State::Done) {
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        if (_state == State::Timeout) {
            return ::m5::hal::v1::service::ServiceResult::Error;
        }
        if (!hasReached(ctx.now_tick, _clock.dueTick())) {
            return ::m5::hal::v1::service::ServiceResult::Idle;
        }

        switch (_state) {
            case State::ReleaseScl:
                _lines->writeSclHigh();
                if (_lines->readScl()) {
                    _clock.scheduleAfterHalfFromNow(_timing, ctx.now_tick, _state, State::PullSdaLow);
                } else {
                    _clock.beginClockStretch(ctx.now_tick, _state, State::WaitClockHigh);
                }
                break;
            case State::WaitClockHigh:
                switch (_clock.waitClockHigh(*_lines, _timing, ctx.now_tick)) {
                    case MasterServiceTiming::ClockWaitResult::Released:
                        _clock.scheduleAfterHalfFromNow(_timing, ctx.now_tick, _state, State::PullSdaLow);
                        break;
                    case MasterServiceTiming::ClockWaitResult::Timeout:
                        _error = ::m5::hal::v1::error::error_t::TIMEOUT_ERROR;
                        _state = State::Timeout;
                        break;
                    case MasterServiceTiming::ClockWaitResult::Waiting:
                        return ::m5::hal::v1::service::ServiceResult::Idle;
                }
                break;
            case State::PullSdaLow:
                _lines->writeSda(false);
                _clock.scheduleAfterHalfFromNow(_timing, ctx.now_tick, _state, State::PullSclLow);
                break;
            case State::PullSclLow:
                _lines->writeSclLow();
                _state = State::Done;
                break;
            case State::Idle:
            case State::Done:
            case State::Timeout:
                break;
        }
        if (_state == State::Done) {
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        if (_state == State::Timeout) {
            return ::m5::hal::v1::service::ServiceResult::Error;
        }
        return ::m5::hal::v1::service::ServiceResult::Progress;
    }

    State state() const
    {
        return _state;
    }
    bool done() const
    {
        return _state == State::Done;
    }
    ::m5::hal::v1::service::fast_tick_t dueTick() const
    {
        return _clock.dueTick();
    }
    ::m5::hal::v1::error::error_t error() const
    {
        return _error;
    }

private:
    MasterLineDriver* _lines = nullptr;
    MasterTiming _timing;
    MasterServiceTiming _clock;
    ::m5::hal::v1::error::error_t _error = ::m5::hal::v1::error::error_t::OK;
    State _state                         = State::Idle;
};

class WriteByteService : public ::m5::hal::v1::service::IService {
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
               ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _lines  = &lines;
        _timing = timing;
        restart(byte, now_tick);
    }

    void restart(uint8_t byte, ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _byte      = byte;
        _bit_index = 0;
        _bit_mask  = 0x80;
        _acked     = false;
        _error     = ::m5::hal::v1::error::error_t::OK;
        _state     = State::RaiseClock;
        _clock.reset(now_tick);
        _lines->writeSda(bitValue());
        _clock.scheduleAfterHalfFromNow(_timing, now_tick, _state, State::RaiseClock);
    }

    ::m5::hal::v1::service::ServiceResult service(const ::m5::hal::v1::service::ServiceContext& ctx) override
    {
        using ::m5::hal::v1::service::hasReached;
        if (_lines == nullptr) {
            return ::m5::hal::v1::service::ServiceResult::Idle;
        }

        const auto state_value = static_cast<uint8_t>(_state);
        if (state_value & kTerminalStateMask) {
            return terminalStateResult(state_value);
        }
        if (!hasReached(ctx.now_tick, _clock.dueTick())) {
            return ::m5::hal::v1::service::ServiceResult::Idle;
        }

        if (_state == State::RaiseClock) {
            _lines->writeSclHigh();
            waitClockHighOrSchedule(ctx.now_tick, State::LowerClock);
            return ::m5::hal::v1::service::ServiceResult::Progress;
        }
        if (_state == State::LowerClock) {
            _lines->writeSclLow();
            _bit_mask >>= 1;
            if (_bit_mask != 0) {
                ++_bit_index;
                _lines->writeSda(bitValue());
                scheduleAfterHalf(ctx.now_tick, State::RaiseClock);
            } else {
                _lines->writeSda(true);
                scheduleAfterHalf(ctx.now_tick, State::RaiseAckClock);
            }
            return ::m5::hal::v1::service::ServiceResult::Progress;
        }

        switch (_state) {
            case State::RaiseClock:
            case State::LowerClock:
                break;
            case State::RaiseAckClock:
                _lines->writeSclHigh();
                waitClockHighOrSchedule(ctx.now_tick, State::SampleAck);
                return ::m5::hal::v1::service::ServiceResult::Progress;
            case State::WaitClockHigh:
                switch (_clock.waitClockHigh(*_lines, _timing, ctx.now_tick)) {
                    case MasterServiceTiming::ClockWaitResult::Released:
                        scheduleAfterHalfFromNow(ctx.now_tick, _after_stretch);
                        return ::m5::hal::v1::service::ServiceResult::Progress;
                    case MasterServiceTiming::ClockWaitResult::Timeout:
                        _error = ::m5::hal::v1::error::error_t::TIMEOUT_ERROR;
                        _state = State::Timeout;
                        return ::m5::hal::v1::service::ServiceResult::Error;
                    case MasterServiceTiming::ClockWaitResult::Waiting:
                        return ::m5::hal::v1::service::ServiceResult::Idle;
                }
                return ::m5::hal::v1::service::ServiceResult::Idle;
            case State::SampleAck:
                _acked = !_lines->readSda();
                _lines->writeSclLow();
                if (_acked) {
                    _state = State::Done;
                    return ::m5::hal::v1::service::ServiceResult::Done;
                } else {
                    _error = ::m5::hal::v1::error::error_t::I2C_NO_ACK;
                    _state = State::Nack;
                    return ::m5::hal::v1::service::ServiceResult::Error;
                }
            case State::Idle:
            case State::Done:
            case State::Nack:
            case State::Timeout:
                break;
        }
        return ::m5::hal::v1::service::ServiceResult::Idle;
    }

    State state() const
    {
        return _state;
    }
    bool done() const
    {
        return _state == State::Done;
    }
    bool acked() const
    {
        return _acked;
    }
    ::m5::hal::v1::error::error_t error() const
    {
        return _error;
    }
    uint8_t bitIndex() const
    {
        return _bit_index;
    }
    ::m5::hal::v1::service::fast_tick_t dueTick() const
    {
        return _clock.dueTick();
    }

private:
    static constexpr uint8_t kTerminalStateMask = 0x80;
    static constexpr uint8_t kErrorStateMask    = 0x40;

    static constexpr ::m5::hal::v1::service::ServiceResult terminalStateResult(uint8_t state_value)
    {
        return (state_value & kErrorStateMask)
                   ? ::m5::hal::v1::service::ServiceResult::Error
                   : ((state_value == static_cast<uint8_t>(State::Done)) ? ::m5::hal::v1::service::ServiceResult::Done
                                                                         : ::m5::hal::v1::service::ServiceResult::Idle);
    }

    bool bitValue() const
    {
        return (_byte & _bit_mask) != 0;
    }
    void scheduleAfterHalf(::m5::hal::v1::service::fast_tick_t now_tick, State next)
    {
        _clock.scheduleNextHalf(_timing, now_tick, _state, next);
    }
    void scheduleAfterHalfFromNow(::m5::hal::v1::service::fast_tick_t now_tick, State next)
    {
        _clock.scheduleAfterHalfFromNow(_timing, now_tick, _state, next);
    }
    void waitClockHighOrSchedule(::m5::hal::v1::service::fast_tick_t now_tick, State next)
    {
        if (_lines->readScl()) {
            scheduleAfterHalf(now_tick, next);
        } else {
            _after_stretch = next;
            _clock.beginClockStretch(now_tick, _state, State::WaitClockHigh);
        }
    }

    MasterLineDriver* _lines = nullptr;
    MasterTiming _timing;
    MasterServiceTiming _clock;
    State _state                         = State::Idle;
    State _after_stretch                 = State::Idle;
    ::m5::hal::v1::error::error_t _error = ::m5::hal::v1::error::error_t::OK;
    uint8_t _byte                        = 0;
    uint8_t _bit_index                   = 0;
    uint8_t _bit_mask                    = 0x80;
    bool _acked                          = false;
};

class StopConditionService : public ::m5::hal::v1::service::IService {
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

    void begin(MasterLineDriver& lines, const MasterTiming& timing, ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _lines  = &lines;
        _timing = timing;
        _error  = ::m5::hal::v1::error::error_t::OK;
        _state  = State::PullSdaLow;
        _clock.reset(now_tick);
    }

    ::m5::hal::v1::service::ServiceResult service(const ::m5::hal::v1::service::ServiceContext& ctx) override
    {
        using ::m5::hal::v1::service::hasReached;
        if (_lines == nullptr || _state == State::Idle) {
            return ::m5::hal::v1::service::ServiceResult::Idle;
        }
        if (_state == State::Done) {
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        if (_state == State::BusError || _state == State::Timeout) {
            return ::m5::hal::v1::service::ServiceResult::Error;
        }
        if (!hasReached(ctx.now_tick, _clock.dueTick())) {
            return ::m5::hal::v1::service::ServiceResult::Idle;
        }

        switch (_state) {
            case State::PullSdaLow:
                _lines->writeSda(false);
                scheduleAfterHalf(ctx.now_tick, State::RaiseClock);
                break;
            case State::RaiseClock:
                _lines->writeSclHigh();
                waitClockHighOrSchedule(ctx.now_tick, State::ReleaseSda);
                break;
            case State::WaitClockHigh:
                switch (_clock.waitClockHigh(*_lines, _timing, ctx.now_tick)) {
                    case MasterServiceTiming::ClockWaitResult::Released:
                        scheduleAfterHalf(ctx.now_tick, _after_stretch);
                        break;
                    case MasterServiceTiming::ClockWaitResult::Timeout:
                        _error = ::m5::hal::v1::error::error_t::TIMEOUT_ERROR;
                        _state = State::Timeout;
                        break;
                    case MasterServiceTiming::ClockWaitResult::Waiting:
                        return ::m5::hal::v1::service::ServiceResult::Idle;
                }
                break;
            case State::ReleaseSda:
                _lines->writeSda(true);
                scheduleAfterHalf(ctx.now_tick, State::VerifySdaHigh);
                break;
            case State::VerifySdaHigh:
                if (_lines->readSda()) {
                    _state = State::Done;
                } else {
                    _error = ::m5::hal::v1::error::error_t::I2C_BUS_ERROR;
                    _state = State::BusError;
                }
                break;
            case State::Idle:
            case State::Done:
            case State::BusError:
            case State::Timeout:
                break;
        }

        if (_state == State::Done) {
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        if (_state == State::BusError || _state == State::Timeout) {
            return ::m5::hal::v1::service::ServiceResult::Error;
        }
        return ::m5::hal::v1::service::ServiceResult::Progress;
    }

    State state() const
    {
        return _state;
    }
    bool done() const
    {
        return _state == State::Done;
    }
    ::m5::hal::v1::error::error_t error() const
    {
        return _error;
    }
    ::m5::hal::v1::service::fast_tick_t dueTick() const
    {
        return _clock.dueTick();
    }

private:
    void scheduleAfterHalf(::m5::hal::v1::service::fast_tick_t now_tick, State next)
    {
        _clock.scheduleAfterHalfFromNow(_timing, now_tick, _state, next);
    }
    void waitClockHighOrSchedule(::m5::hal::v1::service::fast_tick_t now_tick, State next)
    {
        if (_lines->readScl()) {
            scheduleAfterHalf(now_tick, next);
        } else {
            _after_stretch = next;
            _clock.beginClockStretch(now_tick, _state, State::WaitClockHigh);
        }
    }

    MasterLineDriver* _lines = nullptr;
    MasterTiming _timing;
    MasterServiceTiming _clock;
    State _state                         = State::Idle;
    State _after_stretch                 = State::Idle;
    ::m5::hal::v1::error::error_t _error = ::m5::hal::v1::error::error_t::OK;
};

class ReadByteService : public ::m5::hal::v1::service::IService {
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
               ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _lines  = &lines;
        _timing = timing;
        restart(ack_after_read, now_tick);
    }

    void restart(bool ack_after_read, ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _ack_after_read = ack_after_read;
        _byte           = 0;
        _bit_index      = 0;
        _bit_mask       = 0x80;
        _error          = ::m5::hal::v1::error::error_t::OK;
        _state          = State::ReleaseSda;
        _clock.reset(now_tick);
    }

    ::m5::hal::v1::service::ServiceResult service(const ::m5::hal::v1::service::ServiceContext& ctx) override
    {
        using ::m5::hal::v1::service::hasReached;
        if (_lines == nullptr || _state == State::Idle) {
            return ::m5::hal::v1::service::ServiceResult::Idle;
        }
        const auto state_value = static_cast<uint8_t>(_state);
        if (state_value & kTerminalStateMask) {
            return terminalStateResult(state_value);
        }
        if (!hasReached(ctx.now_tick, _clock.dueTick())) {
            return ::m5::hal::v1::service::ServiceResult::Idle;
        }

        if (_state == State::RaiseClock) {
            _lines->writeSclHigh();
            waitClockHighOrSchedule(ctx.now_tick, State::SampleBit);
            return ::m5::hal::v1::service::ServiceResult::Progress;
        }
        if (_state == State::SampleBit) {
            if (_lines->readSda()) {
                _byte |= _bit_mask;
            }
            _lines->writeSclLow();
            _bit_mask >>= 1;
            if (_bit_mask == 0) {
                _lines->writeSda(!_ack_after_read);
                scheduleAfterHalf(ctx.now_tick, State::RaiseAckClock);
            } else {
                ++_bit_index;
                scheduleAfterHalf(ctx.now_tick, State::RaiseClock);
            }
            return ::m5::hal::v1::service::ServiceResult::Progress;
        }

        switch (_state) {
            case State::ReleaseSda:
                _lines->writeSda(true);
                scheduleAfterHalfFromNow(ctx.now_tick, State::RaiseClock);
                break;
            case State::RaiseClock:
                break;
            case State::WaitClockHigh:
                switch (_clock.waitClockHigh(*_lines, _timing, ctx.now_tick)) {
                    case MasterServiceTiming::ClockWaitResult::Released:
                        scheduleAfterHalfFromNow(ctx.now_tick, _after_stretch);
                        break;
                    case MasterServiceTiming::ClockWaitResult::Timeout:
                        _error = ::m5::hal::v1::error::error_t::TIMEOUT_ERROR;
                        _state = State::Timeout;
                        break;
                    case MasterServiceTiming::ClockWaitResult::Waiting:
                        return ::m5::hal::v1::service::ServiceResult::Idle;
                }
                break;
            case State::SampleBit:
                break;
            case State::RaiseAckClock:
                _lines->writeSclHigh();
                waitClockHighOrSchedule(ctx.now_tick, State::LowerAckClock);
                break;
            case State::LowerAckClock:
                _lines->writeSclLow();
                _state = State::Done;
                break;
            case State::Idle:
            case State::Done:
            case State::Timeout:
                break;
        }

        if (_state == State::Done) {
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        if (_state == State::Timeout) {
            return ::m5::hal::v1::service::ServiceResult::Error;
        }
        return ::m5::hal::v1::service::ServiceResult::Progress;
    }

    State state() const
    {
        return _state;
    }
    bool done() const
    {
        return _state == State::Done;
    }
    uint8_t byte() const
    {
        return _byte;
    }
    ::m5::hal::v1::error::error_t error() const
    {
        return _error;
    }
    uint8_t bitIndex() const
    {
        return _bit_index;
    }
    ::m5::hal::v1::service::fast_tick_t dueTick() const
    {
        return _clock.dueTick();
    }

private:
    static constexpr uint8_t kTerminalStateMask = 0x80;
    static constexpr uint8_t kErrorStateMask    = 0x40;

    static constexpr ::m5::hal::v1::service::ServiceResult terminalStateResult(uint8_t state_value)
    {
        return (state_value & kErrorStateMask)
                   ? ::m5::hal::v1::service::ServiceResult::Error
                   : ((state_value == static_cast<uint8_t>(State::Done)) ? ::m5::hal::v1::service::ServiceResult::Done
                                                                         : ::m5::hal::v1::service::ServiceResult::Idle);
    }

    void scheduleAfterHalf(::m5::hal::v1::service::fast_tick_t now_tick, State next)
    {
        _clock.scheduleNextHalf(_timing, now_tick, _state, next);
    }
    void scheduleAfterHalfFromNow(::m5::hal::v1::service::fast_tick_t now_tick, State next)
    {
        _clock.scheduleAfterHalfFromNow(_timing, now_tick, _state, next);
    }
    void waitClockHighOrSchedule(::m5::hal::v1::service::fast_tick_t now_tick, State next)
    {
        if (_lines->readScl()) {
            scheduleAfterHalf(now_tick, next);
        } else {
            _after_stretch = next;
            _clock.beginClockStretch(now_tick, _state, State::WaitClockHigh);
        }
    }

    MasterLineDriver* _lines = nullptr;
    MasterTiming _timing;
    MasterServiceTiming _clock;
    State _state                         = State::Idle;
    State _after_stretch                 = State::Idle;
    ::m5::hal::v1::error::error_t _error = ::m5::hal::v1::error::error_t::OK;
    uint8_t _byte                        = 0;
    uint8_t _bit_index                   = 0;
    uint8_t _bit_mask                    = 0x80;
    bool _ack_after_read                 = false;
};

class MasterTransactionService : public ::m5::hal::v1::service::IService {
public:
    enum class Operation : uint8_t { Idle, Start, WriteByte, ReadByte, Stop, Address, WriteBuffer, ReadBuffer };
    enum class Phase : uint8_t { Idle, Start, Write, Read };

    void beginStart(MasterLineDriver& lines, const MasterTiming& timing, ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _operation = Operation::Start;
        _error     = ::m5::hal::v1::error::error_t::OK;
        _start.begin(lines, timing, now_tick);
    }
    void beginWriteByte(MasterLineDriver& lines, const MasterTiming& timing, uint8_t byte,
                        ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _operation = Operation::WriteByte;
        _error     = ::m5::hal::v1::error::error_t::OK;
        _write.begin(lines, timing, byte, now_tick);
    }
    void beginReadByte(MasterLineDriver& lines, const MasterTiming& timing, bool ack_after_read,
                       ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _operation = Operation::ReadByte;
        _error     = ::m5::hal::v1::error::error_t::OK;
        _read.begin(lines, timing, ack_after_read, now_tick);
    }
    void beginStop(MasterLineDriver& lines, const MasterTiming& timing, ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _operation = Operation::Stop;
        _error     = ::m5::hal::v1::error::error_t::OK;
        _stop.begin(lines, timing, now_tick);
    }
    void beginAddress(MasterLineDriver& lines, const MasterTiming& timing, uint8_t addr_byte,
                      ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _lines     = &lines;
        _timing    = &timing;
        _operation = Operation::Address;
        _phase     = Phase::Start;
        _byte      = addr_byte;
        _error     = ::m5::hal::v1::error::error_t::OK;
        _start.begin(lines, timing, now_tick);
    }
    void beginWriteBuffer(MasterLineDriver& lines, const MasterTiming& timing, const uint8_t* data, size_t len,
                          ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _lines     = &lines;
        _timing    = &timing;
        _operation = Operation::WriteBuffer;
        _phase     = Phase::Write;
        _tx_data   = data;
        _length    = len;
        _index     = 0;
        _error     = ::m5::hal::v1::error::error_t::OK;
        _write.begin(lines, timing, data[0], now_tick);
    }
    void beginReadBuffer(MasterLineDriver& lines, const MasterTiming& timing, uint8_t* data, size_t len, bool last_nack,
                         ::m5::hal::v1::service::fast_tick_t now_tick)
    {
        _lines     = &lines;
        _timing    = &timing;
        _operation = Operation::ReadBuffer;
        _phase     = Phase::Read;
        _rx_data   = data;
        _length    = len;
        _index     = 0;
        _last_nack = last_nack;
        _error     = ::m5::hal::v1::error::error_t::OK;
        _read.begin(lines, timing, ackAfterRead(0), now_tick);
    }

    ::m5::hal::v1::service::ServiceResult service(const ::m5::hal::v1::service::ServiceContext& ctx) override
    {
        if (_operation == Operation::WriteBuffer) {
            return serviceWriteBuffer(ctx);
        }
        if (_operation == Operation::ReadBuffer) {
            return serviceReadBuffer(ctx);
        }

        switch (_operation) {
            case Operation::Start:
                return serviceActive(_start, ctx);
            case Operation::WriteByte:
                return serviceActive(_write, ctx);
            case Operation::ReadByte:
                return serviceActive(_read, ctx);
            case Operation::Stop:
                return serviceActive(_stop, ctx);
            case Operation::Address:
                return serviceAddress(ctx);
            case Operation::WriteBuffer:
                break;
            case Operation::ReadBuffer:
                break;
            case Operation::Idle:
                break;
        }
        return ::m5::hal::v1::service::ServiceResult::Idle;
    }

    Operation operation() const
    {
        return _operation;
    }
    ::m5::hal::v1::error::error_t error() const
    {
        return _error;
    }
    uint8_t byte() const
    {
        return _read.byte();
    }
    bool acked() const
    {
        return _write.acked();
    }
    size_t transferred() const
    {
        return _index;
    }
    ::m5::hal::v1::service::fast_tick_t dueTick() const
    {
        switch (_operation) {
            case Operation::Start:
                return _start.dueTick();
            case Operation::WriteByte:
                return _write.dueTick();
            case Operation::ReadByte:
                return _read.dueTick();
            case Operation::Stop:
                return _stop.dueTick();
            case Operation::Address:
                return (_phase == Phase::Start) ? _start.dueTick() : _write.dueTick();
            case Operation::WriteBuffer:
                return _write.dueTick();
            case Operation::ReadBuffer:
                return _read.dueTick();
            case Operation::Idle:
                break;
        }
        return 0;
    }

private:
    bool ackAfterRead(size_t index) const
    {
        return ((index + 1) < _length) || !_last_nack;
    }
    ::m5::hal::v1::service::ServiceResult serviceAddress(const ::m5::hal::v1::service::ServiceContext& ctx)
    {
        if (_phase == Phase::Start) {
            auto result = serviceComposite(_start, ctx);
            if (result != ::m5::hal::v1::service::ServiceResult::Done) {
                return result;
            }
            _phase = Phase::Write;
            _write.begin(*_lines, *_timing, _byte, ctx.now_tick);
            return ::m5::hal::v1::service::ServiceResult::Progress;
        }

        auto result = serviceComposite(_write, ctx);
        if (result == ::m5::hal::v1::service::ServiceResult::Done) {
            _operation = Operation::Idle;
            _phase     = Phase::Idle;
        }
        return result;
    }
    ::m5::hal::v1::service::ServiceResult serviceWriteBuffer(const ::m5::hal::v1::service::ServiceContext& ctx)
    {
        auto result = _write.service(ctx);
        if (result == ::m5::hal::v1::service::ServiceResult::Error) {
            _error     = _write.error();
            _operation = Operation::Idle;
            _phase     = Phase::Idle;
            return result;
        }
        if (result != ::m5::hal::v1::service::ServiceResult::Done) {
            return result;
        }

        ++_index;
        if (_index >= _length) {
            _operation = Operation::Idle;
            _phase     = Phase::Idle;
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        _write.restart(_tx_data[_index], ctx.now_tick);
        return ::m5::hal::v1::service::ServiceResult::Progress;
    }
    ::m5::hal::v1::service::ServiceResult serviceReadBuffer(const ::m5::hal::v1::service::ServiceContext& ctx)
    {
        auto result = _read.service(ctx);
        if (result == ::m5::hal::v1::service::ServiceResult::Error) {
            _error     = _read.error();
            _operation = Operation::Idle;
            _phase     = Phase::Idle;
            return result;
        }
        if (result != ::m5::hal::v1::service::ServiceResult::Done) {
            return result;
        }

        _rx_data[_index] = _read.byte();
        ++_index;
        if (_index >= _length) {
            _operation = Operation::Idle;
            _phase     = Phase::Idle;
            return ::m5::hal::v1::service::ServiceResult::Done;
        }
        _read.restart(ackAfterRead(_index), ctx.now_tick);
        return ::m5::hal::v1::service::ServiceResult::Progress;
    }
    template <typename Service>
    ::m5::hal::v1::service::ServiceResult serviceActive(Service& service,
                                                        const ::m5::hal::v1::service::ServiceContext& ctx)
    {
        auto result = service.service(ctx);
        if (result == ::m5::hal::v1::service::ServiceResult::Done) {
            _operation = Operation::Idle;
        } else if (result == ::m5::hal::v1::service::ServiceResult::Error) {
            _error     = service.error();
            _operation = Operation::Idle;
        }
        return result;
    }
    template <typename Service>
    ::m5::hal::v1::service::ServiceResult serviceComposite(Service& service,
                                                           const ::m5::hal::v1::service::ServiceContext& ctx)
    {
        auto result = service.service(ctx);
        if (result == ::m5::hal::v1::service::ServiceResult::Error) {
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
    ::m5::hal::v1::error::error_t _error = ::m5::hal::v1::error::error_t::OK;
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

}  // namespace m5::variants::frameworks::software::hal::v1::i2c

// The user-facing surface (Bus_software / BusConfig_software) lives in the
// kind namespace per the variant naming rule (spec/design/variants.md);
// the bit-bang machinery above stays in the variant namespace as an
// implementation detail.
namespace m5::hal::v1::i2c {

// This variant needs no fields beyond the abstract kind config; the
// empty derivation still gives `init` a variant-owned type, so a
// sibling variant's config cannot be passed by accident.
struct BusConfig_software : public IBusConfig {
    using IBusConfig::IBusConfig;
};

class Bus_software : public IBus {
public:
    // Typed init: takes this variant's BusConfig_software, so a sibling
    // variant's config is a compile error instead of a bad downcast.
    ::m5::hal::v1::result_t<void> init(const BusConfig_software& config);
    // The bit-bang variant owns no resource beyond the high-Z state
    // of the pins it grabbed in `init`, so `release` is a no-op that
    // returns OK.
    ::m5::hal::v1::result_t<void> release(void) override
    {
        return {};
    }

    ::m5::hal::v1::result_t<size_t> transfer(::m5::hal::v1::bus::IAccessor* owner,
                                             const ::m5::hal::v1::i2c::MasterAccessConfig& cfg,
                                             const ::m5::hal::v1::i2c::TransferDesc& desc,
                                             ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx) override;

private:
    // Resolved during `init()` from `_config.pin_scl` / `pin_sda`
    // via `m5::hal::v1::M5_Hal.Gpio.getPin(num)`. Cached here on the
    // Bus to skip the lookup on every transfer hot path.
    ::m5::hal::v1::gpio::Pin _pin_scl{};
    ::m5::hal::v1::gpio::Pin _pin_sda{};
};

}  // namespace m5::hal::v1::i2c

#endif
