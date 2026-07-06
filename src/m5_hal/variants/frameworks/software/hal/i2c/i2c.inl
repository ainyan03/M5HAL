// SPDX-License-Identifier: MIT
// Software (bit-bang) I2C implementation. Included by M5HAL_v2.cpp via
// the variant's hal.inl hub. Flat-injected as the default I2C when no
// platform variant offers a hardware implementation.

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_I2C_I2C_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_SOFTWARE_HAL_I2C_I2C_INL

#include "i2c.hpp"
#include <M5Utility.hpp>

#include <new>

namespace m5::variants::frameworks::software::hal::v2::i2c::detail {

::m5::hal::v2::result_t<MasterTiming> MasterTiming::fromConfig(const ::m5::hal::v2::i2c::MasterAccessConfig& cfg)
{
    if (cfg.freq == 0) {
        return m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }

    MasterTiming timing;
    const uint64_t denom = static_cast<uint64_t>(cfg.freq) * 2ull;
    uint64_t half        = (static_cast<uint64_t>(kNsecPerSec) + (denom / 2ull)) / denom;
    if (half == 0) {
        half = 1;
    }
    if (half > ::m5::hal::v2::service::kMaxComparableDelayTicks) {
        return m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    timing.half_period = static_cast<::m5::hal::v2::service::fast_tick_t>(half);
    timing.timeout     = (cfg.wire_timeout_ms > (kMaxUsec / kUsecPerMsec))
                             ? kMaxUsec
                             : static_cast<::m5::hal::v2::service::fast_tick_t>(cfg.wire_timeout_ms * kUsecPerMsec);
    return timing;
}

void MasterServiceTiming::reset(::m5::hal::v2::service::fast_tick_t now_tick)
{
    _due           = now_tick;
    _stretch_start = now_tick;
}

::m5::hal::v2::service::fast_tick_t MasterServiceTiming::dueTick() const
{
    return _due;
}

MasterServiceTiming::ClockWaitResult MasterServiceTiming::waitClockHigh(
    const MasterLineDriver& lines, const MasterTiming& timing, ::m5::hal::v2::service::fast_tick_t now_tick) const
{
    if (lines.readScl()) {
        return ClockWaitResult::Released;
    }
    if (::m5::hal::v2::service::elapsedTicks(now_tick, _stretch_start) >= timing.timeout) {
        return ClockWaitResult::Timeout;
    }
    return ClockWaitResult::Waiting;
}

void StartConditionService::begin(MasterLineDriver& lines, const MasterTiming& timing,
                                  ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _lines  = &lines;
    _timing = timing;
    _error  = ::m5::hal::v2::error::error_t::OK;
    _state  = State::ReleaseScl;
    _clock.reset(now_tick);
}

::m5::hal::v2::service::ServiceResult StartConditionService::service(const ::m5::hal::v2::service::ServiceContext& ctx)
{
    using ::m5::hal::v2::service::hasReached;
    if (_lines == nullptr || _state == State::Idle) {
        return ::m5::hal::v2::service::ServiceResult::Idle;
    }
    if (_state == State::Done) {
        return ::m5::hal::v2::service::ServiceResult::Done;
    }
    if (_state == State::Timeout) {
        return ::m5::hal::v2::service::ServiceResult::Error;
    }
    if (!hasReached(ctx.now_tick, _clock.dueTick())) {
        return ::m5::hal::v2::service::ServiceResult::Idle;
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
                    _error = ::m5::hal::v2::error::error_t::TIMEOUT_ERROR;
                    _state = State::Timeout;
                    break;
                case MasterServiceTiming::ClockWaitResult::Waiting:
                    return ::m5::hal::v2::service::ServiceResult::Idle;
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
        return ::m5::hal::v2::service::ServiceResult::Done;
    }
    if (_state == State::Timeout) {
        return ::m5::hal::v2::service::ServiceResult::Error;
    }
    return ::m5::hal::v2::service::ServiceResult::Progress;
}

StartConditionService::State StartConditionService::state() const
{
    return _state;
}

bool StartConditionService::done() const
{
    return _state == State::Done;
}

::m5::hal::v2::service::fast_tick_t StartConditionService::dueTick() const
{
    return _clock.dueTick();
}

::m5::hal::v2::error::error_t StartConditionService::error() const
{
    return _error;
}

void WriteByteService::begin(MasterLineDriver& lines, const MasterTiming& timing, uint8_t byte,
                             ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _lines  = &lines;
    _timing = timing;
    restart(byte, now_tick);
}

void WriteByteService::restart(uint8_t byte, ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _byte      = byte;
    _bit_index = 0;
    _bit_mask  = 0x80;
    _acked     = false;
    _error     = ::m5::hal::v2::error::error_t::OK;
    _state     = State::RaiseClock;
    _clock.reset(now_tick);
    _lines->writeSda(bitValue());
    _clock.scheduleAfterHalfFromNow(_timing, now_tick, _state, State::RaiseClock);
}

::m5::hal::v2::service::ServiceResult WriteByteService::service(const ::m5::hal::v2::service::ServiceContext& ctx)
{
    using ::m5::hal::v2::service::hasReached;
    if (_lines == nullptr) {
        return ::m5::hal::v2::service::ServiceResult::Idle;
    }

    const auto state_value = static_cast<uint8_t>(_state);
    if (state_value & kTerminalStateMask) {
        return terminalStateResult(state_value);
    }
    if (!hasReached(ctx.now_tick, _clock.dueTick())) {
        return ::m5::hal::v2::service::ServiceResult::Idle;
    }

    if (_state == State::RaiseClock) {
        _lines->writeSclHigh();
        waitClockHighOrSchedule(ctx.now_tick, State::LowerClock);
        return ::m5::hal::v2::service::ServiceResult::Progress;
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
        return ::m5::hal::v2::service::ServiceResult::Progress;
    }

    switch (_state) {
        case State::RaiseClock:
        case State::LowerClock:
            break;
        case State::RaiseAckClock:
            _lines->writeSclHigh();
            waitClockHighOrSchedule(ctx.now_tick, State::SampleAck);
            return ::m5::hal::v2::service::ServiceResult::Progress;
        case State::WaitClockHigh:
            switch (_clock.waitClockHigh(*_lines, _timing, ctx.now_tick)) {
                case MasterServiceTiming::ClockWaitResult::Released:
                    scheduleAfterHalfFromNow(ctx.now_tick, _after_stretch);
                    return ::m5::hal::v2::service::ServiceResult::Progress;
                case MasterServiceTiming::ClockWaitResult::Timeout:
                    _error = ::m5::hal::v2::error::error_t::TIMEOUT_ERROR;
                    _state = State::Timeout;
                    return ::m5::hal::v2::service::ServiceResult::Error;
                case MasterServiceTiming::ClockWaitResult::Waiting:
                    return ::m5::hal::v2::service::ServiceResult::Idle;
            }
            return ::m5::hal::v2::service::ServiceResult::Idle;
        case State::SampleAck:
            _acked = !_lines->readSda();
            _lines->writeSclLow();
            if (_acked) {
                _state = State::Done;
                return ::m5::hal::v2::service::ServiceResult::Done;
            } else {
                _error = ::m5::hal::v2::error::error_t::I2C_NO_ACK;
                _state = State::Nack;
                return ::m5::hal::v2::service::ServiceResult::Error;
            }
        case State::Idle:
        case State::Done:
        case State::Nack:
        case State::Timeout:
            break;
    }
    return ::m5::hal::v2::service::ServiceResult::Idle;
}

WriteByteService::State WriteByteService::state() const
{
    return _state;
}

bool WriteByteService::done() const
{
    return _state == State::Done;
}

bool WriteByteService::acked() const
{
    return _acked;
}

::m5::hal::v2::error::error_t WriteByteService::error() const
{
    return _error;
}

uint8_t WriteByteService::bitIndex() const
{
    return _bit_index;
}

::m5::hal::v2::service::fast_tick_t WriteByteService::dueTick() const
{
    return _clock.dueTick();
}

constexpr ::m5::hal::v2::service::ServiceResult WriteByteService::terminalStateResult(uint8_t state_value)
{
    return (state_value & kErrorStateMask)
               ? ::m5::hal::v2::service::ServiceResult::Error
               : ((state_value == static_cast<uint8_t>(State::Done)) ? ::m5::hal::v2::service::ServiceResult::Done
                                                                     : ::m5::hal::v2::service::ServiceResult::Idle);
}

bool WriteByteService::bitValue() const
{
    return (_byte & _bit_mask) != 0;
}

void WriteByteService::scheduleAfterHalf(::m5::hal::v2::service::fast_tick_t now_tick, State next)
{
    _clock.scheduleNextHalf(_timing, now_tick, _state, next);
}

void WriteByteService::scheduleAfterHalfFromNow(::m5::hal::v2::service::fast_tick_t now_tick, State next)
{
    _clock.scheduleAfterHalfFromNow(_timing, now_tick, _state, next);
}

void WriteByteService::waitClockHighOrSchedule(::m5::hal::v2::service::fast_tick_t now_tick, State next)
{
    if (_lines->readScl()) {
        scheduleAfterHalf(now_tick, next);
    } else {
        _after_stretch = next;
        _clock.beginClockStretch(now_tick, _state, State::WaitClockHigh);
    }
}

void StopConditionService::begin(MasterLineDriver& lines, const MasterTiming& timing,
                                 ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _lines  = &lines;
    _timing = timing;
    _error  = ::m5::hal::v2::error::error_t::OK;
    _state  = State::PullSdaLow;
    _clock.reset(now_tick);
}

::m5::hal::v2::service::ServiceResult StopConditionService::service(const ::m5::hal::v2::service::ServiceContext& ctx)
{
    using ::m5::hal::v2::service::hasReached;
    if (_lines == nullptr || _state == State::Idle) {
        return ::m5::hal::v2::service::ServiceResult::Idle;
    }
    if (_state == State::Done) {
        return ::m5::hal::v2::service::ServiceResult::Done;
    }
    if (_state == State::BusError || _state == State::Timeout) {
        return ::m5::hal::v2::service::ServiceResult::Error;
    }
    if (!hasReached(ctx.now_tick, _clock.dueTick())) {
        return ::m5::hal::v2::service::ServiceResult::Idle;
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
                    _error = ::m5::hal::v2::error::error_t::TIMEOUT_ERROR;
                    _state = State::Timeout;
                    break;
                case MasterServiceTiming::ClockWaitResult::Waiting:
                    return ::m5::hal::v2::service::ServiceResult::Idle;
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
                _error = ::m5::hal::v2::error::error_t::I2C_BUS_ERROR;
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
        return ::m5::hal::v2::service::ServiceResult::Done;
    }
    if (_state == State::BusError || _state == State::Timeout) {
        return ::m5::hal::v2::service::ServiceResult::Error;
    }
    return ::m5::hal::v2::service::ServiceResult::Progress;
}

StopConditionService::State StopConditionService::state() const
{
    return _state;
}

bool StopConditionService::done() const
{
    return _state == State::Done;
}

::m5::hal::v2::error::error_t StopConditionService::error() const
{
    return _error;
}

::m5::hal::v2::service::fast_tick_t StopConditionService::dueTick() const
{
    return _clock.dueTick();
}

void StopConditionService::scheduleAfterHalf(::m5::hal::v2::service::fast_tick_t now_tick, State next)
{
    _clock.scheduleAfterHalfFromNow(_timing, now_tick, _state, next);
}

void StopConditionService::waitClockHighOrSchedule(::m5::hal::v2::service::fast_tick_t now_tick, State next)
{
    if (_lines->readScl()) {
        scheduleAfterHalf(now_tick, next);
    } else {
        _after_stretch = next;
        _clock.beginClockStretch(now_tick, _state, State::WaitClockHigh);
    }
}

void ReadByteService::begin(MasterLineDriver& lines, const MasterTiming& timing, bool ack_after_read,
                            ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _lines  = &lines;
    _timing = timing;
    restart(ack_after_read, now_tick);
}

void ReadByteService::restart(bool ack_after_read, ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _ack_after_read = ack_after_read;
    _byte           = 0;
    _bit_index      = 0;
    _bit_mask       = 0x80;
    _error          = ::m5::hal::v2::error::error_t::OK;
    _state          = State::ReleaseSda;
    _clock.reset(now_tick);
}

::m5::hal::v2::service::ServiceResult ReadByteService::service(const ::m5::hal::v2::service::ServiceContext& ctx)
{
    using ::m5::hal::v2::service::hasReached;
    if (_lines == nullptr || _state == State::Idle) {
        return ::m5::hal::v2::service::ServiceResult::Idle;
    }
    const auto state_value = static_cast<uint8_t>(_state);
    if (state_value & kTerminalStateMask) {
        return terminalStateResult(state_value);
    }
    if (!hasReached(ctx.now_tick, _clock.dueTick())) {
        return ::m5::hal::v2::service::ServiceResult::Idle;
    }

    if (_state == State::RaiseClock) {
        _lines->writeSclHigh();
        waitClockHighOrSchedule(ctx.now_tick, State::SampleBit);
        return ::m5::hal::v2::service::ServiceResult::Progress;
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
        return ::m5::hal::v2::service::ServiceResult::Progress;
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
                    _error = ::m5::hal::v2::error::error_t::TIMEOUT_ERROR;
                    _state = State::Timeout;
                    break;
                case MasterServiceTiming::ClockWaitResult::Waiting:
                    return ::m5::hal::v2::service::ServiceResult::Idle;
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
        return ::m5::hal::v2::service::ServiceResult::Done;
    }
    if (_state == State::Timeout) {
        return ::m5::hal::v2::service::ServiceResult::Error;
    }
    return ::m5::hal::v2::service::ServiceResult::Progress;
}

ReadByteService::State ReadByteService::state() const
{
    return _state;
}

bool ReadByteService::done() const
{
    return _state == State::Done;
}

uint8_t ReadByteService::byte() const
{
    return _byte;
}

::m5::hal::v2::error::error_t ReadByteService::error() const
{
    return _error;
}

uint8_t ReadByteService::bitIndex() const
{
    return _bit_index;
}

::m5::hal::v2::service::fast_tick_t ReadByteService::dueTick() const
{
    return _clock.dueTick();
}

constexpr ::m5::hal::v2::service::ServiceResult ReadByteService::terminalStateResult(uint8_t state_value)
{
    return (state_value & kErrorStateMask)
               ? ::m5::hal::v2::service::ServiceResult::Error
               : ((state_value == static_cast<uint8_t>(State::Done)) ? ::m5::hal::v2::service::ServiceResult::Done
                                                                     : ::m5::hal::v2::service::ServiceResult::Idle);
}

void ReadByteService::scheduleAfterHalf(::m5::hal::v2::service::fast_tick_t now_tick, State next)
{
    _clock.scheduleNextHalf(_timing, now_tick, _state, next);
}

void ReadByteService::scheduleAfterHalfFromNow(::m5::hal::v2::service::fast_tick_t now_tick, State next)
{
    _clock.scheduleAfterHalfFromNow(_timing, now_tick, _state, next);
}

void ReadByteService::waitClockHighOrSchedule(::m5::hal::v2::service::fast_tick_t now_tick, State next)
{
    if (_lines->readScl()) {
        scheduleAfterHalf(now_tick, next);
    } else {
        _after_stretch = next;
        _clock.beginClockStretch(now_tick, _state, State::WaitClockHigh);
    }
}

void MasterTransactionService::beginStart(MasterLineDriver& lines, const MasterTiming& timing,
                                          ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _operation = Operation::Start;
    _error     = ::m5::hal::v2::error::error_t::OK;
    _start.begin(lines, timing, now_tick);
}

void MasterTransactionService::beginWriteByte(MasterLineDriver& lines, const MasterTiming& timing, uint8_t byte,
                                              ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _operation = Operation::WriteByte;
    _error     = ::m5::hal::v2::error::error_t::OK;
    _write.begin(lines, timing, byte, now_tick);
}

void MasterTransactionService::beginReadByte(MasterLineDriver& lines, const MasterTiming& timing, bool ack_after_read,
                                             ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _operation = Operation::ReadByte;
    _error     = ::m5::hal::v2::error::error_t::OK;
    _read.begin(lines, timing, ack_after_read, now_tick);
}

void MasterTransactionService::beginStop(MasterLineDriver& lines, const MasterTiming& timing,
                                         ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _operation = Operation::Stop;
    _error     = ::m5::hal::v2::error::error_t::OK;
    _stop.begin(lines, timing, now_tick);
}

void MasterTransactionService::beginAddress(MasterLineDriver& lines, const MasterTiming& timing, uint8_t addr_byte,
                                            ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _lines     = &lines;
    _timing    = &timing;
    _operation = Operation::Address;
    _phase     = Phase::Start;
    _byte      = addr_byte;
    _error     = ::m5::hal::v2::error::error_t::OK;
    _start.begin(lines, timing, now_tick);
}

void MasterTransactionService::beginWriteBuffer(MasterLineDriver& lines, const MasterTiming& timing,
                                                const uint8_t* data, size_t len,
                                                ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _lines     = &lines;
    _timing    = &timing;
    _operation = Operation::WriteBuffer;
    _phase     = Phase::Write;
    _tx_data   = data;
    _length    = len;
    _index     = 0;
    _error     = ::m5::hal::v2::error::error_t::OK;
    _write.begin(lines, timing, data[0], now_tick);
}

void MasterTransactionService::beginReadBuffer(MasterLineDriver& lines, const MasterTiming& timing, uint8_t* data,
                                               size_t len, bool last_nack, ::m5::hal::v2::service::fast_tick_t now_tick)
{
    _lines     = &lines;
    _timing    = &timing;
    _operation = Operation::ReadBuffer;
    _phase     = Phase::Read;
    _rx_data   = data;
    _length    = len;
    _index     = 0;
    _last_nack = last_nack;
    _error     = ::m5::hal::v2::error::error_t::OK;
    _read.begin(lines, timing, ackAfterRead(0), now_tick);
}

::m5::hal::v2::service::ServiceResult MasterTransactionService::service(
    const ::m5::hal::v2::service::ServiceContext& ctx)
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
    return ::m5::hal::v2::service::ServiceResult::Idle;
}

MasterTransactionService::Operation MasterTransactionService::operation() const
{
    return _operation;
}

::m5::hal::v2::error::error_t MasterTransactionService::error() const
{
    return _error;
}

uint8_t MasterTransactionService::byte() const
{
    return _read.byte();
}

bool MasterTransactionService::acked() const
{
    return _write.acked();
}

size_t MasterTransactionService::transferred() const
{
    return _index;
}

::m5::hal::v2::service::fast_tick_t MasterTransactionService::dueTick() const
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

bool MasterTransactionService::ackAfterRead(size_t index) const
{
    return ((index + 1) < _length) || !_last_nack;
}

::m5::hal::v2::service::ServiceResult MasterTransactionService::serviceAddress(
    const ::m5::hal::v2::service::ServiceContext& ctx)
{
    if (_phase == Phase::Start) {
        auto result = serviceComposite(_start, ctx);
        if (result != ::m5::hal::v2::service::ServiceResult::Done) {
            return result;
        }
        _phase = Phase::Write;
        _write.begin(*_lines, *_timing, _byte, ctx.now_tick);
        return ::m5::hal::v2::service::ServiceResult::Progress;
    }

    auto result = serviceComposite(_write, ctx);
    if (result == ::m5::hal::v2::service::ServiceResult::Done) {
        _operation = Operation::Idle;
        _phase     = Phase::Idle;
    }
    return result;
}

::m5::hal::v2::service::ServiceResult MasterTransactionService::serviceWriteBuffer(
    const ::m5::hal::v2::service::ServiceContext& ctx)
{
    auto result = _write.service(ctx);
    if (result == ::m5::hal::v2::service::ServiceResult::Error) {
        _error     = _write.error();
        _operation = Operation::Idle;
        _phase     = Phase::Idle;
        return result;
    }
    if (result != ::m5::hal::v2::service::ServiceResult::Done) {
        return result;
    }

    ++_index;
    if (_index >= _length) {
        _operation = Operation::Idle;
        _phase     = Phase::Idle;
        return ::m5::hal::v2::service::ServiceResult::Done;
    }
    _write.restart(_tx_data[_index], ctx.now_tick);
    return ::m5::hal::v2::service::ServiceResult::Progress;
}

::m5::hal::v2::service::ServiceResult MasterTransactionService::serviceReadBuffer(
    const ::m5::hal::v2::service::ServiceContext& ctx)
{
    auto result = _read.service(ctx);
    if (result == ::m5::hal::v2::service::ServiceResult::Error) {
        _error     = _read.error();
        _operation = Operation::Idle;
        _phase     = Phase::Idle;
        return result;
    }
    if (result != ::m5::hal::v2::service::ServiceResult::Done) {
        return result;
    }

    _rx_data[_index] = _read.byte();
    ++_index;
    if (_index >= _length) {
        _operation = Operation::Idle;
        _phase     = Phase::Idle;
        return ::m5::hal::v2::service::ServiceResult::Done;
    }
    _read.restart(ackAfterRead(_index), ctx.now_tick);
    return ::m5::hal::v2::service::ServiceResult::Progress;
}

}  // namespace m5::variants::frameworks::software::hal::v2::i2c::detail

namespace m5::hal::v2::i2c {

namespace {
namespace impl_software {

// The bit-bang machinery stays in the variant namespace
// (software/hal/i2c/i2c.hpp); this TU-local alias keeps the `detail::`
// spelling working for Bus_software's implementation below.
namespace detail = ::m5::variants::frameworks::software::hal::v2::i2c::detail;

class PinMasterLineDriver : public detail::MasterLineDriver {
public:
    PinMasterLineDriver(gpio::Pin& scl, gpio::Pin& sda) : _scl(scl), _sda(sda)
    {
    }

    void writeSclHigh() override
    {
        _scl.writeHigh();
    }
    void writeSclLow() override
    {
        _scl.writeLow();
    }
    void writeSda(bool high) override
    {
        _sda.write(high);
    }
    bool readScl() const override
    {
        return _scl.read();
    }
    bool readSda() const override
    {
        return _sda.read();
    }

private:
    gpio::Pin& _scl;
    gpio::Pin& _sda;
};

uint32_t serviceFastTickFrequencyHz()
{
    const static uint32_t frequency_hz = service::fastTickFrequencyHz();
    return frequency_hz;
}

service::fast_tick_t serviceNsecToTick(service::tick_nsec_t nsec)
{
    return service::nsecToFastTickCeil(nsec, serviceFastTickFrequencyHz());
}

detail::MasterTiming serviceTimingToTicks(const detail::MasterTiming& timing)
{
    // Keep detail services generic, but run the synchronous software I2C path
    // in fastTick units so the hot loop does not pay fastTick->nsec conversion.
    const uint32_t frequency_hz = serviceFastTickFrequencyHz();
    const uint32_t ticks_per_us = (frequency_hz >= 1000000u) ? (frequency_hz / 1000000u) : 1u;
    detail::MasterTiming result;
    result.half_period = serviceNsecToTick(timing.half_period);
    result.timeout     = (timing.timeout > (service::kMaxComparableDelayTicks / ticks_per_us))
                             ? service::kMaxComparableDelayTicks
                             : timing.timeout * ticks_per_us;
    return result;
}

service::fast_tick_t serviceNowTick()
{
    return service::fastTick();
}

class TransferState {
public:
    enum class Phase : uint8_t {
        Idle,
        AddressWrite,
        Prefix,
        Tx,
        StopBeforeRead,
        AddressRead,
        RxReserve,
        Rx,
        Stop,
        Done,
        Error,
    };

    TransferState(gpio::Pin& scl, gpio::Pin& sda, const i2c::MasterAccessConfig& cfg, const i2c::TransferDesc& desc,
                  data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len, const detail::MasterTiming& timing,
                  service::fast_tick_t deadline_tick)
        : _lines{scl, sda},
          _cfg{cfg},
          _header{desc.prefix, desc.prefix_len},
          _src{src},
          _dst{dst},
          _tx_remaining{tx_len},
          _rx_remaining{rx_len},
          _timing{timing},
          _deadline_tick{deadline_tick}
    {
    }

    service::ServiceResult service(const service::ServiceContext& ctx)
    {
        if (_phase == Phase::Idle) {
            startInitial(ctx.now_tick);
        }
        for (;;) {
            if (service::hasReached(ctx.now_tick, _deadline_tick)) {
                fail(error::error_t::TIMEOUT_ERROR);
                return service::ServiceResult::Error;
            }
            switch (_phase) {
                case Phase::AddressWrite:
                case Phase::Prefix:
                case Phase::Tx:
                case Phase::StopBeforeRead:
                case Phase::AddressRead:
                case Phase::Rx:
                case Phase::Stop: {
                    auto r = _transaction.service(ctx);
                    if (r == service::ServiceResult::Error) {
                        if (_phase == Phase::Stop) {
                            fail(_stop_after_error ? _error : _transaction.error());
                            return service::ServiceResult::Error;
                        }
                        return beginStopAfterError(_transaction.error(), ctx.now_tick);
                    }
                    if (r != service::ServiceResult::Done) {
                        return r;
                    }
                    auto advanced = advanceAfterDone(ctx.now_tick);
                    if (advanced != service::ServiceResult::Progress) {
                        return advanced;
                    }
                    continue;
                }
                case Phase::RxReserve:
                    return beginReadChunk(ctx.now_tick);
                case Phase::Done:
                    return service::ServiceResult::Done;
                case Phase::Error:
                    return service::ServiceResult::Error;
                case Phase::Idle:
                    break;
            }
        }
    }

    error::error_t error() const
    {
        return _error;
    }
    const bus::TransferTotals& totals() const
    {
        return _totals;
    }
    bool done() const
    {
        return _phase == Phase::Done;
    }

private:
    bool haveTx() const
    {
        return _src != nullptr && _tx_remaining > 0 && !_src->eof();
    }
    bool haveRx() const
    {
        return _dst != nullptr && _rx_remaining > 0;
    }
    void fail(error::error_t err)
    {
        _error = err;
        _phase = Phase::Error;
    }
    uint8_t addressByte(bool read_bit) const
    {
        return static_cast<uint8_t>((_cfg.i2c_addr << 1) | (read_bit ? 1 : 0));
    }
    void beginAddress(bool read_bit, service::fast_tick_t now_tick, Phase phase)
    {
        _phase = phase;
        _transaction.beginAddress(_lines, _timing, addressByte(read_bit), now_tick);
    }
    void beginStop(service::fast_tick_t now_tick)
    {
        _phase = Phase::Stop;
        _transaction.beginStop(_lines, _timing, now_tick);
    }
    void startInitial(service::fast_tick_t now_tick)
    {
        if (!_header.size && !haveTx() && !haveRx()) {
            beginAddress(false, now_tick, Phase::AddressWrite);
            return;
        }
        if (_header.size || haveTx()) {
            beginAddress(false, now_tick, Phase::AddressWrite);
            return;
        }
        beginAddress(true, now_tick, Phase::AddressRead);
    }
    service::ServiceResult advanceAfterDone(service::fast_tick_t now_tick)
    {
        switch (_phase) {
            case Phase::AddressWrite:
                if (!_header.size && !haveTx() && !haveRx()) {
                    beginStop(now_tick);
                    return service::ServiceResult::Progress;
                }
                if (_header.size) {
                    _phase = Phase::Prefix;
                    _transaction.beginWriteBuffer(_lines, _timing, _header.data, _header.size, now_tick);
                    return service::ServiceResult::Progress;
                }
                return beginNextTxOrRead(now_tick);
            case Phase::Prefix:
                return beginNextTxOrRead(now_tick);
            case Phase::Tx: {
                auto advanced = _src->advance(_active_tx_len);
                if (!advanced.has_value()) {
                    fail(advanced.error());
                    return service::ServiceResult::Error;
                }
                _tx_remaining -= _active_tx_len;
                _totals.tx += _active_tx_len;
                _active_tx_len = 0;
                return beginNextTxOrRead(now_tick);
            }
            case Phase::StopBeforeRead:
                beginAddress(true, now_tick, Phase::AddressRead);
                return service::ServiceResult::Progress;
            case Phase::AddressRead:
                _phase = Phase::RxReserve;
                return service::ServiceResult::Progress;
            case Phase::Rx: {
                auto committed = _dst->commit(_active_rx_len);
                if (!committed.has_value()) {
                    fail(committed.error());
                    return service::ServiceResult::Error;
                }
                _rx_remaining -= _active_rx_len;
                _totals.rx += _active_rx_len;
                _active_rx_len = 0;
                if (_rx_remaining > 0 && !_dst->closed()) {
                    _phase = Phase::RxReserve;
                    return service::ServiceResult::Progress;
                }
                beginStop(now_tick);
                return service::ServiceResult::Progress;
            }
            case Phase::Stop:
                if (_stop_after_error) {
                    _phase = Phase::Error;
                    return service::ServiceResult::Error;
                }
                _phase = Phase::Done;
                return service::ServiceResult::Done;
            case Phase::Idle:
            case Phase::RxReserve:
            case Phase::Done:
            case Phase::Error:
                break;
        }
        return service::ServiceResult::Idle;
    }
    service::ServiceResult beginNextTxOrRead(service::fast_tick_t now_tick)
    {
        if (haveTx()) {
            auto peeked = _src->peek(_tx_remaining);
            if (!peeked.has_value()) {
                fail(peeked.error());
                return service::ServiceResult::Error;
            }
            auto span = peeked.value().first(_tx_remaining);
            if (span.size > 0) {
                _active_tx_len = span.size;
                _phase         = Phase::Tx;
                _transaction.beginWriteBuffer(_lines, _timing, span.data, span.size, now_tick);
                return service::ServiceResult::Progress;
            }
        }
        if (haveRx()) {
            if ((_header.size || _totals.tx > 0) && !_cfg.use_restart) {
                _phase = Phase::StopBeforeRead;
                _transaction.beginStop(_lines, _timing, now_tick);
                return service::ServiceResult::Progress;
            }
            beginAddress(true, now_tick, Phase::AddressRead);
            return service::ServiceResult::Progress;
        }
        beginStop(now_tick);
        return service::ServiceResult::Progress;
    }
    service::ServiceResult beginReadChunk(service::fast_tick_t now_tick)
    {
        auto reserved = _dst->reserve(_rx_remaining);
        if (!reserved.has_value()) {
            fail(reserved.error());
            return service::ServiceResult::Error;
        }
        auto span = reserved.value().first(_rx_remaining);
        if (span.size == 0) {
            beginStop(now_tick);
            return service::ServiceResult::Progress;
        }
        _rx_remaining  = span.size;
        _active_rx_len = span.size;
        _phase         = Phase::Rx;
        _transaction.beginReadBuffer(_lines, _timing, span.data, span.size, true, now_tick);
        return service::ServiceResult::Progress;
    }
    service::ServiceResult beginStopAfterError(error::error_t err, service::fast_tick_t now_tick)
    {
        _error            = err;
        _stop_after_error = true;
        beginStop(now_tick);
        return service::ServiceResult::Progress;
    }

    PinMasterLineDriver _lines;
    i2c::MasterAccessConfig _cfg;
    data::ConstDataSpan _header{};
    data::Source* _src    = nullptr;
    data::Sink* _dst      = nullptr;
    size_t _tx_remaining  = 0;
    size_t _rx_remaining  = 0;
    size_t _active_tx_len = 0;
    size_t _active_rx_len = 0;
    detail::MasterTiming _timing;
    service::fast_tick_t _deadline_tick = 0;
    detail::MasterTransactionService _transaction;
    bus::TransferTotals _totals{};
    error::error_t _error  = error::error_t::OK;
    Phase _phase           = Phase::Idle;
    bool _stop_after_error = false;
};

}  // namespace impl_software
}  // anonymous namespace

result_t<void> Bus_software::init(const BusConfig_software& config)
{
    _config = config;

    // BusConfig_software uses the single gpio_number_t path. Resolve through
    // `M5HALCore::Gpio` (the singleton GPIOGroup) with the CHECKED
    // `tryGetPin` — the pin numbers are caller input, so a bad value
    // must come back through the expected path, not the assert/UB
    // fast path of `getPin`. Cache the resulting `Pin` into
    // `_pin_scl` / `_pin_sda` so the transfer hot path skips the lookup.
    if (_config.pin_scl < 0 || _config.pin_sda < 0) {
        M5_LIB_LOGE("software::i2c::Bus_software::init: pins not set");
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto scl_pin = M5_Hal.Gpio.tryGetPin(_config.pin_scl);
    auto sda_pin = M5_Hal.Gpio.tryGetPin(_config.pin_sda);
    if (!scl_pin.has_value() || !sda_pin.has_value()) {
        M5_LIB_LOGE("software::i2c::Bus_software::init: pin resolution failed");
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _pin_scl = scl_pin.value();
    _pin_sda = sda_pin.value();
    _pin_scl.setMode(types::gpio_mode_t::OutputOpenDrainPullup);
    _pin_scl.writeLow();
    _pin_sda.setMode(types::gpio_mode_t::OutputOpenDrainPullup);
    _pin_sda.writeLow();
    _pin_scl.writeHigh();
    m5::utility::delayMicroseconds(5);
    _pin_sda.writeHigh();
    return {};
}

service::ServicePoll Bus_software::serviceImpl(const service::ServiceContext& ctx)
{
    return serviceTransfer(ctx);
}

void Bus_software::unregisterTransferService(void)
{
    if (_transfer_registered) {
        (void)M5_Hal.Services.remove(*this);
        _transfer_registered = false;
    }
}

void Bus_software::clearTransferState(void)
{
    unregisterTransferService();
    delete static_cast<impl_software::TransferState*>(_transfer_state);
    _transfer_state  = nullptr;
    _transfer_owner  = nullptr;
    _transfer_active = false;
    _transfer_done   = true;
    _transfer_error  = error::error_t::OK;
}

service::ServiceResult Bus_software::serviceTransfer(const service::ServiceContext& ctx)
{
    auto* state = static_cast<impl_software::TransferState*>(_transfer_state);
    if (!_transfer_active || _transfer_done) {
        unregisterTransferService();
        return service::ServiceResult::Done;
    }
    if (error::isError(_transfer_error)) {
        unregisterTransferService();
        return service::ServiceResult::Error;
    }

    auto result = state->service(ctx);
    if (result == service::ServiceResult::Error) {
        _transfer_error = state->error();
        unregisterTransferService();
    } else if (result == service::ServiceResult::Done || state->done()) {
        _transfer_totals = state->totals();
        _transfer_done   = true;
        unregisterTransferService();
        return service::ServiceResult::Done;
    }
    return result;
}

result_t<void> Bus_software::transfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg,
                                      const i2c::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                      size_t rx_len)
{
    auto waited = waitTransfer(owner, cfg);
    if (!waited.has_value()) {
        return m5::stl::make_unexpected(waited.error());
    }
    auto* scl = &_pin_scl;
    auto* sda = &_pin_sda;
    if (!scl->isValid() || !sda->isValid()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    // This bit-bang master drives 7-bit addressing only: reject a 10-bit
    // request instead of silently truncating the address (which would
    // address the wrong device). Same degradation as the espidf gen4
    // backend.
    if (cfg.address_is_10bit || cfg.i2c_addr > 0x007Fu) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto timing = impl_software::detail::MasterTiming::fromConfig(cfg);
    if (!timing.has_value()) {
        return m5::stl::make_unexpected(timing.error());
    }
    const auto service_timing = impl_software::serviceTimingToTicks(*timing);
    // Whole-transfer deadline: cfg.wire_timeout_ms bounds this entire transfer
    // (espidf-equivalent per-transfer semantics). The per-stretch
    // timeout inside MasterServiceTiming still applies on top.
    const auto deadline_tick = impl_software::serviceNowTick() + service_timing.timeout;

    _transfer_state = new (std::nothrow)
        impl_software::TransferState{*scl, *sda, cfg, desc, src, tx_len, dst, rx_len, service_timing, deadline_tick};
    if (_transfer_state == nullptr) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    _transfer_owner  = owner;
    _transfer_active = true;
    _transfer_done   = false;
    _transfer_error  = error::error_t::OK;
    _transfer_totals.clear();

    auto first = serviceTransfer(service::ServiceContext{impl_software::serviceNowTick()});
    if (first == service::ServiceResult::Error) {
        const auto err = _transfer_error;
        clearTransferState();
        return m5::stl::make_unexpected(err);
    }
    if (owner == nullptr) {
        auto done = waitTransfer(owner, cfg);
        if (!done.has_value()) {
            return m5::stl::make_unexpected(done.error());
        }
        return {};
    }
    if (!_transfer_done) {
        if (M5_Hal.Services.add(*this)) {
            _transfer_registered = true;
        } else {
            clearTransferState();
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
    }
    return {};
}

result_t<bus::TransferTotals> Bus_software::waitTransfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg)
{
    (void)cfg;
    if (_transfer_active && _transfer_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    while (_transfer_active && !_transfer_done && _transfer_error == error::error_t::OK) {
        if (_transfer_registered && M5_Hal.Services.autoRunActive()) {
            runtime::yield();
        } else {
            auto result = serviceTransfer(service::ServiceContext{impl_software::serviceNowTick()});
            if (result == service::ServiceResult::Error) {
                break;
            }
        }
    }
    if (error::isError(_transfer_error)) {
        const auto err = _transfer_error;
        clearTransferState();
        return m5::stl::make_unexpected(err);
    }
    const auto totals = _transfer_totals;
    _transfer_totals.clear();
    clearTransferState();
    return totals;
}

bool Bus_software::transferBusy(bus::IAccessor* owner)
{
    return _transfer_active && _transfer_owner == owner && !_transfer_done && _transfer_error == error::error_t::OK;
}

}  // namespace m5::hal::v2::i2c

#endif
