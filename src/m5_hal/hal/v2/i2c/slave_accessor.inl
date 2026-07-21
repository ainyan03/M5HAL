// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_I2C_SLAVE_ACCESSOR_INL_
#define M5_HAL_HAL_V2_I2C_SLAVE_ACCESSOR_INL_

#include "slave_accessor.hpp"

namespace m5::hal::v2::i2c {

inline SlaveAccessor::SlaveAccessor(ISlaveBus& bus, slave::QueueStorage<> tx_storage, slave::QueueStorage<> rx_storage,
                                    I2cSegmentStorage segment_storage, const SlaveAccessConfig& config)
    : bus::IAccessor{bus}, context_{makeOperationContext(config)}
{
    auto tx_bound = tx_queue_.bind(tx_storage, config.tx_mode);
    if (!tx_bound.has_value()) {
        queue_bind_error_ = tx_bound.error();
        return;
    }
    auto rx_bound = rx_queue_.bind(rx_storage, config.rx_mode);
    if (!rx_bound.has_value()) {
        queue_bind_error_ = rx_bound.error();
        return;
    }
    auto segments_bound = segment_queue_.bind(segment_storage);
    if (!segments_bound.has_value()) {
        queue_bind_error_ = segments_bound.error();
        return;
    }
    events_.setLevelProbe(&SlaveAccessor::probeEventLevel, this);
}

inline const SlaveAccessConfig& SlaveAccessor::getConfig() const
{
    return context_.config;
}

inline result_t<void> SlaveAccessor::setConfig(const SlaveAccessConfig& config)
{
    if (inAccess() || tx_queue_.hasReservation() || rx_queue_.hasReservation() || segment_queue_.hasReservation() ||
        segment_queue_.readable() != 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto tx_mode = tx_queue_.setMode(config.tx_mode);
    if (!tx_mode.has_value()) {
        return tx_mode;
    }
    auto rx_mode = rx_queue_.setMode(config.rx_mode);
    if (!rx_mode.has_value()) {
        (void)tx_queue_.setMode(context_.config.tx_mode);
        return rx_mode;
    }
    context_.config = config;
    return {};
}

inline ISlaveBus& SlaveAccessor::getBus() const
{
    return static_cast<ISlaveBus&>(bus::IAccessor::getBus());
}

inline result_t<void> SlaveAccessor::beginAccess(uint32_t timeout_ms)
{
    if (error::isError(queue_bind_error_)) {
        return m5::stl::make_unexpected(queue_bind_error_);
    }
    if (tx_queue_.hasReservation() || rx_queue_.hasReservation() || segment_queue_.hasReservation()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return _beginOperationAccess(context_, timeout_ms, bus::OperationMode::Slave,
                                 [&](bus::OperationContext<SlaveAccessConfig>& context) -> result_t<void> {
                                     auto tx_generation = tx_queue_.setGeneration(context.runtime.generation);
                                     if (!tx_generation.has_value()) {
                                         return tx_generation;
                                     }
                                     auto rx_generation = rx_queue_.setGeneration(context.runtime.generation);
                                     if (!rx_generation.has_value()) {
                                         return rx_generation;
                                     }
                                     auto segment_generation = segment_queue_.setGeneration(context.runtime.generation);
                                     if (!segment_generation.has_value()) {
                                         return segment_generation;
                                     }
                                     auto events_started = events_.begin(context.runtime.generation);
                                     if (!events_started.has_value()) {
                                         return events_started;
                                     }
                                     auto begun = getBus().beginOperation(context);
                                     if (!begun.has_value()) {
                                         events_.end(context.runtime.generation);
                                     }
                                     return begun;
                                 });
}

inline result_t<void> SlaveAccessor::endAccess(uint32_t timeout_ms)
{
    if (!inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    const bool leaked_tx_reservation = tx_queue_.hasReservation();
    if (leaked_tx_reservation) {
        (void)tx_queue_.cancelReservation();
        context_.runtime.state |= bus::OperationStateFlags::ReservationCancelled;
    }
    bool leaked_backend_reservation = false;
    auto ended                      = _endOperationAccess(
        context_, timeout_ms, [&](bus::OperationContext<SlaveAccessConfig>& context) -> result_t<void> {
            auto backend               = getBus().endOperation(context);
            leaked_backend_reservation = rx_queue_.hasReservation() || segment_queue_.hasReservation();
            if (rx_queue_.hasReservation()) {
                (void)rx_queue_.cancelReservation();
            }
            if (segment_queue_.hasReservation()) {
                (void)segment_queue_.cancelReservation();
            }
            if (leaked_backend_reservation) {
                context.runtime.state |= bus::OperationStateFlags::ReservationCancelled;
            }
            events_.end(context.runtime.generation);
            if (!backend.has_value()) {
                return backend;
            }
            if (leaked_backend_reservation) {
                return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
            }
            return {};
        });
    if (!ended.has_value()) {
        return ended;
    }
    if (leaked_tx_reservation) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return {};
}

inline bool SlaveAccessor::inAccess() const
{
    return _inOperationAccess();
}

inline result_t<size_t> SlaveAccessor::write(data::ConstDataSpan src)
{
    auto written = tx_queue_.write(src);
    if (written.has_value() && *written != 0) {
        getBus().notifyOperationActivity(this);
    }
    return written;
}

inline result_t<size_t> SlaveAccessor::read(data::DataSpan dst)
{
    auto read = rx_queue_.read(dst);
    if (read.has_value() && *read != 0) {
        getBus().notifyOperationActivity(this);
    }
    return read;
}

inline size_t SlaveAccessor::readable() const
{
    return rx_queue_.readable();
}

inline size_t SlaveAccessor::writable() const
{
    return tx_queue_.writable();
}

inline slave::FrameSinkView SlaveAccessor::txFrames()
{
    return tx_queue_.frameSink();
}

inline I2cFrameSourceView SlaveAccessor::rxFrames()
{
    return I2cFrameSourceView{&rx_queue_, &segment_queue_};
}

inline result_t<void> SlaveAccessor::clearTx()
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    tx_queue_.reset(tx_queue_.generation());
    return {};
}

inline result_t<void> SlaveAccessor::clearRx()
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    rx_queue_.reset(rx_queue_.generation());
    segment_queue_.reset(segment_queue_.generation());
    return {};
}

inline slave::QueueStatus SlaveAccessor::txStatus() const
{
    return tx_queue_.status();
}

inline slave::QueueStatus SlaveAccessor::rxStatus() const
{
    return rx_queue_.status();
}

inline slave::QueueStatus SlaveAccessor::clearTxStatus()
{
    return tx_queue_.clearStatus();
}

inline slave::QueueStatus SlaveAccessor::clearRxStatus()
{
    return rx_queue_.clearStatus();
}

inline result_t<void> SlaveAccessor::setEventCallback(slave::SlaveEventCallback callback, void* user)
{
    return events_.setEventCallback(callback, user);
}

inline result_t<void> SlaveAccessor::dispatchEvents()
{
    return events_.dispatchEvents();
}

inline result_t<void> SlaveAccessor::acknowledgeEvents(slave::SlaveEvent events)
{
    return events_.acknowledgeEvents(events);
}

inline service::IService& SlaveAccessor::eventService()
{
    return events_.eventService();
}

inline slave::SlaveQueue& SlaveAccessor::backendTxQueue()
{
    return tx_queue_;
}

inline result_t<size_t> SlaveAccessor::backendWriteRx(data::ConstDataSpan src)
{
    return rx_queue_.write(src);
}

inline size_t SlaveAccessor::backendRxWritable() const
{
    return rx_queue_.writable();
}

inline void SlaveAccessor::backendRecordDroppedBytes(uint32_t dropped_bytes)
{
    rx_queue_.recordDroppedFrames(0, dropped_bytes);
}

inline void SlaveAccessor::backendRecordDroppedFrame(uint32_t dropped_bytes)
{
    rx_queue_.recordDroppedFrame(dropped_bytes);
}

inline I2cObservedFrameWriter SlaveAccessor::backendRxFrames()
{
    return I2cObservedFrameWriter{&rx_queue_, &segment_queue_};
}

inline slave::SlaveEventEndpoint& SlaveAccessor::backendEvents()
{
    return events_;
}

inline slave::SlaveEventInfo SlaveAccessor::probeEventLevel(void* user, uint32_t generation)
{
    auto& accessor = *static_cast<SlaveAccessor*>(user);
    slave::SlaveEventInfo info;
    info.generation   = generation;
    info.rx_available = accessor.rx_queue_.readable();
    info.tx_available = accessor.tx_queue_.writable();
    if (info.rx_available != 0) {
        info.events |= slave::SlaveEvent::RxAvailable;
    }
    if (accessor.rx_queue_.readableFrames() != 0) {
        info.events |= slave::SlaveEvent::FrameCompleted;
    }
    if (info.tx_available != 0) {
        info.events |= slave::SlaveEvent::TxSpace;
    }
    if (slave::any(accessor.rx_queue_.status().sticky_events & slave::QueueEventFlags::Overflow)) {
        info.events |= slave::SlaveEvent::Overflow;
    }
    if (slave::any(accessor.tx_queue_.status().sticky_events & slave::QueueEventFlags::Underrun)) {
        info.events |= slave::SlaveEvent::Underrun;
    }
    return info;
}

}  // namespace m5::hal::v2::i2c

#endif  // M5_HAL_HAL_V2_I2C_SLAVE_ACCESSOR_INL_
