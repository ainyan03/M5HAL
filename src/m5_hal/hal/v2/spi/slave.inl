// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_SPI_SLAVE_INL_
#define M5_HAL_HAL_V2_SPI_SLAVE_INL_

#include "slave.hpp"

namespace m5::hal::v2::spi {

inline SpiSlaveAccessor::SpiSlaveAccessor(ISlaveBus& bus, slave::QueueStorage<> tx_storage,
                                          slave::QueueStorage<> rx_storage, const SlaveAccessConfig& config)
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
    events_.setLevelProbe(&SpiSlaveAccessor::probeEventLevel, this);
}

inline const SlaveAccessConfig& SpiSlaveAccessor::getConfig() const
{
    return context_.config;
}

inline result_t<void> SpiSlaveAccessor::setConfig(const SlaveAccessConfig& config)
{
    if (inAccess() || tx_queue_.hasReservation()) {
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

inline ISlaveBus& SpiSlaveAccessor::getBus() const
{
    return static_cast<ISlaveBus&>(bus::IAccessor::getBus());
}

inline result_t<void> SpiSlaveAccessor::beginAccess(uint32_t timeout_ms)
{
    if (error::isError(queue_bind_error_)) {
        return m5::stl::make_unexpected(queue_bind_error_);
    }
    if (context_.config.transaction_bytes == 0 || tx_queue_.hasReservation()) {
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

inline result_t<void> SpiSlaveAccessor::endAccess(uint32_t timeout_ms)
{
    if (!inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    const bool leaked_reservation = tx_queue_.hasReservation();
    if (leaked_reservation) {
        (void)tx_queue_.cancelReservation();
        context_.runtime.state |= bus::OperationStateFlags::ReservationCancelled;
    }
    auto ended = _endOperationAccess(context_, timeout_ms,
                                     [&](bus::OperationContext<SlaveAccessConfig>& context) -> result_t<void> {
                                         auto backend = getBus().endOperation(context);
                                         events_.end(context.runtime.generation);
                                         return backend;
                                     });
    if (!ended.has_value()) {
        return ended;
    }
    if (leaked_reservation) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return {};
}

inline bool SpiSlaveAccessor::inAccess() const
{
    return _inOperationAccess();
}

inline result_t<size_t> SpiSlaveAccessor::write(data::ConstDataSpan src)
{
    return tx_queue_.write(src);
}

inline result_t<size_t> SpiSlaveAccessor::read(data::DataSpan dst)
{
    return rx_queue_.read(dst);
}

inline size_t SpiSlaveAccessor::readable() const
{
    return rx_queue_.readable();
}

inline size_t SpiSlaveAccessor::writable() const
{
    return tx_queue_.writable();
}

inline slave::FrameSinkView SpiSlaveAccessor::txFrames()
{
    return tx_queue_.frameSink();
}

inline slave::FrameSourceView SpiSlaveAccessor::rxFrames()
{
    return rx_queue_.frameSource();
}

inline result_t<void> SpiSlaveAccessor::clearTx()
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    tx_queue_.reset(tx_queue_.generation());
    return {};
}

inline result_t<void> SpiSlaveAccessor::clearRx()
{
    if (inAccess()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    rx_queue_.reset(rx_queue_.generation());
    return {};
}

inline slave::QueueStatus SpiSlaveAccessor::txStatus() const
{
    return tx_queue_.status();
}

inline slave::QueueStatus SpiSlaveAccessor::rxStatus() const
{
    return rx_queue_.status();
}

inline slave::QueueStatus SpiSlaveAccessor::clearTxStatus()
{
    return tx_queue_.clearStatus();
}

inline slave::QueueStatus SpiSlaveAccessor::clearRxStatus()
{
    return rx_queue_.clearStatus();
}

inline result_t<void> SpiSlaveAccessor::setEventCallback(slave::SlaveEventCallback callback, void* user)
{
    return events_.setEventCallback(callback, user);
}

inline result_t<void> SpiSlaveAccessor::dispatchEvents()
{
    return events_.dispatchEvents();
}

inline result_t<void> SpiSlaveAccessor::acknowledgeEvents(slave::SlaveEvent event)
{
    return events_.acknowledgeEvents(event);
}

inline service::IService& SpiSlaveAccessor::eventService()
{
    return events_.eventService();
}

inline slave::SlaveQueue& SpiSlaveAccessor::backendTxQueue()
{
    return tx_queue_;
}

inline slave::SlaveQueue& SpiSlaveAccessor::backendRxQueue()
{
    return rx_queue_;
}

inline slave::SlaveEventEndpoint& SpiSlaveAccessor::backendEvents()
{
    return events_;
}

inline slave::SlaveEventInfo SpiSlaveAccessor::probeEventLevel(void* user, uint32_t generation)
{
    auto& accessor = *static_cast<SpiSlaveAccessor*>(user);
    slave::SlaveEventInfo info;
    info.generation   = generation;
    info.rx_available = accessor.rx_queue_.readable();
    info.tx_available = accessor.tx_queue_.writable();
    if (info.rx_available != 0) {
        info.events |= slave::SlaveEvent::RxAvailable;
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

}  // namespace m5::hal::v2::spi

#endif  // M5_HAL_HAL_V2_SPI_SLAVE_INL_
