// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_SPI_SLAVE_HPP_
#define M5_HAL_HAL_V2_SPI_SLAVE_HPP_

#include "../bus/bus.hpp"
#include "../error.hpp"
#include "../service/service.hpp"
#include "../slave/event.hpp"
#include "../slave/queue.hpp"
#include "../types.hpp"

#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v2::spi {

class SpiSlaveAccessor;

struct SlaveBusConfig : public bus::IBusConfig {
    types::gpio_number_t pin_clk  = -1;
    types::gpio_number_t pin_mosi = -1;
    types::gpio_number_t pin_miso = -1;
    types::gpio_number_t pin_cs   = -1;
    uint8_t spi_mode              = 0;
    uint8_t spi_order             = 0;
    int8_t controller             = -1;
    uint8_t tx_fill_byte          = 0x00;

    constexpr SlaveBusConfig() : bus::IBusConfig{types::bus_kind_t::SPI}
    {
    }
};

struct SlaveAccessConfig : public bus::IAccessConfig {
    uint32_t transaction_bytes = 4096;
    slave::QueueMode tx_mode   = slave::QueueMode::Byte;
    slave::QueueMode rx_mode   = slave::QueueMode::Frame;

    constexpr SlaveAccessConfig() : bus::IAccessConfig{types::bus_kind_t::SPI}
    {
    }
};

struct ISlaveBus : public bus::IBus {
    const SlaveBusConfig& getConfig() const override
    {
        return _config;
    }

    virtual result_t<void> init(const SlaveBusConfig& cfg) = 0;
    result_t<void> beginOperation(bus::OperationContext<SlaveAccessConfig>& context)
    {
        auto registered = _operation_slot.registerContext(context, this, _lock_owner);
        if (!registered.has_value()) {
            return registered;
        }
        auto begun = beginOperationBackend(context);
        if (!begun.has_value()) {
            _operation_slot.invalidate(context);
        }
        return begun;
    }
    result_t<void> endOperation(bus::OperationContext<SlaveAccessConfig>& context)
    {
        if (!_operation_slot.valid(context, this, _lock_owner)) {
            if (_operation_slot.registered(context, this)) {
                if (_operation_slot.restoreRegisteredRuntime(context, this, _lock_owner)) {
                    (void)endOperationBackend(context);
                }
                _operation_slot.invalidate(context);
            }
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        auto ended = endOperationBackend(context);
        _operation_slot.invalidate(context);
        return ended;
    }

protected:
    virtual result_t<void> beginOperationBackend(bus::OperationContext<SlaveAccessConfig>& context)
    {
        (void)context;
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    virtual result_t<void> endOperationBackend(bus::OperationContext<SlaveAccessConfig>& context)
    {
        (void)context;
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    bus::IAccessor& operationOwner(bus::OperationContext<SlaveAccessConfig>& context)
    {
        return bus::OperationSlot::contextOwner(context);
    }

    SlaveBusConfig _config;
    bus::OperationSlot _operation_slot;
};

/*!
  @brief Long-lived SPI slave endpoint backed by caller-owned SPSC queues.

  `beginAccess()` starts accepting CS-delimited master transactions and
  `endAccess()` stops the peripheral before releasing the bus. Local queue I/O
  is non-blocking and remains available while inactive, allowing TX preload and
  post-stop RX drain. One application task owns both local queue endpoints;
  backend ISR/task code owns the opposite endpoints.
 */
class SpiSlaveAccessor : public bus::IAccessor {
public:
    SpiSlaveAccessor(ISlaveBus& bus, slave::QueueStorage<> tx_storage, slave::QueueStorage<> rx_storage,
                     const SlaveAccessConfig& config = {});

    const SlaveAccessConfig& getConfig() const override;
    result_t<void> setConfig(const SlaveAccessConfig& config);
    ISlaveBus& getBus() const;

    result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    result_t<void> endAccess(uint32_t timeout_ms = 1000);
    bool inAccess() const;

    result_t<size_t> write(data::ConstDataSpan src);
    result_t<size_t> read(data::DataSpan dst);
    size_t readable() const;
    size_t writable() const;

    slave::FrameSinkView txFrames();
    slave::FrameSourceView rxFrames();
    result_t<void> clearTx();
    result_t<void> clearRx();
    slave::QueueStatus txStatus() const;
    slave::QueueStatus rxStatus() const;
    slave::QueueStatus clearTxStatus();
    slave::QueueStatus clearRxStatus();

    result_t<void> setEventCallback(slave::SlaveEventCallback callback, void* user);
    result_t<void> dispatchEvents();
    result_t<void> acknowledgeEvents(slave::SlaveEvent events);
    service::IService& eventService();

    // Backend endpoint. Applications should use the byte/frame views above.
    slave::SlaveQueue& backendTxQueue();
    slave::SlaveQueue& backendRxQueue();
    slave::SlaveEventEndpoint& backendEvents();

private:
    static slave::SlaveEventInfo probeEventLevel(void* user, uint32_t generation);

    bus::OperationContext<SlaveAccessConfig> context_;
    slave::SlaveQueue tx_queue_{};
    slave::SlaveQueue rx_queue_{};
    slave::SlaveEventEndpoint events_{};
    error::error_t queue_bind_error_ = error::error_t::OK;
};

}  // namespace m5::hal::v2::spi

#include "slave.inl"

#endif  // M5_HAL_HAL_V2_SPI_SLAVE_HPP_
