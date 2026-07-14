// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_HANDLER_HPP_
#define M5_HAL_REMOTE_SERVER_HANDLER_HPP_

#include "../gpio/group.hpp"
#include "server_adapter.hpp"
#include "server_bus_pool.hpp"

namespace m5::hal::v2::remote {

struct RemoteServerHandler {
    static constexpr size_t kMaxGpioSubscriptions     = 64;
    static constexpr size_t kMaxGpioSubscriptionPorts = 32;

    Server* server                      = nullptr;
    ServerBusPool* pool                 = nullptr;
    uint8_t hello_flags                 = 0x00;
    gpio::IGPIO* gpio                   = nullptr;
    gpio::GPIOGroup* gpio_group         = nullptr;
    spi::MasterAccessor* static_spi_acc = nullptr;
    uint8_t static_spi_bus_id           = 0;

    struct GpioSubscription {
        types::gpio_slot_t slot  = 0;
        uint8_t port_index       = 0;
        uint32_t subscribed_mask = 0;
        uint32_t last_value      = 0;
        bool used                = false;
    };

    struct GpioMonitorMask {
        types::gpio_slot_t slot = 0;
        uint8_t port_index      = 0;
        uint32_t mask           = 0;
        bool used               = false;
    };

    struct GpioPinRef {
        types::gpio_number_t pin       = -1;
        types::gpio_slot_t slot        = 0;
        types::gpio_local_pin_t local  = 0;
        uint8_t port_index             = 0;
        uint32_t bit_mask              = 0;
        gpio::GPIOGroup::PortAccess pa = {};
    };

    struct PendingGpioBit {
        types::gpio_slot_t slot = 0;
        uint8_t port_index      = 0;
        uint32_t bit_mask       = 0;
    };

    GpioSubscription gpio_subscriptions[kMaxGpioSubscriptionPorts] = {};
    GpioMonitorMask gpio_monitor_masks[kMaxGpioSubscriptionPorts]  = {};
    types::gpio_number_t gpio_snapshot_pins[kMaxGpioSubscriptions] = {};
    bool gpio_snapshot_levels[kMaxGpioSubscriptions]               = {};
    size_t gpio_snapshot_count                                     = 0;
    uint8_t gpio_event_seq                                         = 0;

    static result_t<void> handler(void* ctx, frame::Kind kind, uint8_t seq, data::ConstDataSpan payload,
                                  data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec);
    static result_t<void> gpioSubscribe(void* ctx, bool subscribe, const types::gpio_number_t* pins, size_t count);
    static result_t<void> gpioModeSet(void* ctx, types::gpio_number_t pin, types::gpio_mode_t mode);
    static void gpioPinsClaimed(void* ctx, const types::gpio_number_t* pins, size_t count);
    static result_t<void> poll(void* ctx, data::MuxFrameEncoder& enc);

private:
    void clearGpioSubscriptions();
    void clearGpioMonitorMasks();
    void clearGpioSnapshot();
    result_t<void> appendGpioSnapshot(types::gpio_number_t pin, bool level);
    result_t<void> writeGpioSnapshotEvent(data::MuxFrameEncoder& enc);
    result_t<void> writeGpioSnapshotThenResponse(data::MuxFrameEncoder& enc, uint8_t seq, data::ConstDataSpan response);
    bool resolveGpioPin(types::gpio_number_t pin, GpioPinRef& ref);
    bool decodeGpioPin(types::gpio_number_t pin, types::gpio_slot_t& slot, uint8_t& port_index,
                       uint32_t& bit_mask) const;
    GpioSubscription* findGpioSubscription(types::gpio_slot_t slot, uint8_t port_index);
    GpioSubscription* firstFreeGpioSubscription();
    GpioMonitorMask* findGpioMonitorMask(types::gpio_slot_t slot, uint8_t port_index);
    GpioMonitorMask* firstFreeGpioMonitorMask();
    size_t freeGpioSubscriptionCount() const;
    size_t freeGpioSubscriptionPortCount() const;
    size_t subscribedGpioBitCount() const;
    void removeGpioSubscription(types::gpio_number_t pin);
    static bool hasPendingGpioBit(const PendingGpioBit* bits, size_t count, types::gpio_slot_t slot, uint8_t port_index,
                                  uint32_t bit_mask);
    static bool hasPendingGpioPort(const PendingGpioBit* bits, size_t count, types::gpio_slot_t slot,
                                   uint8_t port_index);
    static size_t countBits(uint32_t value);
    static result_t<void> writeError(data::MuxFrameEncoder& enc, uint8_t seq, error::error_t code);
};

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_SERVER_HANDLER_HPP_
