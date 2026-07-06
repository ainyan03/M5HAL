// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_GPIO_GPIO_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_GPIO_GPIO_HPP

#include "../../../../../hal/v2/assert.hpp"
#include "../../../../../hal/v2/data/memory.hpp"
#include "../../../../../hal/v2/gpio/gpio.hpp"
#include "../../../../../hal/v2/gpio/group.hpp"
#include "../../../../../hal/v2/gpio/port.hpp"
#include "../../../../../hal/v2/memory/allocator.hpp"
#include "../../../../../hal/v2/types.hpp"
#include "../../session.hpp"

#include <cstddef>
#include <cstdint>

namespace m5::hal::v2::gpio {

using remote::RemoteSession;

class GPIO_remote;

class Port_remote : public gpio::IPort {
public:
    Port_remote() = default;

    void init(RemoteSession* session, types::gpio_slot_t device_slot, uint8_t port_index)
    {
        _session     = session;
        _device_slot = device_slot;
        _port_index  = port_index;
    }

    void setCachedBit(uint32_t mask)
    {
        _cached |= mask;
    }
    void clearCachedBit(uint32_t mask)
    {
        _cached &= ~mask;
    }
    void setCachedValue(uint32_t val)
    {
        _cached = val;
    }

    result_t<void> syncRead();

protected:
    void _writePinEncoded(uint32_t pin_mask, bool v) override;
    void _writePinEncodedHigh(uint32_t pin_mask) override;
    void _writePinEncodedLow(uint32_t pin_mask) override;
    bool _readPinEncoded(uint32_t pin_mask) override;
    void _setPinModeEncoded(uint32_t pin_mask, types::gpio_mode_t mode) override;
    uint32_t _readPortAll() override;
    void _writePortMasked(uint32_t set_mask, uint32_t clear_mask) override;
    types::gpio_local_pin_t _toLocalPin(uint32_t pin_mask) const override;
    uint32_t _fromLocalPin(types::gpio_local_pin_t pin_index) const override;

private:
    RemoteSession* _session         = nullptr;
    types::gpio_slot_t _device_slot = 0;
    uint8_t _port_index             = 0;
    uint32_t _cached                = 0;

    friend class GPIO_remote;
};

class GPIO_remote final : public gpio::IGPIO {
public:
    static constexpr size_t kMaxPorts = 2;

    GPIO_remote(RemoteSession& session, types::gpio_slot_t device_slot, uint8_t port_count, uint16_t pin_count);

    gpio::IPort* portForPin(types::gpio_local_pin_t pin_index) const override;
    gpio::IPort* getPort(uint8_t port_number) const override;
    uint16_t getPinCount() const override;
    uint8_t getPortCount() const override;
    result_t<void> seedCache();
    result_t<void> subscribeAll();

    void bindEventGroup(GPIOGroup* group, types::gpio_slot_t slot)
    {
        _event_group = group;
        _event_slot  = slot;
    }

    static void onGpioEvent(void* ctx, types::gpio_number_t pin, bool level);
    static void onSessionEvent(void* ctx, uint8_t seq, data::ConstDataSpan body);

private:
    mutable Port_remote _ports[kMaxPorts];
    uint8_t _port_count            = 0;
    uint16_t _pin_count            = 0;
    GPIOGroup* _event_group        = nullptr;
    types::gpio_slot_t _event_slot = 0;
};

}  // namespace m5::hal::v2::gpio

// Backward compatibility: existing code uses remote::RemoteGPIO / remote::RemotePort.
namespace m5::hal::v2::remote {
using RemoteGPIO = gpio::GPIO_remote;
using RemotePort = gpio::Port_remote;
}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_GPIO_GPIO_HPP
