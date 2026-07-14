// SPDX-License-Identifier: MIT
#ifndef M5_HAL_GPIO_GPIO_HPP_
#define M5_HAL_GPIO_GPIO_HPP_

#include "../types.hpp"
#include "port.hpp"

namespace m5::hal::v2::gpio {

class IGPIO {
public:
    struct PinLocation {
        uint8_t port_index;
        uint8_t bit_index;
    };

    virtual IPort* portForPin(::m5::hal::v2::types::gpio_local_pin_t pin_index) const = 0;

    virtual IPort* getPort(uint8_t portNumber) const = 0;

    virtual uint16_t getPinCount() const = 0;

    virtual uint8_t getPortCount() const = 0;

    virtual bool isValid(::m5::hal::v2::types::gpio_local_pin_t pin_index) const
    {
        return pin_index < getPinCount();
    }

    virtual Pin getPin(::m5::hal::v2::types::gpio_local_pin_t pin_index) const
    {
        return portForPin(pin_index)->getPin(pin_index);
    }

    /*!
      @brief Resolve a local pin to its `getPort()` ordinal and port-mask bit.

      The default derives the port ordinal from `portForPin()` and uses
      `(local_pin & 31)` as the `readPort()` / `writePort()` mask bit.
      IGPIOs with another port-bit layout override this method.  Every
      local pin must have a unique `(port_index, bit_index)` location.
     */
    virtual PinLocation locatePin(::m5::hal::v2::types::gpio_local_pin_t pin_index) const
    {
        if (!isValid(pin_index)) {
            return PinLocation{UINT8_MAX, UINT8_MAX};
        }
        IPort* const target = portForPin(pin_index);
        if (target == nullptr) {
            return PinLocation{UINT8_MAX, UINT8_MAX};
        }
        const uint8_t port_count = getPortCount();
        for (uint8_t port_index = 0; port_index < port_count; ++port_index) {
            if (getPort(port_index) == target) {
                return PinLocation{port_index, static_cast<uint8_t>(pin_index & 31u)};
            }
        }
        return PinLocation{UINT8_MAX, UINT8_MAX};
    }

    /*! @brief Checked `locatePin()` wrapper for external/configured pins. */
    bool tryLocatePin(::m5::hal::v2::types::gpio_local_pin_t pin_index, PinLocation* out_location) const
    {
        if (out_location == nullptr || !isValid(pin_index)) {
            return false;
        }
        const auto location = locatePin(pin_index);
        if (location.port_index >= getPortCount() || location.bit_index >= 32) {
            return false;
        }
        IPort* const port = getPort(location.port_index);
        if (port == nullptr || portForPin(pin_index) != port) {
            return false;
        }
        *out_location = location;
        return true;
    }

    /*!
      @brief Resolve a port ordinal and port-mask bit back to a local pin.

      Returns false when no local pin owns that `(port, bit)` location.
     */
    bool localPinForLocation(uint8_t port_index, uint8_t bit_index,
                             ::m5::hal::v2::types::gpio_local_pin_t* out_pin) const
    {
        if (out_pin == nullptr || port_index >= getPortCount() || bit_index >= 32) {
            return false;
        }
        const uint16_t pin_count = getPinCount();
        for (uint16_t pin = 0; pin < pin_count; ++pin) {
            const auto local = static_cast<::m5::hal::v2::types::gpio_local_pin_t>(pin);
            PinLocation location;
            if (tryLocatePin(local, &location) && location.port_index == port_index &&
                location.bit_index == bit_index) {
                *out_pin = local;
                return true;
            }
        }
        return false;
    }

    /*!
      @brief True when this IGPIO's pin states are fed by push events
             (`GPIOGroup::notifyPinStateChanged`) instead of polling.

      GPIOGroup's watch poll pass skips push-fed entries entirely: each
      (slot, port) has exactly ONE state source (poll or push), which is
      what makes the shadow-XOR edge detection race-free.
     */
    virtual bool hasPushEvents() const
    {
        return false;
    }

protected:
    ~IGPIO() = default;
};

}  // namespace m5::hal::v2::gpio

#endif
