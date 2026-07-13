// SPDX-License-Identifier: MIT
#ifndef M5_HAL_GPIO_GPIO_HPP_
#define M5_HAL_GPIO_GPIO_HPP_

#include "../types.hpp"
#include "port.hpp"

namespace m5::hal::v2::gpio {

class IGPIO {
public:
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
