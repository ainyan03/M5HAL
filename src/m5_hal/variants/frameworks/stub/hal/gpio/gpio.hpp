// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_STUB_HAL_GPIO_GPIO_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_STUB_HAL_GPIO_GPIO_HPP

// GPIO_stub implementation for the `stub` variant — keeps internal state
// in memory so native gtests can observe behaviour. The encoded
// value is the bit position inside the Port_stub (0 .. width - 1).
// Authoritative spec: spec/design/gpio.md, spec/design/variants.md.

#include "../../../../../hal/v2/gpio/port.hpp"
#include "../../../../../hal/v2/gpio/gpio.hpp"
#include "../../../../../hal/v2/gpio/group.hpp"
#include "../../../../../hal/v2/assert.hpp"

namespace m5::hal::v2::gpio {

class Port_stub : public gpio::IPort {
public:
    static constexpr uint8_t kMaxWidth = 32;

    explicit Port_stub(uint8_t width = kMaxWidth, types::gpio_local_pin_t base = 0) : _width{width}, _base{base}
    {
        M5HAL_ASSERT(width <= kMaxWidth, "width exceeds kMaxWidth");
    }

protected:
    void _writePinEncoded(uint32_t encoded_num, bool v) override
    {
        M5HAL_ASSERT(encoded_num < _width, "encoded_num out of range");
        const uint32_t bit = 1u << encoded_num;
        if (v) {
            _state |= bit;
        } else {
            _state &= ~bit;
        }
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        M5HAL_ASSERT(encoded_num < _width, "encoded_num out of range");
        return (_state & (1u << encoded_num)) != 0;
    }
    void _setPinModeEncoded(uint32_t encoded_num, types::gpio_mode_t mode) override
    {
        M5HAL_ASSERT(encoded_num < _width, "encoded_num out of range");
        _modes[encoded_num] = mode;
    }
    types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        M5HAL_ASSERT(encoded_num < _width, "encoded_num out of range");
        return _base + static_cast<types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(types::gpio_local_pin_t pin_index) const override
    {
        M5HAL_ASSERT(pin_index >= _base && pin_index < _base + static_cast<types::gpio_local_pin_t>(_width),
                     "pin_index out of range");
        return static_cast<uint32_t>(pin_index - _base);
    }
    uint32_t _readPortAll() override
    {
        return _state;
    }
    void _writePortMasked(uint32_t set_mask, uint32_t clear_mask) override
    {
        _state &= ~clear_mask;
        _state |= set_mask;
    }

private:
    uint32_t _state{0};
    types::gpio_mode_t _modes[kMaxWidth]{};
    uint8_t _width;
    types::gpio_local_pin_t _base;
};

// One built-in Port_stub (32-bit, base = 0). The mutable state forbids
// `constexpr`, so this is placed in RAM via a function-local static.
// `_port` is `mutable` so the `const` virtual overrides can drive it.
class GPIO_stub : public gpio::IGPIO {
public:
    GPIO_stub() = default;

    gpio::IPort* portForPin(types::gpio_local_pin_t pin_index) const override
    {
        M5HAL_ASSERT(pin_index < static_cast<types::gpio_local_pin_t>(Port_stub::kMaxWidth), "pin_index out of range");
        return &_port;
    }
    gpio::IPort* getPort(uint8_t portNumber) const override
    {
        M5HAL_ASSERT(portNumber == 0, "portNumber must be 0");
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return Port_stub::kMaxWidth;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }

private:
    mutable Port_stub _port{};
};

// MCU-internal GPIO_stub (mutable state means it cannot be constexpr; uses
// a function-local static).
inline const gpio::IGPIO* getMCUGPIO_stub()
{
    static GPIO_stub s_instance;
    return &s_instance;
}

// Flat-injection entry: the MCU-internal GPIO_stub as an `IGPIO*` (the
// local pin space). `M5HALCore::ctor` uses this as the slot-0
// bootstrap source.
inline const gpio::IGPIO* getGPIO_stub()
{
    return getMCUGPIO_stub();
}

// `getGPIOGroup()` / `pin()` free functions are gone. Global lookups
// go through `m5::hal::v2::M5_Hal.Gpio.getPin(num)`.

}  // namespace m5::hal::v2::gpio

#endif
