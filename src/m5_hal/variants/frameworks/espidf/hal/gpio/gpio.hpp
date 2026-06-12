// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_GPIO_GPIO_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_GPIO_GPIO_HPP

// GPIO implementation for the espidf framework variant — wraps the
// ESP-IDF SDK (`gpio_config` / `gpio_set_level` / `gpio_get_level`
// from `driver/gpio.h`). Activation: `ESP_PLATFORM`. Even from
// Arduino-on-IDF builds, this variant can be selected explicitly via
// its alias. The encoded value is the gpio_number itself (one Port
// is enough because `gpio_num_t` already addresses pins directly).
// Authoritative spec: spec/design/gpio.md, spec/design/variants.md.

#include "../../../../../hal/v1/gpio/gpio.hpp"
#include "../../../../../hal/v1/gpio/port.hpp"
#include "../../../../../hal/v1/gpio/group.hpp"
#include "../../../../../hal/v1/assert.hpp"

#if defined(ESP_PLATFORM)

#include "driver/gpio.h"
#include "soc/soc_caps.h"

namespace m5::variants::frameworks::espidf::hal::v1::gpio {

class Port : public ::m5::hal::v1::gpio::IPort {
public:
    // Per chip (esp32 = 40 / s3 = 49 / c6 = 31 / p4 = 55). Out-of-range
    // checking is centralized in `_fromLocalPin`'s assert.
    static constexpr uint8_t kWidth = SOC_GPIO_PIN_COUNT;

    constexpr Port() = default;

protected:
    void _writePinEncoded(uint32_t encoded_num, bool v) override
    {
        gpio_set_level(static_cast<gpio_num_t>(encoded_num), v ? 1 : 0);
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        return gpio_get_level(static_cast<gpio_num_t>(encoded_num)) != 0;
    }
    void _setPinModeEncoded(uint32_t encoded_num, ::m5::hal::v1::types::gpio_mode_t mode) override
    {
        // Bitfield mode -> `gpio_config_t`. Output values pick
        // `INPUT_OUTPUT[_OD]` so that input read stays implicitly
        // enabled — protocols like I2C cannot work without reading
        // the actual line state while driving the bus.
        namespace bits      = ::m5::hal::v1::types::gpio_mode_bits;
        const uint8_t value = static_cast<uint8_t>(mode);

        gpio_config_t cfg = {};
        cfg.pin_bit_mask  = (1ULL << encoded_num);
        cfg.intr_type     = GPIO_INTR_DISABLE;

        cfg.mode = GPIO_MODE_INPUT;
        if (value & bits::output) {
            cfg.mode = (value & bits::open_drain) ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_INPUT_OUTPUT;
        }

        cfg.pull_up_en   = (value & bits::pull_up) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = (value & bits::pull_down) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;

        gpio_config(&cfg);
    }
    ::m5::hal::v1::types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<::m5::hal::v1::types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(::m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        M5HAL_ASSERT(pin_index < static_cast<::m5::hal::v1::types::gpio_local_pin_t>(kWidth), "pin_index out of range");
        return static_cast<uint32_t>(pin_index);
    }
};

// Minimal GPIO with a single built-in Port (one Port covers every
// chip because `gpio_num_t` already addresses pins individually).
// constexpr singleton — `getInstance()` returns a `static constexpr`
// placed in rodata, so there is no guard variable.
class GPIO : public ::m5::hal::v1::gpio::IGPIO {
public:
    ::m5::hal::v1::gpio::IPort* portForPin(::m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        M5HAL_ASSERT(pin_index < static_cast<::m5::hal::v1::types::gpio_local_pin_t>(Port::kWidth),
                     "pin_index out of range");
        return &_port;
    }
    ::m5::hal::v1::gpio::IPort* getPort(uint8_t portNumber) const override
    {
        M5HAL_ASSERT(portNumber == 0, "portNumber must be 0");
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return Port::kWidth;
    }
    uint8_t getPortCount() const override
    {
        return 1;
    }

    static const GPIO* getInstance()
    {
        static constexpr GPIO s_gpio_instance{};

        return &s_gpio_instance;
    }

protected:
    constexpr GPIO() = default;

    // Singleton: copy / move deleted (also required to keep the type literal).
    constexpr GPIO(const GPIO&)            = delete;
    constexpr GPIO& operator=(const GPIO&) = delete;
    constexpr GPIO(GPIO&&)                 = delete;
    constexpr GPIO& operator=(GPIO&&)      = delete;

private:
    mutable Port _port{};
};

// MCU-internal GPIO (constexpr instance).
inline const ::m5::hal::v1::gpio::IGPIO* getMCUGPIO()
{
    return GPIO::getInstance();
}

// Flat-injection entry: the MCU-internal GPIO as an `IGPIO*` (the
// local pin space). `M5HALCore::ctor` uses this as the slot-0
// bootstrap source.
inline const ::m5::hal::v1::gpio::IGPIO* getGPIO()
{
    return getMCUGPIO();
}

// `getGPIOGroup()` / `pin()` free functions are gone. Global lookups
// go through `m5::hal::v1::M5_Hal.Gpio.getPin(num)`.

}  // namespace m5::variants::frameworks::espidf::hal::v1::gpio

#endif  // ESP_PLATFORM

#endif
