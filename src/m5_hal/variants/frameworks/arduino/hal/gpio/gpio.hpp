// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_GPIO_GPIO_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_GPIO_GPIO_HPP

// GPIO_arduino implementation for the arduino framework variant — wraps the
// Arduino API (`pinMode` / `digitalWrite` / `digitalRead`). The
// encoded value is the gpio_number itself: even on ESP32 boards, the
// Arduino API addresses by gpio_number, so one stateless Port_arduino
// can serve every logical 32-bit port ordinal.
// Authoritative spec: spec/design/gpio.md, spec/design/variants.md.

#include "../../../../../hal/v2/gpio/gpio.hpp"
#include "../../../../../hal/v2/gpio/port.hpp"
#include "../../../../../hal/v2/gpio/group.hpp"
#include "../../../../../hal/v2/assert.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif

#if defined(ESP_PLATFORM) && __has_include(<driver/gpio.h>)
#include <driver/gpio.h>
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_GPIO_HAS_ESPIDF_PULL_ 1
#else
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_GPIO_HAS_ESPIDF_PULL_ 0
#endif

#if defined(ARDUINO)

namespace m5::hal::v2::gpio {

class Port_arduino : public gpio::IPort {
public:
    // Concrete pin validity is delegated to the Arduino API; `kWidth` only caps the range.
    static constexpr uint8_t kWidth = NUM_DIGITAL_PINS;

    constexpr Port_arduino() = default;

protected:
    void _writePinEncoded(uint32_t encoded_num, bool v) override
    {
        // HIGH/LOW retain ArduinoCore-API's PinStatus type on strict cores
        // (ArduinoCore-renesas) while remaining integer macros elsewhere.
        digitalWrite(static_cast<uint8_t>(encoded_num), v ? HIGH : LOW);
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        return digitalRead(static_cast<uint8_t>(encoded_num));
    }
    void _setPinModeEncoded(uint32_t encoded_num, types::gpio_mode_t mode) override
    {
        // Bitfield mode -> Arduino pinMode argument. There is no
        // dedicated `OUTPUT + pull` value, so it falls back to plain
        // `OUTPUT` (the lowlevel POD path can configure pulls
        // independently when needed).
        namespace bits      = types::gpio_mode_bits;
        const uint8_t value = static_cast<uint8_t>(mode);

        // Preserve the core's mode type. ArduinoCore-renesas uses the strict
        // PinMode enum; older cores expose integer macros.
        auto arduino_mode = INPUT;
        if (value & bits::output) {
            // Open-drain output constant name/availability varies by core:
            // arduino-esp32 spells it OUTPUT_OPEN_DRAIN, arduino-pico
            // OUTPUT_OPENDRAIN (no underscore); cores with neither have no
            // dedicated open-drain output mode and fall back to plain
            // OUTPUT (push-pull), same degradation as the missing
            // OUTPUT+pull combination noted above.
#if defined(ARDUINO_ARCH_RENESAS_UNO)
            // ArduinoCore-renesas declares OUTPUT_OPENDRAIN as a PinMode
            // enumerator, so preprocessor existence checks cannot see it.
            arduino_mode = (value & bits::open_drain) ? OUTPUT_OPENDRAIN : OUTPUT;
#elif defined(OUTPUT_OPEN_DRAIN)
            arduino_mode = (value & bits::open_drain) ? OUTPUT_OPEN_DRAIN : OUTPUT;
#elif defined(OUTPUT_OPENDRAIN)
            arduino_mode = (value & bits::open_drain) ? OUTPUT_OPENDRAIN : OUTPUT;
#else
            arduino_mode = OUTPUT;
#endif
        } else if (value & bits::pull_up) {
            arduino_mode = INPUT_PULLUP;
        } else if (value & bits::pull_down) {
#if defined(ARDUINO_ARCH_ESP8266)
            // ESP8266 has no generic pulldown; only GPIO16 has one, with its
            // own mode constant. Other pins fall back to plain INPUT, the
            // same degradation policy as the missing modes above.
            arduino_mode = (encoded_num == 16) ? INPUT_PULLDOWN_16 : INPUT;
#else
            arduino_mode = INPUT_PULLDOWN;
#endif
        } else {
            arduino_mode = INPUT;
        }
        pinMode(static_cast<uint8_t>(encoded_num), arduino_mode);

#if M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_GPIO_HAS_ESPIDF_PULL_
        const auto gpio_num = static_cast<gpio_num_t>(encoded_num);
        (value & bits::pull_up) ? gpio_pullup_en(gpio_num) : gpio_pullup_dis(gpio_num);
        (value & bits::pull_down) ? gpio_pulldown_en(gpio_num) : gpio_pulldown_dis(gpio_num);
#endif
    }
    types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<types::gpio_local_pin_t>(encoded_num);
    }
    uint32_t _fromLocalPin(types::gpio_local_pin_t pin_index) const override
    {
        M5HAL_ASSERT(pin_index < static_cast<types::gpio_local_pin_t>(kWidth), "pin_index out of range");
        return static_cast<uint32_t>(pin_index);
    }
};

// Minimal GPIO_arduino with one stateless Port_arduino shared by all
// logical 32-bit mask words. constexpr singleton —
// `getInstance()` returns a `static constexpr` placed in rodata, so
// there is no guard variable. `_port` is stateless, which keeps the
// `mutable` + rodata combination free of UB.
class GPIO_arduino : public gpio::IGPIO {
public:
    static constexpr uint8_t portCount = (Port_arduino::kWidth + 31u) >> 5;

    gpio::IPort* portForPin(types::gpio_local_pin_t pin_index) const override
    {
        M5HAL_ASSERT(pin_index < static_cast<types::gpio_local_pin_t>(Port_arduino::kWidth), "pin_index out of range");
        return &_port;
    }
    gpio::IPort* getPort(uint8_t portNumber) const override
    {
        M5HAL_ASSERT(portNumber < portCount, "portNumber out of range");
        return &_port;
    }
    uint16_t getPinCount() const override
    {
        return Port_arduino::kWidth;
    }
    uint8_t getPortCount() const override
    {
        return portCount;
    }
    PinLocation locatePin(types::gpio_local_pin_t pin_index) const override
    {
        M5HAL_ASSERT(pin_index < static_cast<types::gpio_local_pin_t>(Port_arduino::kWidth), "pin_index out of range");
        return PinLocation{static_cast<uint8_t>(pin_index >> 5), static_cast<uint8_t>(pin_index & 31u)};
    }

    static const GPIO_arduino* getInstance()
    {
        static constexpr GPIO_arduino s_gpio_instance{};

        return &s_gpio_instance;
    }

protected:
    constexpr GPIO_arduino() = default;

    // Singleton: copy / move deleted (also required to keep the type literal).
    constexpr GPIO_arduino(const GPIO_arduino&)            = delete;
    constexpr GPIO_arduino& operator=(const GPIO_arduino&) = delete;
    constexpr GPIO_arduino(GPIO_arduino&&)                 = delete;
    constexpr GPIO_arduino& operator=(GPIO_arduino&&)      = delete;

private:
    mutable Port_arduino _port{};
};

// MCU-internal GPIO_arduino (constexpr instance). Call directly when a test
// (or similar) needs to talk to this specific implementation.
inline const gpio::IGPIO* getMCUGPIO_arduino()
{
    return GPIO_arduino::getInstance();
}

// Flat-injection entry: the MCU-internal GPIO_arduino as an `IGPIO*` (the
// local pin space). `M5HALCore::ctor` uses this as the slot-0
// bootstrap source.
inline const gpio::IGPIO* getGPIO_arduino()
{
    return getMCUGPIO_arduino();
}

// `getGPIOGroup()` / `pin()` free functions are gone. Global lookups
// go through `m5::hal::v2::M5_Hal.Gpio.getPin(num)`.

}  // namespace m5::hal::v2::gpio

#endif  // ARDUINO

#endif
