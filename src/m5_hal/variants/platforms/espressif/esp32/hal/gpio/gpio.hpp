// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_PLATFORMS_ESPRESSIF_ESP32_HAL_GPIO_GPIO_HPP
#define M5_HAL_VARIANTS_PLATFORMS_ESPRESSIF_ESP32_HAL_GPIO_GPIO_HPP

// GPIO implementation for the ESP32 family platform variant — direct
// W1TS / W1TC register writes via the lowlevel RegSet defined in
// lowlevel.hpp. Platform variants beat arduino / espidf in the scan
// order, and SDK macros (`SOC_GPIO_PIN_COUNT` etc.) cover every chip
// (ESP32 / S2 / S3 / C3 / C6 / P4) inside a single variant. The
// encoded value is the raw `pin_mask`; chips with
// `SOC_GPIO_PIN_COUNT > 32` are dispatched bank by bank using
// `pin_index >> 5`.
// Authoritative spec: spec/design/gpio.md, spec/design/variants.md.

#include "../../../../../../hal/v1/gpio/gpio.hpp"
#include "../../../../../../hal/v1/gpio/port.hpp"
#include "../../../../../../hal/v1/gpio/group.hpp"
#include "../../../../../../hal/v1/assert.hpp"
#include "lowlevel.hpp"

#if defined(ESP_PLATFORM)

#include <soc/soc_caps.h>

namespace m5::variants::platforms::espressif::esp32::hal::v1::gpio {

class Port : public ::m5::hal::v1::gpio::IPort {
public:
    // Out-of-range checking is centralized in `_fromLocalPin`'s assert.
    static constexpr uint8_t kWidth = SOC_GPIO_PIN_COUNT;

    constexpr Port(uint8_t port_index) : _regs{lowlevel::getRegSetAt(port_index)}, _port_index{port_index}
    {
    }

protected:
    const lowlevel::RegSet* _regs{};
    const uint8_t _port_index = 0;

    void _writePinEncoded(uint32_t encoded_num, bool v) override
    {
        _regs->write(encoded_num, v);
    }
    // Direct W1TS / W1TC write (value-agnostic, lighter than `gpio_set_level`).
    void _writePinEncodedHigh(uint32_t encoded_num) override
    {
        _regs->writeHigh(encoded_num);
    }
    void _writePinEncodedLow(uint32_t encoded_num) override
    {
        _regs->writeLow(encoded_num);
    }
    bool _readPinEncoded(uint32_t encoded_num) override
    {
        return _regs->read(encoded_num);
    }
    void _setPinModeEncoded(uint32_t encoded_num, ::m5::hal::v1::types::gpio_mode_t mode) override
    {
        // Bitfield mode -> gpio_config_t (the mapping lives in lowlevel.hpp).
        lowlevel::setMode(lowlevel::pin(_toLocalPin(encoded_num)), mode);
    }
    ::m5::hal::v1::types::gpio_local_pin_t _toLocalPin(uint32_t encoded_num) const override
    {
        return static_cast<::m5::hal::v1::types::gpio_local_pin_t>(__builtin_ctz(encoded_num)) + (_port_index << 5);
    }
    uint32_t _fromLocalPin(::m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        M5HAL_ASSERT(pin_index < static_cast<::m5::hal::v1::types::gpio_local_pin_t>(kWidth), "pin_index out of range");
        // Bank membership: without this, a pin from another bank would
        // silently alias onto `pin_index & 31` of this bank (e.g.
        // `getPort(0)->write(40, v)` driving GPIO8).
        M5HAL_ASSERT((pin_index >> 5) == _port_index, "pin_index %u not in port %u", static_cast<unsigned>(pin_index),
                     static_cast<unsigned>(_port_index));
        return 1u << (pin_index & 31);  // pin_mask. `1u <<` avoids signed shift UB.
    }
};

// Holds multiple Ports (one Port per 32 bits) in an array. Chips with
// `SOC_GPIO_PIN_COUNT > 32` (s3 / p4) dispatch per bank. constexpr
// singleton: `getInstance()` returns a `static constexpr` placed in
// rodata, so there is no guard variable.
class GPIO : public ::m5::hal::v1::gpio::IGPIO {
public:
    ::m5::hal::v1::gpio::IPort* portForPin(::m5::hal::v1::types::gpio_local_pin_t pin_index) const override
    {
        M5HAL_ASSERT(pin_index < static_cast<::m5::hal::v1::types::gpio_local_pin_t>(Port::kWidth),
                     "pin_index out of range");
        // `pin_index >> 5` selects the bank (the old `_port[portCount]` indexing was OOB).
        return &_port[pin_index >> 5];
    }
    ::m5::hal::v1::gpio::IPort* getPort(uint8_t port_index) const override
    {
        M5HAL_ASSERT(port_index < GPIO::portCount, "port_index out of range");
        return &_port[port_index];
    }
    uint16_t getPinCount() const override
    {
        return Port::kWidth;
    }
    uint8_t getPortCount() const override
    {
        return GPIO::portCount;
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
    // Bank count = ceil(SOC_GPIO_PIN_COUNT / 32).
    static constexpr uint8_t portCount = (SOC_GPIO_PIN_COUNT + 31) >> 5;
    mutable Port _port[portCount]      = {
        0,
#if SOC_GPIO_PIN_COUNT > 32
        1,
#if SOC_GPIO_PIN_COUNT > 64
        2,
#endif
#endif
    };
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

}  // namespace m5::variants::platforms::espressif::esp32::hal::v1::gpio

// Winner-bindable names in the kind namespace (variant naming rule,
// spec/design/variants.md). The implementation stays in the platform
// namespace because it leans on the lowlevel register layer; these
// bridges keep the `_esp32`-suffixed names addressable next to the
// framework variants.
namespace m5::hal::v1::gpio {
using Port_esp32 = ::m5::variants::platforms::espressif::esp32::hal::v1::gpio::Port;
using GPIO_esp32 = ::m5::variants::platforms::espressif::esp32::hal::v1::gpio::GPIO;
inline const IGPIO* getMCUGPIO_esp32(void)
{
    return ::m5::variants::platforms::espressif::esp32::hal::v1::gpio::getMCUGPIO();
}
inline const IGPIO* getGPIO_esp32(void)
{
    return ::m5::variants::platforms::espressif::esp32::hal::v1::gpio::getGPIO();
}
}  // namespace m5::hal::v1::gpio

#endif  // ESP_PLATFORM

#endif
