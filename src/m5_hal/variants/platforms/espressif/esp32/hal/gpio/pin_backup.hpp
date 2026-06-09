#ifndef M5_HAL_VARIANTS_PLATFORMS_ESPRESSIF_ESP32_HAL_GPIO_PIN_BACKUP_HPP
#define M5_HAL_VARIANTS_PLATFORMS_ESPRESSIF_ESP32_HAL_GPIO_PIN_BACKUP_HPP

// Saves / restores the full GPIO-matrix + IO_MUX routing state of a single
// MCU pin, so a pin currently bound to a peripheral (I2C / SPI ...) can be
// borrowed for ad-hoc bit-bang GPIO control and then returned to its exact
// previous role. Mirrors LovyanGFX gpio::pin_backup_t.
//
// The register macros come from the ESP-IDF soc headers and are uniform
// across the ESP32 family (ESP32 / S3 / C3 / C6 / H2 / P4), so a single
// `#if`-guarded implementation covers every chip. Only MCU pins (slot 0 of
// the `gpio_number_t` space) carry these registers; expander pins are a
// no-op. Authoritative spec: spec/design/gpio.md.

#if defined(ESP_PLATFORM)

#include "../../../../../../hal/v1/types.hpp"

#include <stdint.h>
#include <stddef.h>
#include <soc/soc_caps.h>
#include <soc/gpio_reg.h>
#include <soc/gpio_struct.h>  // ::GPIO instance (func_in_sel_cfg)
#include <soc/gpio_periph.h>  // GPIO_PIN_MUX_REG[]
#include <soc/io_mux_reg.h>

namespace m5::variants::platforms::espressif::esp32::hal::v1::gpio {

// Routing-state backup for one MCU pin. `backup()` is explicit (the ctor
// only records the target pin); call `restore()` to write the captured
// registers back. Default / sentinel pin (-1) makes every call a no-op.
class PinBackup {
public:
    using gpio_number_t = ::m5::hal::v1::types::gpio_number_t;

    PinBackup() = default;
    explicit PinBackup(gpio_number_t pin) : _pin_num{pin}
    {
    }

    void setPin(gpio_number_t pin)
    {
        _pin_num = pin;
    }
    gpio_number_t getPin() const
    {
        return _pin_num;
    }

    // Capture the routing registers currently driving the configured pin.
    void backup()
    {
        _in_func_num = -1;
        if (_pin_num < 0 || ::m5::hal::v1::types::extractSlot(_pin_num) != 0) {
            return;  // invalid or non-MCU (expander) pin -> nothing to save
        }
        const size_t pin = ::m5::hal::v1::types::extractLocalPin(_pin_num);
        if (pin >= SOC_GPIO_PIN_COUNT) {
            return;
        }

        _io_mux_gpio_reg   = *reinterpret_cast<volatile uint32_t*>(GPIO_PIN_MUX_REG[pin]);
        _gpio_pin_reg      = *reinterpret_cast<volatile uint32_t*>(GPIO_PIN0_REG + (pin * 4));
        _gpio_func_out_reg = *reinterpret_cast<volatile uint32_t*>(GPIO_FUNC0_OUT_SEL_CFG_REG + (pin * 4));
#if defined(GPIO_ENABLE1_REG)
        _gpio_enable =
            (*reinterpret_cast<volatile uint32_t*>((pin & 32) ? GPIO_ENABLE1_REG : GPIO_ENABLE_REG) >> (pin & 31)) & 1;
#else
        _gpio_enable = (*reinterpret_cast<volatile uint32_t*>(GPIO_ENABLE_REG) >> (pin & 31)) & 1;
#endif

        // If this pin feeds a peripheral input via the GPIO matrix, save that
        // input-routing register too so restore() rewires the input side.
        const size_t func_num = (_gpio_func_out_reg >> GPIO_FUNC0_OUT_SEL_S) & GPIO_FUNC0_OUT_SEL_V;
        // `::GPIO` (global soc instance) — the unqualified `GPIO` would bind to
        // the variant's `class GPIO` in this same namespace.
        const size_t in_count = sizeof(::GPIO.func_in_sel_cfg) / sizeof(::GPIO.func_in_sel_cfg[0]);
        if (func_num < in_count) {
#if defined(GPIO_FUNC0_IN_SEL_CFG_REG)
            const uint32_t in_reg = *reinterpret_cast<volatile uint32_t*>(GPIO_FUNC0_IN_SEL_CFG_REG + (func_num * 4));
            const bool hit        = func_num == ((in_reg >> GPIO_FUNC0_IN_SEL_S) & GPIO_FUNC0_IN_SEL_V);
#else
            const uint32_t in_reg =
                *reinterpret_cast<volatile uint32_t*>(GPIO_FUNC1_IN_SEL_CFG_REG + ((func_num - 1) * 4));
            const bool hit = func_num == ((in_reg >> GPIO_FUNC1_IN_SEL_S) & GPIO_FUNC1_IN_SEL_V);
#endif
            if (hit) {
                _gpio_func_in_reg = in_reg;
                _in_func_num      = static_cast<int16_t>(func_num);
            }
        }
    }
    void backup(gpio_number_t pin)
    {
        setPin(pin);
        backup();
    }

    // Write the saved registers back, returning the pin to its prior role.
    void restore() const
    {
        if (_pin_num < 0 || ::m5::hal::v1::types::extractSlot(_pin_num) != 0) {
            return;
        }
        const size_t pin = ::m5::hal::v1::types::extractLocalPin(_pin_num);
        if (pin >= SOC_GPIO_PIN_COUNT) {
            return;
        }

        if (_in_func_num >= 0) {
            ::GPIO.func_in_sel_cfg[_in_func_num].val = _gpio_func_in_reg;
        }
        *reinterpret_cast<volatile uint32_t*>(GPIO_PIN_MUX_REG[pin])                  = _io_mux_gpio_reg;
        *reinterpret_cast<volatile uint32_t*>(GPIO_PIN0_REG + (pin * 4))              = _gpio_pin_reg;
        *reinterpret_cast<volatile uint32_t*>(GPIO_FUNC0_OUT_SEL_CFG_REG + (pin * 4)) = _gpio_func_out_reg;

#if defined(GPIO_ENABLE1_REG)
        auto enable_reg = reinterpret_cast<volatile uint32_t*>((pin & 32) ? GPIO_ENABLE1_REG : GPIO_ENABLE_REG);
#else
        auto enable_reg = reinterpret_cast<volatile uint32_t*>(GPIO_ENABLE_REG);
#endif
        const uint32_t mask = 1u << (pin & 31);
        if (_gpio_enable) {
            *enable_reg |= mask;
        } else {
            *enable_reg &= ~mask;
        }
    }

private:
    uint32_t _io_mux_gpio_reg   = 0;
    uint32_t _gpio_pin_reg      = 0;
    uint32_t _gpio_func_out_reg = 0;
    uint32_t _gpio_func_in_reg  = 0;
    int16_t _in_func_num        = -1;
    gpio_number_t _pin_num      = -1;
    bool _gpio_enable           = false;
};

// RAII scope guard around PinBackup: captures the pin's routing state on
// construction and restores it on destruction, so a pin borrowed for ad-hoc
// GPIO control is returned to its prior peripheral role automatically on
// scope exit (including early return). Move-only (std::unique_lock style):
// the moved-from guard is disarmed so restore() runs exactly once. Copying
// is forbidden to avoid a double restore. `dismiss()` cancels the pending
// restore when the new pin configuration should be kept.
class ScopedPinBackup {
public:
    using gpio_number_t = PinBackup::gpio_number_t;

    // Empty guard: holds no pin, restores nothing.
    ScopedPinBackup() = default;

    // Capture (backup) the pin immediately; restore() runs at scope exit.
    explicit ScopedPinBackup(gpio_number_t pin) : _backup{pin}
    {
        _backup.backup();
        _armed = true;
    }

    ScopedPinBackup(const ScopedPinBackup&)            = delete;
    ScopedPinBackup& operator=(const ScopedPinBackup&) = delete;

    ScopedPinBackup(ScopedPinBackup&& other) noexcept : _backup{other._backup}, _armed{other._armed}
    {
        other._armed = false;
    }
    ScopedPinBackup& operator=(ScopedPinBackup&& other) noexcept
    {
        if (this != &other) {
            if (_armed) {
                _backup.restore();
            }
            _backup      = other._backup;
            _armed       = other._armed;
            other._armed = false;
        }
        return *this;
    }

    ~ScopedPinBackup()
    {
        if (_armed) {
            _backup.restore();
        }
    }

    // Cancel the pending restore (keep the current pin configuration).
    void dismiss()
    {
        _armed = false;
    }
    // True while a restore is still pending on destruction.
    bool armed() const
    {
        return _armed;
    }
    gpio_number_t getPin() const
    {
        return _backup.getPin();
    }

private:
    PinBackup _backup{};
    bool _armed = false;
};

}  // namespace m5::variants::platforms::espressif::esp32::hal::v1::gpio

// --- public exposure of the chip-level GPIO capabilities ---
//
// PinBackup / ScopedPinBackup are platform (chip) capabilities, not a HAL-kind
// implementation, so their visibility must NOT depend on which variant wins
// the `m5::hal::v1::gpio` flat injection (the offer scan picks one Port / GPIO
// provider). The platform header is always included when an Espressif platform
// is detected, regardless of the GPIO HAL winner, so we expose these symbols
// here directly. Even if a framework variant (arduino / espidf) becomes the
// GPIO HAL provider, `m5::hal::v1::gpio::PinBackup` stays reachable.
//
// These are `using`-declarations (named symbols), not a `using namespace`
// directive, so they coexist with the variant flat injection without
// ambiguity (they name the same entity when the platform variant also wins).
// HAL kinds keep using the offer / winner mechanism; additive chip
// capabilities use this explicit platform-level exposure.
namespace m5::hal::v1::gpio {
using ::m5::variants::platforms::espressif::esp32::hal::v1::gpio::PinBackup;
using ::m5::variants::platforms::espressif::esp32::hal::v1::gpio::ScopedPinBackup;
}  // namespace m5::hal::v1::gpio

#endif  // ESP_PLATFORM

#endif
