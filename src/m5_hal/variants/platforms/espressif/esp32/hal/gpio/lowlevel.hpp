// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_PLATFORMS_ESPRESSIF_ESP32_HAL_GPIO_LOWLEVEL_HPP
#define M5_HAL_VARIANTS_PLATFORMS_ESPRESSIF_ESP32_HAL_GPIO_LOWLEVEL_HPP

// POD path that hits the ESP32 (classic) GPIO registers directly.
// Independent low-level prototype that does not go through the
// `m5::hal::v1::gpio::Pin` / `Port` abstractions.

#if defined(ESP_PLATFORM)

#include "../../../../../../hal/v1/types.hpp"

#include <stdint.h>
#include <soc/gpio_reg.h>
#include <soc/soc_caps.h>
#include <hal/gpio_types.h>
#include <driver/gpio.h>

namespace m5::variants::platforms::espressif::esp32::hal::v1::gpio::lowlevel {

using ::m5::hal::v1::types::gpio_mode_t;
using ::m5::hal::v1::types::gpio_number_t;

// GPIO register triple: [0] = W1TC (clear output bits), [1] = W1TS (set output bits), [2] = IN (read input).
struct RegSet {
    volatile uint32_t* reg_desc[3];

    inline void writeHigh(uint32_t pin_mask) const
    {
        *reg_desc[1] = pin_mask;
    }
    inline void writeLow(uint32_t pin_mask) const
    {
        *reg_desc[0] = pin_mask;
    }
    inline void write(uint32_t pin_mask, bool v) const
    {
        *reg_desc[v ? 1 : 0] = pin_mask;
    }
    inline bool read(uint32_t pin_mask) const
    {
        return (*reg_desc[2] & pin_mask) != 0;
    }
    inline uint32_t readAll() const
    {
        return *reg_desc[2];
    }
};

// POD describing a single pin: pointer to its RegSet, bit mask, cached pin number.
struct PinHandle {
    const RegSet* regs;
    uint32_t pin_mask;
    gpio_number_t num;

    inline void writeHigh() const
    {
        regs->writeHigh(pin_mask);
    }
    inline void writeLow() const
    {
        regs->writeLow(pin_mask);
    }
    inline void write(bool v) const
    {
        regs->write(pin_mask, v);
    }
    inline bool read() const
    {
        return regs->read(pin_mask);
    }
    inline bool isValid() const
    {
        return regs != nullptr && regs->reg_desc[0] != nullptr;
    }
};

inline const RegSet kInvalidRegSet{{nullptr, nullptr, nullptr}};

inline const RegSet kRegSets[] = {
    {{reinterpret_cast<volatile uint32_t*>(GPIO_OUT_W1TC_REG), reinterpret_cast<volatile uint32_t*>(GPIO_OUT_W1TS_REG),
      reinterpret_cast<volatile uint32_t*>(GPIO_IN_REG)}},
#if SOC_GPIO_PIN_COUNT > 32
    {{reinterpret_cast<volatile uint32_t*>(GPIO_OUT1_W1TC_REG),
      reinterpret_cast<volatile uint32_t*>(GPIO_OUT1_W1TS_REG), reinterpret_cast<volatile uint32_t*>(GPIO_IN1_REG)}},
#endif
};

inline constexpr uint8_t kRegSetCount = sizeof(kRegSets) / sizeof(kRegSets[0]);

inline const RegSet* invalidRegSet()
{
    return &kInvalidRegSet;
}

inline constexpr const RegSet* getRegSetAt(uint8_t index)
{
    return index < kRegSetCount ? &kRegSets[index] : &kInvalidRegSet;
}

inline const RegSet* getRegSetByPin(gpio_number_t num)
{
    if (num < 0 || num >= SOC_GPIO_PIN_COUNT) {
        return &kInvalidRegSet;
    }
    return getRegSetAt(static_cast<uint8_t>(num) >> 5);
}

inline PinHandle pin(gpio_number_t num)
{
    PinHandle h{&kInvalidRegSet, 0, num};
    if (num < 0 || num >= SOC_GPIO_PIN_COUNT) {
        return h;
    }
    h.regs     = getRegSetByPin(num);
    h.pin_mask = 1u << (static_cast<uint8_t>(num) & 31);
    return h;
}

inline bool setMode(PinHandle h, gpio_mode_t mode)
{
    if (!h.isValid()) {
        return false;
    }
    namespace bits      = ::m5::hal::v1::types::gpio_mode_bits;
    const uint8_t value = static_cast<uint8_t>(mode);

    gpio_config_t cfg = {};
    cfg.pin_bit_mask  = 1ULL << static_cast<uint8_t>(h.num);
    cfg.intr_type     = GPIO_INTR_DISABLE;
    cfg.pull_up_en    = (value & bits::pull_up) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    cfg.pull_down_en  = (value & bits::pull_down) ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;

    cfg.mode = GPIO_MODE_INPUT;
    if (value & bits::output) {
        cfg.mode = (value & bits::open_drain) ? GPIO_MODE_INPUT_OUTPUT_OD : GPIO_MODE_INPUT_OUTPUT;
    }
    return gpio_config(&cfg) == ESP_OK;
}

}  // namespace m5::variants::platforms::espressif::esp32::hal::v1::gpio::lowlevel

#endif  // ESP_PLATFORM

#endif
