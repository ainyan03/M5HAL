// SPDX-License-Identifier: MIT
#pragma once

// Fake mirror of ESP-IDF's esp_rom_gpio.h. Like driver/gpio.h, only used by
// the espidf slave backend's init()-time GPIO-matrix routing (hardware
// wiring side effects, not part of the ISR-driven state machine). No-ops.

#include <driver/gpio.h>

#include <cstdint>

inline void esp_rom_gpio_pad_select_gpio(gpio_num_t)
{
}
inline void esp_rom_gpio_connect_out_signal(gpio_num_t, uint32_t, bool, bool)
{
}
inline void esp_rom_gpio_connect_in_signal(gpio_num_t, uint32_t, bool)
{
}
