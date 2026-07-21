// SPDX-License-Identifier: MIT
#pragma once

// Fake mirror of ESP-IDF's driver/gpio.h. The espidf slave backend only
// uses this to manually mux SDA/SCL into the I2C peripheral at init() and
// restore the pins at close() -- pure hardware-routing side effects, not
// part of the ISR-driven state machine under test. No-ops throughout.

#include <esp_err.h>

#include <cstdint>

enum gpio_num_t : int {};

enum gpio_mode_t {
    GPIO_MODE_INPUT_OUTPUT_OD,
    GPIO_MODE_OUTPUT,
};

enum gpio_pull_mode_t {
    GPIO_PULLUP_ONLY,
};

inline esp_err_t gpio_set_level(gpio_num_t, uint32_t)
{
    return ESP_OK;
}
inline esp_err_t gpio_set_direction(gpio_num_t, gpio_mode_t)
{
    return ESP_OK;
}
inline esp_err_t gpio_set_pull_mode(gpio_num_t, gpio_pull_mode_t)
{
    return ESP_OK;
}
inline esp_err_t gpio_reset_pin(gpio_num_t)
{
    return ESP_OK;
}

#if defined(M5HAL_TEST_ESPIDF_SPI_SLAVE_HOST_HARNESS)
namespace m5hal_hostharness {
inline int spiCsLevel = 1;
}
inline int gpio_get_level(gpio_num_t)
{
    return m5hal_hostharness::spiCsLevel;
}
#endif
