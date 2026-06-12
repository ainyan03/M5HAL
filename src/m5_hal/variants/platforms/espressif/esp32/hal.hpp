// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_PLATFORMS_ESPRESSIF_ESP32_HAL_HPP
#define M5_HAL_VARIANTS_PLATFORMS_ESPRESSIF_ESP32_HAL_HPP

#include "../../../../hal/v1/bus/bus.hpp"
#include "hal/gpio/lowlevel.hpp"
#include "hal/gpio/gpio.hpp"
#include "hal/gpio/pin_backup.hpp"

// Chip-level types (placed outside the hal sub-namespace so they do not
// collide with using-directive of ::m5::hal inside hal).
namespace m5::variants::platforms::espressif::esp32::types {

enum class PeripheralType : uint8_t {
    none = 0,
    i2c0,
    i2c1,
    spi2,
    spi3,
};
using periph_t = PeripheralType;

}  // namespace m5::variants::platforms::espressif::esp32::types

#endif
