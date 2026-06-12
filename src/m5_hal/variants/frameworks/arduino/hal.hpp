// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_HPP

// Per-kind hub for the arduino framework variant. Each subfile is
// self-contained: ARDUINO gating, namespace scaffolding, and the
// `using namespace ::m5::hal::v1;` resolver are local to the kind file.
// The hub only enumerates the kinds this variant offers.
#include "hal/gpio/gpio.hpp"
#include "hal/i2c/i2c.hpp"
#include "hal/spi/spi.hpp"
#include "hal/uart/uart.hpp"

#endif
