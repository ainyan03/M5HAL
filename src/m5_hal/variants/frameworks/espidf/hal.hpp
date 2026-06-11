#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_HPP

// Per-kind hub for the ESP-IDF framework variant. Each subfile is
// self-contained: `ESP_PLATFORM` gating, namespace scaffolding, and
// the `using namespace ::m5::hal::v1;` resolver all live inside the
// per-kind file. This hub is a thin file that only lists the kinds
// this variant offers.
#include "hal/gpio/gpio.hpp"
#include "hal/i2c/i2c.hpp"
#include "hal/spi/spi.hpp"
#include "hal/uart/uart.hpp"
#include "hal/i2s/i2s.hpp"

#endif
