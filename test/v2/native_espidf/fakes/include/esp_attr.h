// SPDX-License-Identifier: MIT
#pragma once

// Peripheral-agnostic placement-attribute fake. This harness always builds
// with M5HAL_CONFIG_ESPIDF_I2C_SLAVE_IRAM_ISR=0 (see platformio.ini), so the
// product code never actually applies IRAM_ATTR -- these are defined
// (empty) only so a backend that references them unconditionally still
// compiles, and so future peripherals added to this harness (see
// ../../README.md "Extending to another peripheral") do not need to
// reintroduce this header.

#define IRAM_ATTR
#define DRAM_ATTR
