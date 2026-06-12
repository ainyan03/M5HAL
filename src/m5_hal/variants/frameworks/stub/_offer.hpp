// SPDX-License-Identifier: MIT
// clang-format off
// Capability self-declaration for the stub framework variant.
//
// This file intentionally has NO include guard and NO #pragma once.
// See frameworks/arduino/_offer.hpp for the rationale.
//
// The stub variant is the unconditional fallback used when no other
// variant provides a given HAL. It declares HAS_HAL_*_ for every HAL
// kind for which it ships a no-op concrete (see header.hpp).

#define M5HAL_VARIANT_CURRENT_ALIAS_   stub
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::frameworks::stub
#define M5HAL_VARIANT_CURRENT_ID_      M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB

#define M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_    1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_ 1
// I2C / SPI / UART are not yet declared because their abstract bases
// are not finalized. As stubs are added in later stages, the matching
// HAS_HAL_*_ macros will be appended here.
// clang-format on
