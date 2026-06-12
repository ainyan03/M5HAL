// SPDX-License-Identifier: MIT
// clang-format off
// Capability self-declaration for the ESP-IDF framework variant.
//
// This file intentionally has NO include guard and NO #pragma once.
// See frameworks/arduino/_offer.hpp for the rationale.
//
// `M5HAL_FRAMEWORK_HAS_ESPIDF` (frameworks/_checker.hpp) is `ESP_PLATFORM`.
// Arduino-on-IDF can therefore have both arduino and espidf variants; scan
// order controls the default flat injection while variant aliases keep both
// implementations addressable.

#include "detail/espidf_version.hpp"

#define M5HAL_VARIANT_CURRENT_ALIAS_   espidf
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::frameworks::espidf
#define M5HAL_VARIANT_CURRENT_ID_      M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF

#define M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_ 1
#if M5HAL_ESPIDF_I2C_HAS_MASTER
#define M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_  1
#endif
#if M5HAL_ESPIDF_SPI_HAS_MASTER
#define M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_  1
#endif
#define M5HAL_VARIANT_CURRENT_HAS_HAL_UART_ 1
#if M5HAL_ESPIDF_I2S_HAS_STD
#define M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_ 1
#endif
// clang-format on
