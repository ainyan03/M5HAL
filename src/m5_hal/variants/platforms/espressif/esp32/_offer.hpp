// SPDX-License-Identifier: MIT
// clang-format off
// Capability self-declaration for the Espressif ESP32 (1st gen) platform variant.
//
// This file intentionally has NO include guard and NO #pragma once.
// See frameworks/arduino/_offer.hpp for the rationale.

#define M5HAL_VARIANT_CURRENT_ALIAS_   esp32
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::platforms::espressif::esp32
#define M5HAL_VARIANT_CURRENT_ID_      M5HAL_V1_VARIANT_ID_PLATFORM_ESP32

// Concrete Port / GPIO have been shipped under
// platforms::espressif::esp32::hal::v1::gpio, so the capability is
// enabled. Platform variants come earlier than framework variants
// in the scan order, so the ESP32 (1st gen) build picks this
// platform's GPIO and activates the W1TS / W1TC direct-register
// fast path (the `_writePinEncodedHigh/Low` overrides).
#define M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_ 1
// clang-format on
