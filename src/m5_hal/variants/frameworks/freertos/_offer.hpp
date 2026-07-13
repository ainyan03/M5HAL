// SPDX-License-Identifier: MIT
// clang-format off
// Capability self-declaration for the FreeRTOS framework variant.
//
// This file intentionally has NO include guard and NO #pragma once.
// See frameworks/arduino/_offer.hpp for the rationale.
//
// FreeRTOS provides runtime OS primitives (Mutex, Task) used by both
// Arduino-on-IDF and plain ESP-IDF (and any future FreeRTOS-hosted
// framework). Time functions (millis, delayMs, ...) stay with the
// per-framework variant (arduino, espidf) because they depend on
// framework-specific APIs (Arduino core, esp_timer, etc.).
//
// Scan order places FreeRTOS BEFORE arduino/espidf, so it wins
// RUNTIME_MUTEX and RUNTIME_TASK while those frameworks win RUNTIME
// (time functions).

#define M5HAL_VARIANT_CURRENT_ALIAS_   freertos
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::frameworks::freertos
#define M5HAL_VARIANT_CURRENT_ID_      M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS

#define M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_MUTEX_ 1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_TASK_  1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_EVENT_ 1
// clang-format on
