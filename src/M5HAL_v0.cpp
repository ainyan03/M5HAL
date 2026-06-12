// SPDX-License-Identifier: MIT
// M5HAL v0 translation unit.
//
// Aggregates the v0 (published v0.0.x API) `.inl` files. The v1
// translation unit lives in M5HAL_v1.cpp. v0 and v1 share macros
// (`M5HAL_FRAMEWORK_HAS_*` / `M5HAL_TARGET_PLATFORM_*`) whose
// definitions differ between the two, so each `.inl` set has to be
// included from its own TU. See spec/design/v0_v1_coexistence.md
// §制約 for the full reasoning.

#include "M5HAL_v0.hpp"

#include "./m5_hal/hal/v0/bus/bus.inl"
#include "./m5_hal/hal/v0/bus/i2c.inl"
#include "./m5_hal/hal/v0/bus/spi.inl"

#define M5HAL_STATIC_MACRO_PATH_IMPL M5HAL_STATIC_MACRO_CONCAT(M5HAL_TARGET_PLATFORM_PATH, impl.inl)

#if M5HAL_TARGET_PLATFORM_NUMBER != 0
#include M5HAL_STATIC_MACRO_PATH_IMPL
#endif

// Pull in the Arduino-backed implementation only when Arduino is available.
#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include "./m5_hal/hal/v0/frameworks/arduino/impl.inl"
#endif
