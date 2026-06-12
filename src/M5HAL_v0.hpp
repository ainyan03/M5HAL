// SPDX-License-Identifier: MIT
#ifndef M5_HAL_V0_HPP
#define M5_HAL_V0_HPP

// =============================================================================
// M5HAL v0 expose entry header (explicit name).
//
// Users who explicitly want v0 include this header directly. Including
// `<M5HAL.hpp>` without naming a version goes through the compatibility
// shim and lands here (v0 is the default for now). See the README for
// the full story.
//
// The published v0.0.x API is exposed through `inline namespace v0`,
// so existing `m5::hal::Foo` references resolve to v0 symbols and
// legacy code needs no edits.
//
// To use the new v1 API, include `<M5HAL_v1.hpp>`. v0 and v1 entry
// headers may coexist in the same translation unit, but each intermediate
// library should still make its intended generation explicit. Details:
// spec/design/v0_v1_coexistence.md.
// =============================================================================

#include "m5_hal_config.hpp"  // M5HAL_INLINE_V0
#include <M5Utility.hpp>

#include "./m5_hal/hal/v0/platform_checker.hpp"
#include "./m5_hal/hal/v0/framework_checker.hpp"
#include "./m5_hal/hal/v0/bus/i2c.hpp"
#include "./m5_hal/hal/v0/bus/spi.hpp"
#include "./m5_hal/hal/v0/bus/bus.hpp"
#include "./m5_hal/hal/v0/interface/gpio.hpp"

#define M5HAL_STATIC_MACRO_STRING(x) #x
// clang-format off
#define M5HAL_STATIC_MACRO_CONCAT(x, y) M5HAL_STATIC_MACRO_STRING(x/y)
// clang-format on

#define M5HAL_STATIC_MACRO_PATH_HEADER M5HAL_STATIC_MACRO_CONCAT(M5HAL_TARGET_PLATFORM_PATH, header.hpp)

#if M5HAL_TARGET_PLATFORM_NUMBER != 0
#include M5HAL_STATIC_MACRO_PATH_HEADER
#endif

#undef M5HAL_STATIC_MACRO_PATH_HEADER

// When the Arduino framework is available, expose the Arduino-backed implementation.
#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include "./m5_hal/hal/v0/frameworks/arduino/header.hpp"

// If no specific target platform is identified, inject the
// Arduino-backed implementation into the v0 namespace.
#if 1  // M5HAL_TARGET_PLATFORM_NUMBER == 0
namespace m5 {
namespace hal {
M5HAL_INLINE_V0 namespace v0 {
using namespace frameworks::arduino;
}
}  // namespace hal
}  // namespace m5
#endif

#elif M5HAL_TARGET_PLATFORM_NUMBER != 0

// Without Arduino (pure ESP-IDF) and with a known platform, use the
// native implementation. `using namespace platforms::esp32;` would
// pull in a nested `platforms::esp32::types` from the first-gen ESP32
// header that collides with `m5::hal::types`, so we cherry-pick only
// the gpio free functions we need into `m5::hal::v0::gpio`.
namespace m5 {
namespace hal {
M5HAL_INLINE_V0 namespace v0 {
namespace gpio {
using platforms::esp32::gpio::getGPIO;
using platforms::esp32::gpio::getPin;
}  // namespace gpio
}  // namespace v0
}  // namespace hal
}  // namespace m5

#endif

#endif  // M5_HAL_V0_HPP
