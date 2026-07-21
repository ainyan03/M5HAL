// SPDX-License-Identifier: MIT
// Sony SPRESENSE wrapper for the canonical public API compile fixture.
#include <Arduino.h>

#if !defined(ARDUINO_ARCH_SPRESENSE) || defined(SUBCORE)
#error "BuildCheckSpresense requires the SPRESENSE MainCore"
#endif

#include <M5HAL_v2.hpp>

static_assert(M5HAL_FRAMEWORK_HAS_BSD_SOCKET == 0, "SPRESENSE NuttX socket transport must remain disabled");
static_assert(M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID == M5HAL_V2_VARIANT_ID_NONE,
              "SPRESENSE has no chip-specific platform variant");

// The sketch is copied into the build tree by arduino-cli, so a relative path
// cannot reach the repository. The workflow passes -I<repo root> instead.
#include "examples/v2/BuildTest/BuildTest.cpp"
