// SPDX-License-Identifier: MIT
// Sony SPRESENSE MainCore build fence. This intentionally reuses the common
// v2 API surface instead of the hardware-specific public examples.
#include <Arduino.h>

#if !defined(ARDUINO_ARCH_SPRESENSE) || defined(SUBCORE)
#error "BuildCheckSpresense requires the SPRESENSE MainCore"
#endif

#include <build_check.hpp>

static_assert(M5HAL_FRAMEWORK_HAS_BSD_SOCKET == 0, "SPRESENSE NuttX socket transport must remain disabled");
static_assert(M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID == M5HAL_V2_VARIANT_ID_NONE,
              "SPRESENSE has no chip-specific platform variant");

void setup()
{
    m5hal_build_check::v2::compileApiSurface();
}

void loop()
{
}
