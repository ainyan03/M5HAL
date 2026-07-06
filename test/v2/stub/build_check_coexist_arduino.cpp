// SPDX-License-Identifier: MIT
// v0/v2 coexistence build check for Arduino-based ESP32 envs.
//
// One translation unit includes BOTH public entries, in the direction a
// real migration takes (a legacy v0 sketch starts adding v2 code). The
// native fence (test_coexist_include) already guards include-guard
// collisions; this device fence additionally proves the two platform
// checkers keep their own macro namespaces: v0 owns the unprefixed
// M5HAL_TARGET_PLATFORM_* names, v2 owns M5HAL_V2_TARGET_PLATFORM_*.
#include <M5HAL_v0.hpp>

#include "../build_check/build_check.hpp"  // includes <M5HAL_v2.hpp>

// The v0 macro values must survive the v2 include untouched.
#if defined(CONFIG_IDF_TARGET_ESP32)
static_assert(M5HAL_TARGET_PLATFORM_NUMBER == M5HAL_PLATFORM_NUMBER_ESP32_1st,
              "v0 platform number clobbered by the v2 entry");
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
static_assert(M5HAL_TARGET_PLATFORM_NUMBER == M5HAL_PLATFORM_NUMBER_ESP32_S3,
              "v0 platform number clobbered by the v2 entry");
#endif
static_assert(M5HAL_V2_TARGET_PLATFORM_VARIANT_ID == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32,
              "v2 platform variant id missing or clobbered");

// Both generations' core symbols resolve through their explicit namespaces.
static_assert(static_cast<int>(::m5::hal::v0::error::error_t::OK) == 0, "v0 symbols unusable");
static_assert(static_cast<int>(::m5::hal::v2::error::error_t::OK) == 0, "v2 symbols unusable");

void setup()
{
    m5hal_build_check::v2::compileApiSurface();
}
void loop()
{
}
