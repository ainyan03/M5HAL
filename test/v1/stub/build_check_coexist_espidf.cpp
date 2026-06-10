// SPDX-License-Identifier: MIT
// v0/v1 coexistence build check for pure ESP-IDF envs.
//
// Same intent as build_check_coexist_arduino.cpp: one translation unit
// includes BOTH public entries and proves the two platform checkers keep
// their own macro namespaces (v0 unprefixed, v1 M5HAL_V1_-prefixed) on
// the pure-IDF path, where v0 takes its cherry-picked namespace route.
#include <M5HAL_v0.hpp>

#include "../build_check/build_check.hpp"  // includes <M5HAL_v1.hpp>

// The v0 macro values must survive the v1 include untouched.
#if defined(CONFIG_IDF_TARGET_ESP32)
static_assert(M5HAL_TARGET_PLATFORM_NUMBER == M5HAL_PLATFORM_NUMBER_ESP32_1st,
              "v0 platform number clobbered by the v1 entry");
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
static_assert(M5HAL_TARGET_PLATFORM_NUMBER == M5HAL_PLATFORM_NUMBER_ESP32_S3,
              "v0 platform number clobbered by the v1 entry");
#endif
static_assert(M5HAL_V1_TARGET_PLATFORM_NUMBER == M5HAL_V1_PLATFORM_NUMBER_ESP32,
              "v1 platform number missing or clobbered");

// Both generations' core symbols resolve through their explicit namespaces.
static_assert(static_cast<int>(::m5::hal::v0::error::error_t::OK) == 0, "v0 symbols unusable");
static_assert(static_cast<int>(::m5::hal::v1::error::error_t::OK) == 0, "v1 symbols unusable");

extern "C" void app_main(void)
{
    m5hal_build_check::v1::compileApiSurface();
}
