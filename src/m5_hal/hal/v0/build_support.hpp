// SPDX-License-Identifier: MIT
#ifndef M5_HAL_V0_BUILD_SUPPORT_HPP
#define M5_HAL_V0_BUILD_SUPPORT_HPP

// v0 remains the ESP32 compatibility surface. Arduino build systems compile
// every translation unit in an installed library, so a non-ESP Arduino target
// must be able to carry M5HAL_v0.cpp without accidentally claiming that the v0
// API works there. Native and pure ESP-IDF builds retain their existing v0
// compile surface; non-ESP Arduino users must select M5HAL_v2.hpp explicitly.
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
#define M5HAL_DETAIL_V0_IMPLEMENTATION_SUPPORTED_ 0
#else
#define M5HAL_DETAIL_V0_IMPLEMENTATION_SUPPORTED_ 1
#endif

#endif  // M5_HAL_V0_BUILD_SUPPORT_HPP
