// SPDX-License-Identifier: MIT
// Expected-failure fence: an installed M5HAL library may carry the v0
// translation unit on non-ESP Arduino targets, but users must select v2.
#define ARDUINO 10800
#define ARDUINO_ARCH_RP2040

#if defined(M5HAL_TEST_V0_COMPATIBILITY_SHIM)
#include <M5HAL.hpp>
#else
#include <M5HAL_v0.hpp>
#endif
