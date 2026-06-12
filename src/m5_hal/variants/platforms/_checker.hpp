#ifndef M5_HAL_PLATFORM_CHECKER_HPP
#define M5_HAL_PLATFORM_CHECKER_HPP

// v1 platform detection. All macros carry the M5HAL_V1_ prefix so that a
// translation unit may include both generation entries: the frozen v0 tree
// owns the unprefixed M5HAL_TARGET_PLATFORM_* / M5HAL_PLATFORM_NUMBER_*
// names (hal/v0/platform_checker.hpp) and must keep seeing its own values
// (only the NAMES are generation-shared concerns; v1 ids deliberately do
// not carry v0's numbering).
//
// Detection logic only — the identity numbers live in the single
// registry (../ids.hpp). The detected platform is reported as
// M5HAL_V1_TARGET_PLATFORM_VARIANT_ID, a registry value, so it compares
// directly against M5HAL_V1_SELECTED_VARIANT_<KIND> and the
// M5HAL_V1_VARIANT_ID_* constants.

#include "../ids.hpp"

#if defined(ESP_PLATFORM)
#if __has_include(<sdkconfig.h>)
#include <sdkconfig.h>
#endif

// clang-format off
#if defined(CONFIG_IDF_TARGET)
#define M5HAL_V1_TARGET_PLATFORM_VARIANT_ID M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#define M5HAL_V1_TARGET_PLATFORM_PATH       m5_hal/variants/platforms/espressif/esp32
#endif
#else
#endif
// clang-format on

#if !defined(M5HAL_V1_TARGET_PLATFORM_VARIANT_ID)
#define M5HAL_V1_TARGET_PLATFORM_VARIANT_ID M5HAL_V1_VARIANT_ID_NONE
#endif

#endif
