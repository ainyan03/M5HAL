#ifndef M5_HAL_PLATFORM_CHECKER_HPP
#define M5_HAL_PLATFORM_CHECKER_HPP

// v1 platform detection. All macros carry the M5HAL_V1_ prefix so that a
// translation unit may include both generation entries: the frozen v0 tree
// owns the unprefixed M5HAL_TARGET_PLATFORM_* / M5HAL_PLATFORM_NUMBER_*
// names (hal/v0/platform_checker.hpp) and must keep seeing its own values.

#define M5HAL_V1_PLATFORM_NUMBER_UNKNOWN 0
#define M5HAL_V1_PLATFORM_NUMBER_WINDOWS 10
#define M5HAL_V1_PLATFORM_NUMBER_MACOS   20
#define M5HAL_V1_PLATFORM_NUMBER_LINUX   30

#define M5HAL_V1_PLATFORM_NUMBER_SDL_MAX 99

#define M5HAL_V1_PLATFORM_NUMBER_AVR       200
#define M5HAL_V1_PLATFORM_NUMBER_ESP8266   300
#define M5HAL_V1_PLATFORM_NUMBER_ESP32     310
#define M5HAL_V1_PLATFORM_NUMBER_RP2040    400
#define M5HAL_V1_PLATFORM_NUMBER_SAMD21    500
#define M5HAL_V1_PLATFORM_NUMBER_SAMD51    510
#define M5HAL_V1_PLATFORM_NUMBER_SPRESENSE 600
#define M5HAL_V1_PLATFORM_NUMBER_STM32     700

#if defined(ESP_PLATFORM)
#if __has_include(<sdkconfig.h>)
#include <sdkconfig.h>
#endif

// clang-format off
#if defined(CONFIG_IDF_TARGET)
#define M5HAL_V1_TARGET_PLATFORM_NUMBER M5HAL_V1_PLATFORM_NUMBER_ESP32
#define M5HAL_V1_TARGET_PLATFORM_PATH   m5_hal/variants/platforms/espressif/esp32
#endif
#else
#endif
// clang-format on

#if !defined(M5HAL_V1_TARGET_PLATFORM_NUMBER)
#define M5HAL_V1_TARGET_PLATFORM_NUMBER M5HAL_V1_PLATFORM_NUMBER_UNKNOWN
#endif

#endif
