#ifndef M5_HAL_FRAMEWORK_CHECKER_HPP
#define M5_HAL_FRAMEWORK_CHECKER_HPP

#if defined(ARDUINO)
#define M5HAL_FRAMEWORK_HAS_ARDUINO 1
#else
#define M5HAL_FRAMEWORK_HAS_ARDUINO 0
#endif

// ESP-IDF detection. ESP_PLATFORM means the ESP-IDF API surface is
// available, including Arduino-on-IDF and ESP-IDF projects that add
// Arduino as a component. When multiple framework variants offer the same
// HAL kind, scan order in M5HAL_v1.hpp decides the default flat injection.
#if defined(ESP_PLATFORM)
#define M5HAL_FRAMEWORK_HAS_ESPIDF 1
#else
#define M5HAL_FRAMEWORK_HAS_ESPIDF 0
#endif

#if __has_include(<FreeRTOS.h>) || __has_include(<freertos/FreeRTOS.h>)
#define M5HAL_FRAMEWORK_HAS_FREERTOS 1
#else
#define M5HAL_FRAMEWORK_HAS_FREERTOS 0
#endif

// POSIX host framework variant (termios serial). Opt-out, not opt-in: on a
// plain POSIX host (no Arduino / ESP-IDF SDK in the build) the host serial
// port is what a real application wants, so posix is the default UART
// provider there. stub keeps no-op'ing the HAL kinds that have no host
// implementation. Define M5HAL_DISABLE_POSIX to suppress it (e.g. a host
// test that wants UART left unprovided).
#if !defined(M5HAL_DISABLE_POSIX) && !defined(ESP_PLATFORM) && !defined(ARDUINO) && __has_include(<termios.h>)
#define M5HAL_FRAMEWORK_HAS_POSIX 1
#else
#define M5HAL_FRAMEWORK_HAS_POSIX 0
#endif

#if __has_include(<SDL2/SDL.h>) || __has_include(<SDL.h>)
#define M5HAL_FRAMEWORK_HAS_SDL 1
#else
#define M5HAL_FRAMEWORK_HAS_SDL 0
#endif

#endif
