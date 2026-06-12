#ifndef M5_HAL_FRAMEWORK_CHECKER_HPP
#define M5_HAL_FRAMEWORK_CHECKER_HPP

// ARDUINO / FREERTOS / SDL are also defined, with the SAME names, by the
// frozen hal/v0/framework_checker.hpp. Identical-token redefinition is
// well-formed, so sharing is intentional — but any edit here that changes
// those three definitions breaks the contract and shows up as a
// redefinition warning in the coexist fences (test_coexist_include,
// v0v1_check_*). v1-only additions (ESPIDF, POSIX, ...) are free.
// See spec/design/v0_v1_coexistence.md §制約.

#if defined(ARDUINO)
#define M5HAL_FRAMEWORK_HAS_ARDUINO 1
#else
#define M5HAL_FRAMEWORK_HAS_ARDUINO 0
#endif

// The arduino variant is implemented against the arduino-esp32 core only
// (TwoWire::begin(sda, scl), SPIClass::transferBytes, ...), and no other
// variant covers a non-ESP32 Arduino core either (software depends on
// <thread>). Fail loudly here instead of letting the build die deep
// inside a variant header. Remove this gate when a non-ESP32 Arduino
// core gains a supported variant set. See spec/design/variants.md.
#if defined(ARDUINO) && !defined(ESP_PLATFORM)
#error "M5HAL currently supports the arduino-esp32 core only; this Arduino core is not yet supported."
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
// implementation. Set M5HAL_CONFIG_POSIX_UART=0 to suppress it (e.g. a host
// test that wants UART left unprovided).
#ifndef M5HAL_CONFIG_POSIX_UART
#define M5HAL_CONFIG_POSIX_UART 1
#endif
#if M5HAL_CONFIG_POSIX_UART && !defined(ESP_PLATFORM) && !defined(ARDUINO) && __has_include(<termios.h>)
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
