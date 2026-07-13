// SPDX-License-Identifier: MIT
#ifndef M5_HAL_FRAMEWORK_CHECKER_HPP
#define M5_HAL_FRAMEWORK_CHECKER_HPP

// ARDUINO / FREERTOS / SDL are also defined, with the SAME names, by the
// frozen hal/v0/framework_checker.hpp. Identical-token redefinition is
// well-formed, so sharing is intentional — but any edit here that changes
// those three definitions breaks the contract and shows up as a
// redefinition warning in the coexist fences (test_coexist_include,
// v0v2_check_*). v2-only additions (ESPIDF, POSIX, ...) are free.
// See spec/design/v0_v2_coexistence.md §制約.

#if defined(ARDUINO)
#define M5HAL_FRAMEWORK_HAS_ARDUINO 1
#else
#define M5HAL_FRAMEWORK_HAS_ARDUINO 0
#endif

// The arduino variant targets arduino-esp32 plus a short allowlist of other
// Arduino cores that the variant's ESP_PLATFORM-gated fallback paths (Wire /
// SPI begin() overloads, runtime::Mutex) have been written against: RP2040
// (arduino-pico), SAMD51 (Adafruit/Arduino SAMD core), STM32 (official
// ST core / STM32duino), nRF52840 (Adafruit nRF52 core), and ESP8266
// (official ESP8266 Arduino core — Espressif silicon but NOT ESP_PLATFORM;
// it takes the generic paths except where noted per call site). None is
// HIL-verified (build-check only, see spec/design/variants.md); other
// Arduino cores still fail loudly here instead of dying deep inside a
// variant header. Widen this allowlist only after adding the matching
// fallback path.
//
// ARDUINO_ARCH_MBED is excluded explicitly: ArduinoCore-mbed's RP2040 boards
// also define ARDUINO_ARCH_RP2040, but that core is a different, unverified
// surface (Mbed OS underneath, its own GCC toolchain whose <chrono> the
// Arduino.h abs()/round() macros corrupt) — only earlephilhower's
// arduino-pico core has been written against here.
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
#define M5HAL_ARDUINO_VARIANT_SUPPORTED_ 1
#elif defined(ARDUINO_ARCH_SAMD) && defined(__SAMD51__)
#define M5HAL_ARDUINO_VARIANT_SUPPORTED_ 1
#elif defined(ARDUINO_ARCH_STM32)
#define M5HAL_ARDUINO_VARIANT_SUPPORTED_ 1
// Adafruit nRF52 core only: the sandeepmistry core (ARDUINO_ARCH_NRF5, no
// ARDUINO_NRF52_ADAFRUIT) and Arduino's mbed-based nRF boards (Nano 33 BLE:
// ARDUINO_ARCH_MBED) are different, unverified surfaces.
#elif defined(ARDUINO_ARCH_NRF52) && defined(ARDUINO_NRF52_ADAFRUIT)
#define M5HAL_ARDUINO_VARIANT_SUPPORTED_ 1
#elif defined(ARDUINO_ARCH_ESP8266)
#define M5HAL_ARDUINO_VARIANT_SUPPORTED_ 1
#elif defined(ESP_PLATFORM)
#define M5HAL_ARDUINO_VARIANT_SUPPORTED_ 1
#else
#define M5HAL_ARDUINO_VARIANT_SUPPORTED_ 0
#endif

#if defined(ARDUINO) && !M5HAL_ARDUINO_VARIANT_SUPPORTED_
#error \
    "M5HAL's arduino variant supports arduino-esp32, RP2040 (arduino-pico), SAMD51, STM32 (official ST core), nRF52840 (Adafruit nRF52 core), and ESP8266 only; this Arduino core is not yet supported."
#endif

#undef M5HAL_ARDUINO_VARIANT_SUPPORTED_

// ESP-IDF detection. ESP_PLATFORM means the ESP-IDF API surface is
// available, including Arduino-on-IDF and ESP-IDF projects that add
// Arduino as a component. When multiple framework variants offer the same
// HAL kind, scan order in M5HAL_v2.hpp decides the default flat injection.
#if defined(ESP_PLATFORM)
#define M5HAL_FRAMEWORK_HAS_ESPIDF 1
#else
#define M5HAL_FRAMEWORK_HAS_ESPIDF 0
#endif

// FreeRTOS detection: the freertos framework variant provides
// runtime::Mutex and runtime::Task. Currently gated to the ESP-IDF
// include layout (<freertos/FreeRTOS.h>) because the Task implementation
// uses Espressif extensions (xTaskCreatePinnedToCore). When a portable
// FreeRTOS backend is added, widen this to also match <FreeRTOS.h>.
#if __has_include(<freertos/FreeRTOS.h>)
#define M5HAL_FRAMEWORK_HAS_FREERTOS 1
#else
#define M5HAL_FRAMEWORK_HAS_FREERTOS 0
#endif

// POSIX host framework variant: active on a plain POSIX host (no Arduino /
// ESP-IDF SDK in the build). Offers UART (termios serial) and runtime
// (CLOCK_MONOTONIC time + std::timed_mutex). UART is opt-out, not opt-in:
// the host serial port is what a real application wants, so posix is the
// default UART provider there; set M5HAL_CONFIG_POSIX_UART=0 to leave UART
// unprovided (e.g. a host test). The opt-out is scoped to the UART kind —
// it must not deactivate the variant, or every Bus on the host would
// silently fall back to the stub fake mutex. stub keeps no-op'ing the
// HAL kinds that have no host implementation.
#ifndef M5HAL_CONFIG_POSIX_UART
#define M5HAL_CONFIG_POSIX_UART 1
#endif
#if !defined(ESP_PLATFORM) && !defined(ARDUINO) && __has_include(<termios.h>)
#define M5HAL_FRAMEWORK_HAS_POSIX 1
#else
#define M5HAL_FRAMEWORK_HAS_POSIX 0
#endif

// BSD socket API: available on POSIX hosts (macOS / Linux) and on
// ESP-IDF targets (lwIP exposes <sys/socket.h> through the VFS).
// The bsd framework variant backs the TCP transport for the remote bus.
#if __has_include(<sys/socket.h>) && __has_include(<netinet/tcp.h>)
#define M5HAL_FRAMEWORK_HAS_BSD_SOCKET 1
#else
#define M5HAL_FRAMEWORK_HAS_BSD_SOCKET 0
#endif

// Remote framework variant: proxy buses that forward operations to a peer
// MCU over RemoteSession (mux transport). Opt-in via build flag; when
// enabled, remote wins bus kinds not already claimed by a higher-priority
// variant. Scanned after posix and before software in M5HAL_v2.hpp.
#ifndef M5HAL_CONFIG_REMOTE
#define M5HAL_CONFIG_REMOTE 0
#endif
#if M5HAL_CONFIG_REMOTE
#define M5HAL_FRAMEWORK_HAS_REMOTE 1
#else
#define M5HAL_FRAMEWORK_HAS_REMOTE 0
#endif

#if __has_include(<SDL2/SDL.h>) || __has_include(<SDL.h>)
#define M5HAL_FRAMEWORK_HAS_SDL 1
#else
#define M5HAL_FRAMEWORK_HAS_SDL 0
#endif

#endif
