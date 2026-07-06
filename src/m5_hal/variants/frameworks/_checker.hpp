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
