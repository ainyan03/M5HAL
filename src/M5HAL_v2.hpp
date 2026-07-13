// SPDX-License-Identifier: MIT
//
// M5HAL v2 entry header. Include this ONE header to opt into the v2 API;
// it wires up variant selection and pulls in every kind. This file is
// mostly variant plumbing (macros + winner-alias generation) and does
// NOT define the user-facing API itself. The documented call surface —
// Bus / Accessor / config types and their usage — lives in the per-kind
// namespace Doxygen at `m5_hal/hal/v2/<kind>/<kind>.hpp` (e.g.
// `m5_hal/hal/v2/i2c/i2c.hpp`). Read those for signatures; read the
// top-level README for a minimal runnable example.
#ifndef M5_HAL_V2_HPP
#define M5_HAL_V2_HPP

// The v2 API is written in C++17 (tag-typed pins, `constexpr` config,
// `if constexpr` variant selection). Fail loudly and early here instead
// of drowning the user in ~80 unexplained `constexpr` errors from deep
// inside the headers. A PlatformIO consumer on espressif32@6.x (Arduino
// core 2.x) defaults to gnu++11 and must opt in with `-std=gnu++17`;
// arduino-esp32 3.x and the Arduino IDE already default to C++17.
//
// The whole body is wrapped in the `#else` branch on purpose: GCC's
// `#error` is only a diagnostic and does NOT stop the compile, so without
// the wrap the user would still get this line PLUS the ~80 downstream
// C++17-syntax errors. Skipping the body keeps it to exactly one message.
#if defined(__cplusplus) && __cplusplus < 201703L

#error \
    "M5HAL v2 requires C++17. Add -std=gnu++17 to your build flags (PlatformIO: build_flags = -std=gnu++17 / build_unflags = -std=gnu++11)."

#else  // __cplusplus >= 201703L

#include "m5_hal_config.hpp"  // M5HAL_INLINE_V2
#include <M5Utility.hpp>

// Lock in the inline-ness of the `m5::hal::v2` namespace up front.
// An inline namespace's inline-ness is decided by its first
// declaration and propagates to every later re-opening through a
// nested specifier (`namespace m5::hal::v2::i2c { ... }`). Declaring
// it once here means v2 implementation files do not need to touch it.
namespace m5 {
namespace hal {
M5HAL_INLINE_V2 namespace v2
{
}
}  // namespace hal
}  // namespace m5

#include "./m5_hal/variants/ids.hpp"
#include "./m5_hal/variants/platforms/_checker.hpp"
#include "./m5_hal/variants/frameworks/_checker.hpp"

#if defined(M5HAL_V2_SELECTED_VARIANT_RUNTIME) && !defined(M5HAL_DETAIL_VARIANT_SELECTED_RUNTIME_)
#error "M5HAL_V2_SELECTED_VARIANT_RUNTIME is a read-only output and must not be predefined"
#endif
#if defined(M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX) && !defined(M5HAL_DETAIL_VARIANT_SELECTED_RUNTIME_MUTEX_)
#error "M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX is a read-only output and must not be predefined"
#endif
#if defined(M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK) && !defined(M5HAL_DETAIL_VARIANT_SELECTED_RUNTIME_TASK_)
#error "M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK is a read-only output and must not be predefined"
#endif
#if defined(M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT) && !defined(M5HAL_DETAIL_VARIANT_SELECTED_RUNTIME_EVENT_)
#error "M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT is a read-only output and must not be predefined"
#endif
#if defined(M5HAL_V2_SELECTED_VARIANT_GPIO) && !defined(M5HAL_DETAIL_VARIANT_SELECTED_GPIO_)
#error "M5HAL_V2_SELECTED_VARIANT_GPIO is a read-only output and must not be predefined"
#endif
#if defined(M5HAL_V2_SELECTED_VARIANT_I2C) && !defined(M5HAL_DETAIL_VARIANT_SELECTED_I2C_)
#error "M5HAL_V2_SELECTED_VARIANT_I2C is a read-only output and must not be predefined"
#endif
#if defined(M5HAL_V2_SELECTED_VARIANT_SPI) && !defined(M5HAL_DETAIL_VARIANT_SELECTED_SPI_)
#error "M5HAL_V2_SELECTED_VARIANT_SPI is a read-only output and must not be predefined"
#endif
#if defined(M5HAL_V2_SELECTED_VARIANT_I2S) && !defined(M5HAL_DETAIL_VARIANT_SELECTED_I2S_)
#error "M5HAL_V2_SELECTED_VARIANT_I2S is a read-only output and must not be predefined"
#endif
#if defined(M5HAL_V2_SELECTED_VARIANT_UART) && !defined(M5HAL_DETAIL_VARIANT_SELECTED_UART_)
#error "M5HAL_V2_SELECTED_VARIANT_UART is a read-only output and must not be predefined"
#endif

// runtime kind (time + mutex): resolved EARLY, before any bus header —
// bus::IBus embeds runtime::Mutex by value, so the winning variant must
// be known here. The header below runs its own runtime-only scan pass
// (same dispatch block in offer_all.inl, other kinds masked); the main
// scan further down only re-emits the runtime variant aliases.
#include "./m5_hal/hal/v2/runtime/runtime.hpp"

#include "./m5_hal/hal/v2/i2c/i2c.hpp"
#include "./m5_hal/hal/v2/i2c/slave.hpp"
#include "./m5_hal/hal/v2/spi/spi.hpp"
#include "./m5_hal/hal/v2/spi/slave.hpp"
#include "./m5_hal/hal/v2/uart/uart.hpp"
#include "./m5_hal/hal/v2/i2s/i2s.hpp"

#include "./m5_hal/hal/v2/bus/bus.hpp"
#include "./m5_hal/hal/v2/gpio/gpio.hpp"
#include "./m5_hal/hal/v2/gpio/group.hpp"
#include "./m5_hal/hal/v2/service/service.hpp"
#include "./m5_hal/hal/v2/memory/allocator.hpp"
#include "./m5_hal/hal/v2/data.hpp"
#include "./m5_hal/hal/v2/data/memory.hpp"
#include "./m5_hal/hal/v2/data/limited.hpp"
#include "./m5_hal/hal/v2/data/stream.hpp"
#include "./m5_hal/hal/v2/data/tap.hpp"
#include "./m5_hal/hal/v2/data/ring.hpp"
#include "./m5_hal/hal/v2/data/block.hpp"
#include "./m5_hal/hal/v2/data/mux.hpp"
#include "./m5_hal/hal/v2/frame/frame.hpp"
#include "./m5_hal/hal/v2/bytecode/bytecode.hpp"
#include "./m5_hal/hal/v2/remote/remote.hpp"
#include "./m5_hal/hal/v2/remote/server.hpp"
#include "./m5_hal/variants/frameworks/remote/session.hpp"
#include "./m5_hal/variants/frameworks/remote/backend.hpp"
#include "./m5_hal/hal/v2/remote/server_adapter.hpp"
#if M5HAL_FRAMEWORK_HAS_BSD_SOCKET
#include "./m5_hal/variants/frameworks/bsd/hal/remote/tcp_server.hpp"
#endif

#define M5HAL_STATIC_MACRO_STRING(x) #x
// clang-format off
#define M5HAL_STATIC_MACRO_CONCAT(x, y) M5HAL_STATIC_MACRO_STRING(x/y)
// clang-format on

#define M5HAL_STATIC_MACRO_PATH_HEADER M5HAL_STATIC_MACRO_CONCAT(M5HAL_V2_DETECTED_PLATFORM_VARIANT_PATH, hal.hpp)

#if M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID != M5HAL_V2_VARIANT_ID_NONE
#include M5HAL_STATIC_MACRO_PATH_HEADER
#endif

#undef M5HAL_STATIC_MACRO_PATH_HEADER

// Pull in the Arduino framework variant when available.
#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include "./m5_hal/variants/frameworks/arduino/hal.hpp"
#endif

// ESP-IDF framework variant: active whenever ESP_PLATFORM exposes the
// ESP-IDF API surface. Arduino-on-IDF may therefore have both arduino and
// espidf variants; scan order decides the default flat injection.
#if M5HAL_FRAMEWORK_HAS_ESPIDF
#include "./m5_hal/variants/frameworks/espidf/hal.hpp"
#endif

// POSIX host framework variant: real serial port via termios. Active on a
// plain POSIX host build (see frameworks/_checker.hpp). Offers UART only.
#if M5HAL_FRAMEWORK_HAS_POSIX
#include "./m5_hal/variants/frameworks/posix/hal.hpp"
#endif

// Remote framework variant: proxy buses (I2C/SPI/UART/I2S) that forward to a
// peer MCU over RemoteSession. Opt-in via M5HAL_CONFIG_REMOTE_VARIANT. The
// umbrella defines the Bus_remote / BusConfig_remote types the offer scan below
// binds as winner aliases, so it must precede that scan.
#if M5HAL_FRAMEWORK_HAS_REMOTE
#include "./m5_hal/variants/frameworks/remote/hal.hpp"
#endif

// software variant: bit-banged HAL implementations always available as a
// fallback when no platform/framework variant offers a hardware version
// of the same HAL kind. Always included.
#include "./m5_hal/variants/frameworks/software/hal.hpp"

// Public access to the active variant's HAL goes through the winner
// aliases emitted by _macro/offer_all.inl below (e.g. m5::hal::i2c::Bus
// = i2c::Bus_<variant>). The legacy `using namespace` flat injection
// is retained only by the runtime kind (free functions + Mutex).

// === variant scan and winner alias generation ===
//
// Scan order: platform -> freertos framework -> arduino framework ->
// espidf framework -> posix framework -> remote framework -> software
// framework -> stub fallback.
// Each pass includes the variant's _offer.hpp followed by
// offer_all.inl, which on the first hit per kind binds the winner's
// suffixed types (`Bus_<variant>` etc., defined directly in
// m5::hal::v2::<kind>) to the unsuffixed names (`using Bus =
// Bus_<variant>;`) and undefs the M5HAL_VARIANT_CURRENT_*_ macros.
// Non-winning variants stay addressable by their suffixed names.

#include "./m5_hal/variants/frameworks/stub/hal.hpp"

// 1. platform _offer.hpp scan
#define M5HAL_STATIC_MACRO_PATH_OFFER M5HAL_STATIC_MACRO_CONCAT(M5HAL_V2_DETECTED_PLATFORM_VARIANT_PATH, _offer.hpp)
#if M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID != M5HAL_V2_VARIANT_ID_NONE
#include M5HAL_STATIC_MACRO_PATH_OFFER
#include "./m5_hal/_macro/offer_all.inl"
#endif
#undef M5HAL_STATIC_MACRO_PATH_OFFER

// 2. freertos framework _offer.hpp scan (OS primitives: Mutex, Task).
//    Scanned before arduino/espidf so it wins RUNTIME_MUTEX/RUNTIME_TASK.
#if M5HAL_FRAMEWORK_HAS_FREERTOS
#include "./m5_hal/variants/frameworks/freertos/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"
#endif

// 3. arduino framework _offer.hpp scan
#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include "./m5_hal/variants/frameworks/arduino/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"
#endif

// 4. espidf framework _offer.hpp scan
#if M5HAL_FRAMEWORK_HAS_ESPIDF
#include "./m5_hal/variants/frameworks/espidf/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"
#endif

// 5. posix host framework _offer.hpp scan (host serial; offers UART). Placed
//    before software/stub so it wins the host UART slot, which no other host
//    variant fills.
#if M5HAL_FRAMEWORK_HAS_POSIX
#include "./m5_hal/variants/frameworks/posix/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"
#endif

// 6. remote framework _offer.hpp scan (proxy buses that forward operations to
//    a peer MCU via RemoteSession). Opt-in via
//    M5HAL_CONFIG_REMOTE_VARIANT=1 build flag.
//    Scanned after posix so posix wins UART (the host serial transport), while
//    remote wins I2C / SPI / I2S / UART (the tunnelled bus kinds) on host builds
//    where no higher-priority variant offers them.
#if M5HAL_FRAMEWORK_HAS_REMOTE
#include "./m5_hal/variants/frameworks/remote/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"
#endif

// 7. software framework _offer.hpp scan (always present, between hardware
//    variants and stub: provides bit-bang fallback for platforms / frameworks
//    that do not offer a hardware implementation of the HAL kind)
#include "./m5_hal/variants/frameworks/software/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"

// 8. stub fallback _offer.hpp scan (always last)
#include "./m5_hal/variants/frameworks/stub/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"

// Selected-variant markers: the scan passes above burned the winner's id
// into M5HAL_V2_SELECTED_VARIANT_<KIND> for every offered kind. Default
// the unoffered kinds to NONE so `#if M5HAL_V2_SELECTED_VARIANT_<KIND>
// == ...` comparisons are always well-formed. RUNTIME has no NONE
// default on purpose: bus::IBus depends on the type existing, so the
// early scan in hal/v2/runtime/runtime.hpp #errors instead when no
// variant offers it (the stub fallback always does).
#ifndef M5HAL_DETAIL_VARIANT_SELECTED_GPIO_
#define M5HAL_DETAIL_VARIANT_SELECTED_GPIO_ 1
#define M5HAL_V2_SELECTED_VARIANT_GPIO      M5HAL_V2_VARIANT_ID_NONE
#endif
#ifndef M5HAL_DETAIL_VARIANT_SELECTED_I2C_
#define M5HAL_DETAIL_VARIANT_SELECTED_I2C_ 1
#define M5HAL_V2_SELECTED_VARIANT_I2C      M5HAL_V2_VARIANT_ID_NONE
#endif
#ifndef M5HAL_DETAIL_VARIANT_SELECTED_SPI_
#define M5HAL_DETAIL_VARIANT_SELECTED_SPI_ 1
#define M5HAL_V2_SELECTED_VARIANT_SPI      M5HAL_V2_VARIANT_ID_NONE
#endif
#ifndef M5HAL_DETAIL_VARIANT_SELECTED_I2S_
#define M5HAL_DETAIL_VARIANT_SELECTED_I2S_ 1
#define M5HAL_V2_SELECTED_VARIANT_I2S      M5HAL_V2_VARIANT_ID_NONE
#endif
#ifndef M5HAL_DETAIL_VARIANT_SELECTED_UART_
#define M5HAL_DETAIL_VARIANT_SELECTED_UART_ 1
#define M5HAL_V2_SELECTED_VARIANT_UART      M5HAL_V2_VARIANT_ID_NONE
#endif

// ----- M5HAL_V2_TARGET_IS_PC convenience macro -----
//
// True when the build targets a POSIX host (PC), not an MCU.
// User code can branch on `#if M5HAL_V2_TARGET_IS_PC` instead of
// `#if !defined(ESP_PLATFORM)` for dual-target sources.
#ifdef M5HAL_V2_TARGET_IS_PC
#error "M5HAL_V2_TARGET_IS_PC is a read-only output and must not be predefined"
#endif
#define M5HAL_V2_TARGET_IS_PC M5HAL_FRAMEWORK_HAS_POSIX

// ----- M5HALCore (singleton HAL object layer) -----
//
// Declare the `M5HALCore` class in a context where every flat-injection
// scan has already happened. The placement is an exception to the
// 1:1 namespace/filename rule (see spec/architecture.md, namespace-rules
// section). Callers reach sub-objects via `m5::hal::v2::M5_Hal.Gpio.*`.
#include "./m5_hal/hal/v2/m5_hal.hpp"

#endif  // __cplusplus >= 201703L

#endif  // M5_HAL_V2_HPP
