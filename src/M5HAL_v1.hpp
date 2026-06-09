#ifndef M5_HAL_HPP
#define M5_HAL_HPP

#include "m5_hal_config.hpp"  // M5HAL_INLINE_V1
#include <M5Utility.hpp>

// Lock in the inline-ness of the `m5::hal::v1` namespace up front.
// An inline namespace's inline-ness is decided by its first
// declaration and propagates to every later re-opening through a
// nested specifier (`namespace m5::hal::v1::i2c { ... }`). Declaring
// it once here means v1 implementation files do not need to touch it.
namespace m5 {
namespace hal {
M5HAL_INLINE_V1 namespace v1
{
}
}  // namespace hal
}  // namespace m5

#include "./m5_hal/variants/platforms/_checker.hpp"
#include "./m5_hal/variants/frameworks/_checker.hpp"
#include "./m5_hal/hal/v1/i2c/i2c.hpp"
#include "./m5_hal/hal/v1/i2c/slave.hpp"
#include "./m5_hal/hal/v1/spi/spi.hpp"
#include "./m5_hal/hal/v1/uart/uart.hpp"

#include "./m5_hal/hal/v1/bus/bus.hpp"
#include "./m5_hal/hal/v1/gpio/gpio.hpp"
#include "./m5_hal/hal/v1/gpio/group.hpp"
#include "./m5_hal/hal/v1/service/service.hpp"
#include "./m5_hal/hal/v1/memory/allocator.hpp"
#include "./m5_hal/hal/v1/data.hpp"
#include "./m5_hal/hal/v1/data/memory.hpp"
#include "./m5_hal/hal/v1/data/limited.hpp"

#define M5HAL_STATIC_MACRO_STRING(x) #x
// clang-format off
#define M5HAL_STATIC_MACRO_CONCAT(x, y) M5HAL_STATIC_MACRO_STRING(x/y)
// clang-format on

#define M5HAL_STATIC_MACRO_PATH_HEADER M5HAL_STATIC_MACRO_CONCAT(M5HAL_TARGET_PLATFORM_PATH, hal.hpp)

#if M5HAL_TARGET_PLATFORM_NUMBER != 0
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

// software variant: bit-banged HAL implementations always available as a
// fallback when no platform/framework variant offers a hardware version
// of the same HAL kind. Always included.
#include "./m5_hal/variants/frameworks/software/hal.hpp"

// Public access to the active variant's HAL goes through the flat injection
// performed by _macro/offer_all.inl below (e.g. m5::hal::gpio::Pin).
// The legacy `using namespace frameworks::arduino;` injection into m5::hal
// is not used.

// === variant scan and namespace alias generation ===
//
// Scan order: platform -> arduino framework -> espidf framework -> posix
// framework -> software framework -> stub fallback.
// Each pass includes the variant's _offer.hpp followed by
// offer_all.inl, which generates namespace aliases under
// m5::hal::<hal>::variant::* for HALs the variant offers and
// undefs the M5HAL_VARIANT_CURRENT_*_ macros. The first variant
// to offer a given HAL is also flat-injected into m5::hal::<hal>.

#include "./m5_hal/variants/frameworks/stub/hal.hpp"

// 1. platform _offer.hpp scan
#define M5HAL_STATIC_MACRO_PATH_OFFER M5HAL_STATIC_MACRO_CONCAT(M5HAL_TARGET_PLATFORM_PATH, _offer.hpp)
#if M5HAL_TARGET_PLATFORM_NUMBER != 0
#define M5HAL_VARIANT_PLATFORM_ 1
#include M5HAL_STATIC_MACRO_PATH_OFFER
#include "./m5_hal/_macro/offer_all.inl"
#undef M5HAL_VARIANT_PLATFORM_
#endif
#undef M5HAL_STATIC_MACRO_PATH_OFFER

// 2. arduino framework _offer.hpp scan
#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include "./m5_hal/variants/frameworks/arduino/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"
#endif

// 3. espidf framework _offer.hpp scan
#if M5HAL_FRAMEWORK_HAS_ESPIDF
#include "./m5_hal/variants/frameworks/espidf/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"
#endif

// 4. posix host framework _offer.hpp scan (host serial; offers UART). Placed
//    before software/stub so it wins the host UART slot, which no other host
//    variant fills.
#if M5HAL_FRAMEWORK_HAS_POSIX
#include "./m5_hal/variants/frameworks/posix/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"
#endif

// 5. software framework _offer.hpp scan (always present, between hardware
//    variants and stub: provides bit-bang fallback for platforms / frameworks
//    that do not offer a hardware implementation of the HAL kind)
#include "./m5_hal/variants/frameworks/software/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"

// 6. stub fallback _offer.hpp scan (always last)
#include "./m5_hal/variants/frameworks/stub/_offer.hpp"
#include "./m5_hal/_macro/offer_all.inl"

// ----- M5HALCore (singleton HAL object layer) -----
//
// Declare the `M5HALCore` class in a context where every flat-injection
// scan has already happened. The placement is an exception to the
// 1:1 namespace/filename rule (see spec/architecture.md, namespace-rules
// section). Callers reach sub-objects via `m5::hal::v1::M5_Hal.Gpio.*`.
#include "./m5_hal/hal/v1/m5_hal.hpp"

#endif
