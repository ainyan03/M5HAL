// SPDX-License-Identifier: MIT
// clang-format off
//
// Re-includable. M5HAL_v1.hpp re-includes this after every _offer.hpp
// during variant scanning: consumes the M5HAL_VARIANT_CURRENT_*_ macros,
// emits namespace aliases for offered HALs (via offer_kind.inl), then
// undefs every consumed macro so the next pass starts clean.
//
// Inputs (from the just-included _offer.hpp):
//   M5HAL_VARIANT_CURRENT_ALIAS_   — variant short name used as alias namespace (e.g. arduino)
//   M5HAL_VARIANT_CURRENT_BASE_NS_ — variant base namespace path (e.g. frameworks::arduino)
//   M5HAL_VARIANT_CURRENT_ID_      — variant identity constant (variants/ids.hpp)
//   M5HAL_VARIANT_CURRENT_HAS_HAL_*_ — capability flag(s)
// Optional input (set by the M5HAL_v1.hpp scan loop, not the _offer.hpp):
//   M5HAL_VARIANT_PLATFORM_          — 1 only while scanning a platform variant
//
// Aliases generated per offered kind (emitted by offer_kind.inl):
//   m5::hal::v1::<kind>::variant::<ALIAS>   — always created
//   m5::hal::v1::<kind>::variant::platform  — only while scanning a platform variant, first hit only
//   m5::hal::v1::<kind>                     — flat injection of the first hit across all variants
//
// The first hit also burns the winner's identity into
// M5HAL_V1_SELECTED_VARIANT_<KIND> — a plain integer macro usable in
// both #if and static_assert (values: M5HAL_V1_VARIANT_ID_*,
// variants/ids.hpp; M5HAL_v1.hpp defaults unoffered kinds to NONE) —
// and that marker doubles as the kind's first-hit guard. #define does
// not expand its replacement list, so the value cannot be forwarded
// from the (later-undeffed) M5HAL_VARIANT_CURRENT_ID_; the explicit
// #elif chain per variant id is the only way to fix the value at
// first-hit time, and it cannot be shared across kinds because the
// marker NAME is kind-specific ([026]). When adding a VARIANT, extend
// every chain (the #else makes a miss a compile error); when adding a
// KIND, copy one dispatch block and adjust the kind tokens.
//
// Note: v0/v1 coexistence — target / source namespaces both pass through
// the explicit ::v1:: sub namespace. The shared bus base classes live in
// m5::hal::v1::bus::*. v0 (when offered via
// inline namespace v0) provides its own legacy structure and is not
// touched by this scan.

// ---------------------------------------------------------------------
// hal/gpio
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_) && M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_

#  define M5HAL_OFFER_KIND_NS_ gpio
#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_GPIO_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_GPIO_BOUND_ 1
#    define M5HAL_OFFER_KIND_EMIT_PLATFORM_ 1
#  endif
#  ifndef M5HAL_V1_SELECTED_VARIANT_GPIO
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V1_SELECTED_VARIANT_GPIO M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V1_SELECTED_VARIANT_GPIO M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V1_SELECTED_VARIANT_GPIO M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V1_SELECTED_VARIANT_GPIO M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V1_SELECTED_VARIANT_GPIO M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V1_SELECTED_VARIANT_GPIO M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#    else
#      error "offer_all.inl: M5HAL_VARIANT_CURRENT_ID_ missing from the gpio selected-marker chain"
#    endif
#  endif
#  include "./offer_kind.inl"

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_

// ---------------------------------------------------------------------
// hal/i2c
//
// Source and target namespaces are symmetric: each variant nests its
// HAL kinds directly under the variant base namespace (e.g.
// m5::variants::frameworks::software::hal::v1::i2c), and the flat injection
// target is m5::hal::v1::i2c. There is no intermediate "bus" hop on either
// side — the shared bus base classes live in m5::hal::v1::bus::* but i2c
// concrete types do not pass through that namespace.
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_) && M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_

#  define M5HAL_OFFER_KIND_NS_ i2c
#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_I2C_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_I2C_BOUND_ 1
#    define M5HAL_OFFER_KIND_EMIT_PLATFORM_ 1
#  endif
#  ifndef M5HAL_V1_SELECTED_VARIANT_I2C
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V1_SELECTED_VARIANT_I2C M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V1_SELECTED_VARIANT_I2C M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V1_SELECTED_VARIANT_I2C M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V1_SELECTED_VARIANT_I2C M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V1_SELECTED_VARIANT_I2C M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V1_SELECTED_VARIANT_I2C M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#    else
#      error "offer_all.inl: M5HAL_VARIANT_CURRENT_ID_ missing from the i2c selected-marker chain"
#    endif
#  endif
#  include "./offer_kind.inl"

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_

// ---------------------------------------------------------------------
// hal/spi
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_) && M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_

#  define M5HAL_OFFER_KIND_NS_ spi
#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_SPI_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_SPI_BOUND_ 1
#    define M5HAL_OFFER_KIND_EMIT_PLATFORM_ 1
#  endif
#  ifndef M5HAL_V1_SELECTED_VARIANT_SPI
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V1_SELECTED_VARIANT_SPI M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V1_SELECTED_VARIANT_SPI M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V1_SELECTED_VARIANT_SPI M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V1_SELECTED_VARIANT_SPI M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V1_SELECTED_VARIANT_SPI M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V1_SELECTED_VARIANT_SPI M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#    else
#      error "offer_all.inl: M5HAL_VARIANT_CURRENT_ID_ missing from the spi selected-marker chain"
#    endif
#  endif
#  include "./offer_kind.inl"

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_

// ---------------------------------------------------------------------
// hal/i2s
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_) && M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_

#  define M5HAL_OFFER_KIND_NS_ i2s
#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_I2S_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_I2S_BOUND_ 1
#    define M5HAL_OFFER_KIND_EMIT_PLATFORM_ 1
#  endif
#  ifndef M5HAL_V1_SELECTED_VARIANT_I2S
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V1_SELECTED_VARIANT_I2S M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V1_SELECTED_VARIANT_I2S M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V1_SELECTED_VARIANT_I2S M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V1_SELECTED_VARIANT_I2S M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V1_SELECTED_VARIANT_I2S M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V1_SELECTED_VARIANT_I2S M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#    else
#      error "offer_all.inl: M5HAL_VARIANT_CURRENT_ID_ missing from the i2s selected-marker chain"
#    endif
#  endif
#  include "./offer_kind.inl"

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_

// ---------------------------------------------------------------------
// hal/uart
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_UART_) && M5HAL_VARIANT_CURRENT_HAS_HAL_UART_

#  define M5HAL_OFFER_KIND_NS_ uart
#  if defined(M5HAL_VARIANT_PLATFORM_) && !defined(M5HAL_VARIANT_PLATFORM_UART_BOUND_)
#    define M5HAL_VARIANT_PLATFORM_UART_BOUND_ 1
#    define M5HAL_OFFER_KIND_EMIT_PLATFORM_ 1
#  endif
#  ifndef M5HAL_V1_SELECTED_VARIANT_UART
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V1_SELECTED_VARIANT_UART M5HAL_V1_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V1_SELECTED_VARIANT_UART M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V1_SELECTED_VARIANT_UART M5HAL_V1_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V1_SELECTED_VARIANT_UART M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V1_SELECTED_VARIANT_UART M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V1_SELECTED_VARIANT_UART M5HAL_V1_VARIANT_ID_FRAMEWORK_STUB
#    else
#      error "offer_all.inl: M5HAL_VARIANT_CURRENT_ID_ missing from the uart selected-marker chain"
#    endif
#  endif
#  include "./offer_kind.inl"

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_UART_

// ---------------------------------------------------------------------
// cleanup: undef every macro the just-completed _offer.hpp may have
// declared, so the next pass starts clean.
// ---------------------------------------------------------------------
#undef M5HAL_VARIANT_CURRENT_ALIAS_
#undef M5HAL_VARIANT_CURRENT_BASE_NS_
#undef M5HAL_VARIANT_CURRENT_ID_
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_UART_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_UART_
#endif
