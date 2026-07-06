// SPDX-License-Identifier: MIT
// clang-format off
//
// Re-includable. M5HAL_v2.hpp re-includes this after every _offer.hpp
// during variant scanning: consumes the M5HAL_VARIANT_CURRENT_*_ macros,
// emits the winner aliases for offered HALs (via offer_kind.inl), then
// undefs every consumed macro so the next pass starts clean.
//
// Inputs (from the just-included _offer.hpp):
//   M5HAL_VARIANT_CURRENT_ALIAS_   — variant short name (e.g. arduino), also
//                                    the `_<variant>` suffix of its types
//   M5HAL_VARIANT_CURRENT_BASE_NS_ — variant base namespace path (e.g.
//                                    frameworks::arduino; runtime kind only)
//   M5HAL_VARIANT_CURRENT_ID_      — variant identity constant (variants/ids.hpp)
//   M5HAL_VARIANT_CURRENT_HAS_HAL_*_ — capability flag(s)
//
// Winner binding (emitted by offer_kind.inl on the first hit per kind):
//   bus kinds:  using Bus = Bus_<variant>; using BusConfig = BusConfig_<variant>;
//   gpio:       using Port / GPIO aliases + getMCUGPIO() / getGPIO() wrappers
//   runtime:    using namespace (time free functions only)
//   runtime_mutex: using Mutex = Mutex_<variant>;
//   runtime_task:  using Task  = Task_<variant>;
// Variants define their concrete types directly in m5::hal::v2::<kind>
// with the `_<variant>` suffix, so non-winning variants stay addressable
// (e.g. i2c::Bus_software next to the winning i2c::Bus_arduino).
//
// The first hit also burns the winner's identity into
// M5HAL_V2_SELECTED_VARIANT_<KIND> — a plain integer macro usable in
// both #if and static_assert (values: M5HAL_V2_VARIANT_ID_*,
// variants/ids.hpp; M5HAL_v2.hpp defaults unoffered kinds to NONE) —
// and that marker doubles as the kind's first-hit guard. #define does
// not expand its replacement list, so the value cannot be forwarded
// from the (later-undeffed) M5HAL_VARIANT_CURRENT_ID_; the explicit
// #elif chain per variant id is the only way to fix the value at
// first-hit time, and it cannot be shared across kinds because the
// marker NAME is kind-specific ([026]). When adding a VARIANT, extend
// every chain (the #else makes a miss a compile error); when adding a
// KIND, copy one dispatch block and adjust the kind tokens.
//
// Note: v0/v2 coexistence — target / source namespaces both pass through
// the explicit ::v2:: sub namespace. The shared bus base classes live in
// m5::hal::v2::bus::*. v0 (when offered via
// inline namespace v0) provides its own legacy structure and is not
// touched by this scan.

// ---------------------------------------------------------------------
// hal/runtime (time functions only: millis / micros / delayMs / delayUs / yield)
//
// Split from the old monolithic runtime block: Mutex and Task are now
// independent sub-kinds (RUNTIME_MUTEX / RUNTIME_TASK below) so that
// FreeRTOS can win OS primitives while arduino/espidf win time.
// The RUNTIME block retains the `using namespace` injection for the
// time free functions. Burned by the EARLY scan in
// hal/v2/runtime/runtime.hpp (same as before).
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_) && M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_

#  define M5HAL_OFFER_KIND_NS_ runtime
#  ifndef M5HAL_V2_SELECTED_VARIANT_RUNTIME
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    define M5HAL_OFFER_KIND_RUNTIME_ 1
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#    else
#      error "offer_all.inl: M5HAL_VARIANT_CURRENT_ID_ missing from the runtime selected-marker chain"
#    endif
#  endif
#  include "./offer_kind.inl"

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_

// ---------------------------------------------------------------------
// runtime::Mutex (sub-kind)
//
// Type alias injection: the first variant to offer RUNTIME_MUTEX wins
// `using Mutex = Mutex_<variant>;` in m5::hal::v2::runtime. FreeRTOS
// wins on embedded; posix/stub provide host/test fallbacks. Handled
// inline (not through offer_kind.inl) because the emit pattern differs
// from bus kinds and the monolithic runtime injection.
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_MUTEX_) && M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_MUTEX_

#  ifndef M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#    else
#      error "offer_all.inl: M5HAL_VARIANT_CURRENT_ID_ missing from the runtime_mutex selected-marker chain"
#    endif
namespace m5 { namespace hal { namespace v2 { namespace runtime {
using Mutex = ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v2::runtime::Mutex;
} } } }
#  endif

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_MUTEX_

// ---------------------------------------------------------------------
// runtime::Task (sub-kind)
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_TASK_) && M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_TASK_

#  ifndef M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#    else
#      error "offer_all.inl: M5HAL_VARIANT_CURRENT_ID_ missing from the runtime_task selected-marker chain"
#    endif
namespace m5 { namespace hal { namespace v2 { namespace runtime {
using Task = ::m5::M5HAL_VARIANT_CURRENT_BASE_NS_::hal::v2::runtime::Task;
} } } }
#  endif

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_TASK_

// ---------------------------------------------------------------------
// hal/gpio
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_) && M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_

#  define M5HAL_OFFER_KIND_NS_ gpio
#  ifndef M5HAL_V2_SELECTED_VARIANT_GPIO
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    define M5HAL_OFFER_KIND_GPIO_ 1
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V2_SELECTED_VARIANT_GPIO M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V2_SELECTED_VARIANT_GPIO M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V2_SELECTED_VARIANT_GPIO M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#      define M5HAL_V2_SELECTED_VARIANT_GPIO M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V2_SELECTED_VARIANT_GPIO M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V2_SELECTED_VARIANT_GPIO M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#      define M5HAL_V2_SELECTED_VARIANT_GPIO M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V2_SELECTED_VARIANT_GPIO M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#    else
#      error "offer_all.inl: M5HAL_VARIANT_CURRENT_ID_ missing from the gpio selected-marker chain"
#    endif
#  endif
#  include "./offer_kind.inl"

#endif  // M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_

// ---------------------------------------------------------------------
// hal/i2c
//
// Variants define their concrete types directly in m5::hal::v2::i2c
// (`Bus_<variant>` / `BusConfig_<variant>`); the winner additionally
// owns the unsuffixed `i2c::Bus` / `i2c::BusConfig` aliases. The shared
// bus base classes live in m5::hal::v2::bus::*; the kind bases are
// i2c::IBus / i2c::IBusConfig.
// ---------------------------------------------------------------------
#if defined(M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_) && M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_

#  define M5HAL_OFFER_KIND_NS_ i2c
#  ifndef M5HAL_V2_SELECTED_VARIANT_I2C
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    define M5HAL_OFFER_KIND_FACADE_ 1  // i2c::Bus is a runtime facade (ADR 034); emit only BusConfig alias
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V2_SELECTED_VARIANT_I2C M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V2_SELECTED_VARIANT_I2C M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V2_SELECTED_VARIANT_I2C M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#      define M5HAL_V2_SELECTED_VARIANT_I2C M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V2_SELECTED_VARIANT_I2C M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V2_SELECTED_VARIANT_I2C M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#      define M5HAL_V2_SELECTED_VARIANT_I2C M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V2_SELECTED_VARIANT_I2C M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
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
#  ifndef M5HAL_V2_SELECTED_VARIANT_SPI
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    define M5HAL_OFFER_KIND_FACADE_ 1  // spi::Bus is a runtime facade (ADR 034); emit only BusConfig alias
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V2_SELECTED_VARIANT_SPI M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V2_SELECTED_VARIANT_SPI M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V2_SELECTED_VARIANT_SPI M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#      define M5HAL_V2_SELECTED_VARIANT_SPI M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V2_SELECTED_VARIANT_SPI M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V2_SELECTED_VARIANT_SPI M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#      define M5HAL_V2_SELECTED_VARIANT_SPI M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V2_SELECTED_VARIANT_SPI M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
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
#  ifndef M5HAL_V2_SELECTED_VARIANT_I2S
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    define M5HAL_OFFER_KIND_FACADE_ 1  // i2s::Bus is a runtime facade (ADR 034); emit only BusConfig alias
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V2_SELECTED_VARIANT_I2S M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V2_SELECTED_VARIANT_I2S M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V2_SELECTED_VARIANT_I2S M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#      define M5HAL_V2_SELECTED_VARIANT_I2S M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V2_SELECTED_VARIANT_I2S M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V2_SELECTED_VARIANT_I2S M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#      define M5HAL_V2_SELECTED_VARIANT_I2S M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V2_SELECTED_VARIANT_I2S M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
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
#  ifndef M5HAL_V2_SELECTED_VARIANT_UART
#    define M5HAL_OFFER_KIND_EMIT_FLAT_ 1
#    define M5HAL_OFFER_KIND_FACADE_ 1  // uart::Bus is a runtime facade (ADR 034); emit only BusConfig alias
#    if M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#      define M5HAL_V2_SELECTED_VARIANT_UART M5HAL_V2_VARIANT_ID_PLATFORM_ESP32
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#      define M5HAL_V2_SELECTED_VARIANT_UART M5HAL_V2_VARIANT_ID_FRAMEWORK_ARDUINO
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#      define M5HAL_V2_SELECTED_VARIANT_UART M5HAL_V2_VARIANT_ID_FRAMEWORK_ESPIDF
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#      define M5HAL_V2_SELECTED_VARIANT_UART M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#      define M5HAL_V2_SELECTED_VARIANT_UART M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#      define M5HAL_V2_SELECTED_VARIANT_UART M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#      define M5HAL_V2_SELECTED_VARIANT_UART M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE
#    elif M5HAL_VARIANT_CURRENT_ID_ == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
#      define M5HAL_V2_SELECTED_VARIANT_UART M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
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
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_MUTEX_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_MUTEX_
#endif
#ifdef M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_TASK_
#  undef M5HAL_VARIANT_CURRENT_HAS_HAL_RUNTIME_TASK_
#endif
