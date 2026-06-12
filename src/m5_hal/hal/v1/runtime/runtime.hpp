// SPDX-License-Identifier: MIT
#ifndef M5_HAL_RUNTIME_HPP_
#define M5_HAL_RUNTIME_HPP_

#include "../../../../m5_hal_config.hpp"  // M5HAL_INLINE_V1

// Lock in the inline-ness of `m5::hal::v1` (see M5HAL_v1.hpp): this
// header is the first to open the namespace when a kind header is
// included standalone.
namespace m5 {
namespace hal {
M5HAL_INLINE_V1 namespace v1
{
}
}  // namespace hal
}  // namespace m5

/*!
  @namespace m5::hal::v1::runtime
  @brief Environment facilities (time + mutex) injected from the
         selected variant. Authoritative contract: spec/design/runtime.md.

  The runtime kind has no abstract base. Unlike the bus kinds (whose
  winners bind through type aliases), runtime keeps the `using
  namespace` injection because its contract is free functions and one
  concrete class instead of Bus / Accessor:

      uint32_t millis(void);        // wall-clock ms since start, wraps
      uint32_t micros(void);        // wall-clock µs since start, wraps
      void     delayMs(uint32_t);   // at least ms; yields the task
      void     delayUs(uint32_t);   // busy-wait precision, short delays
      class    Mutex;               // bool lock(uint32_t timeout_ms);
                                    // void unlock();

  Mutex semantics: `lock` waits up to timeout_ms and returns
  whether the mutex was taken; 0 = immediate try-lock,
  `types::TIMEOUT_FOREVER` = block until acquired (the stub fake is
  the documented exception: with no second task to release the lock
  it fails immediately instead of hanging). Non-recursive — a re-lock
  from the holding task waits until the timeout and fails (F4); with
  TIMEOUT_FOREVER it deadlocks (fail-loud: the task watchdog fires).
  Task context only; never call from an ISR (F5). Timeout granularity
  follows the variant (one FreeRTOS tick — 10 ms by default — on the
  embedded targets).

  EARLY SCAN: unlike the bus kinds, runtime is resolved HERE rather
  than at the end of M5HAL_v1.hpp, because bus::IBus embeds
  runtime::Mutex by value and therefore needs the complete type. The
  passes below mirror the main scan's framework order (arduino ->
  espidf -> posix -> stub; software does not offer runtime) and ride
  the same dispatch block in offer_all.inl with the non-runtime kinds
  masked (_macro/offer_runtime_only.inl). Platform variants do not
  currently offer runtime; when one does, add its pass FIRST here so
  the documented platform-before-framework scan order keeps holding
  for this kind too.
 */

#include "../../../variants/ids.hpp"
#include "../../../variants/frameworks/_checker.hpp"

#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include "../../../variants/frameworks/arduino/hal/runtime/runtime.hpp"
#include "../../../variants/frameworks/arduino/_offer.hpp"
#include "../../../_macro/offer_runtime_only.inl"
#endif

#if M5HAL_FRAMEWORK_HAS_ESPIDF
#include "../../../variants/frameworks/espidf/hal/runtime/runtime.hpp"
#include "../../../variants/frameworks/espidf/_offer.hpp"
#include "../../../_macro/offer_runtime_only.inl"
#endif

#if M5HAL_FRAMEWORK_HAS_POSIX
#include "../../../variants/frameworks/posix/hal/runtime/runtime.hpp"
#include "../../../variants/frameworks/posix/_offer.hpp"
#include "../../../_macro/offer_runtime_only.inl"
#endif

#include "../../../variants/frameworks/stub/hal/runtime/runtime.hpp"
#include "../../../variants/frameworks/stub/_offer.hpp"
#include "../../../_macro/offer_runtime_only.inl"

// The stub fallback always offers runtime, so unlike the bus kinds
// the selected-variant marker can never stay NONE — bus::IBus depends
// on the type existing. Fail loudly if the invariant ever breaks.
#ifndef M5HAL_V1_SELECTED_VARIANT_RUNTIME
#error "runtime: no variant offered the runtime kind (the stub fallback must always offer it)"
#endif

#endif
