// SPDX-License-Identifier: MIT
#ifndef M5_HAL_RUNTIME_HPP_
#define M5_HAL_RUNTIME_HPP_

#include "../../../../m5_hal_config.hpp"  // M5HAL_INLINE_V2

// Lock in the inline-ness of `m5::hal::v2` (see M5HAL_v2.hpp): this
// header is the first to open the namespace when a kind header is
// included standalone.
namespace m5 {
namespace hal {
M5HAL_INLINE_V2 namespace v2
{
}
}  // namespace hal
}  // namespace m5

/*!
  @namespace m5::hal::v2::runtime
  @brief Environment facilities (time + mutex + task + event) injected
         from the selected variant. Authoritative contract:
         spec/design/runtime.md.

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
      class    Task;                // bool start(entry_fn_t, void*, ...);
                                    // void join(); bool joinable() const;
      class    Event;               // bool wait(uint32_t timeout_ms);
                                    // void notify();
      void*    currentTaskId(void); // opaque per-task identity, ==-only

  Event semantics: a latching binary event for task-context
  wait/notify pairs (single waiter, at most one notify latched;
  repeated notifies may merge). `wait` blocks up to timeout_ms for a
  notify and consumes it (true); 0 = non-blocking check,
  `types::TIMEOUT_FOREVER` = block until notified (the stub fake
  fails immediately instead, same documented exception as Mutex).
  A notify that arrives BEFORE the wait is not lost — the next wait
  consumes it immediately. Writes made before notify() are visible
  after the wait() that consumes it (release/acquire pairing). Task
  context only; destroy only when no waiter exists and no concurrent
  notify can occur. Full contract: spec/design/runtime.md.

  Mutex semantics: `lock` waits up to timeout_ms and returns
  whether the mutex was taken; 0 = immediate try-lock,
  `types::TIMEOUT_FOREVER` = block until acquired (the stub fake is
  the documented exception: with no second task to release the lock
  it fails immediately instead of hanging). Non-recursive — a re-lock
  from the holding task waits until the timeout and fails; with
  TIMEOUT_FOREVER it deadlocks (fail-loud: the task watchdog fires).
  Task context only; never call from an ISR. Timeout granularity
  follows the variant (one FreeRTOS tick — 10 ms by default — on the
  embedded targets).

  Task semantics: `start` creates one task/thread for a plain
  `void (*)(void*)` entry point. The task stays joinable until `join`
  observes the entry point return; the destructor joins if needed.
  Cancellation and stop flags belong to the user's argument object,
  not to runtime::Task.

  currentTaskId semantics: returns an opaque `void*` identifying the
  calling task/thread. The only defined operation is same-value
  comparison (never dereference it); it never returns null in a running
  task context. It follows the RUNTIME_TASK winner so the id matches the
  task runtime::Task creates (FreeRTOS handle / posix thread_local
  address). The stub returns a single fixed id, consistent with its
  single-task fake — code that needs to distinguish tasks (e.g. the
  ServiceRunner self-call check) is a no-op there, which is correct
  because the stub never runs a second task.

  EARLY SCAN: unlike the bus kinds, runtime is resolved HERE rather
  than at the end of M5HAL_v2.hpp, because bus::IBus embeds
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

// FreeRTOS framework variant: provides Mutex and Task. Scanned FIRST
// so it wins RUNTIME_MUTEX / RUNTIME_TASK before arduino/espidf
// (which win RUNTIME = time functions).
#if M5HAL_FRAMEWORK_HAS_FREERTOS
#include "../../../variants/frameworks/freertos/hal/runtime/mutex.hpp"
#include "../../../variants/frameworks/freertos/hal/runtime/task.hpp"
#include "../../../variants/frameworks/freertos/hal/runtime/event.hpp"
#include "../../../variants/frameworks/freertos/_offer.hpp"
#include "../../../_macro/offer_runtime_only.inl"
#endif

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
#ifndef M5HAL_DETAIL_VARIANT_SELECTED_RUNTIME_
#error "runtime: no variant offered the runtime kind (the stub fallback must always offer it)"
#endif
#ifndef M5HAL_DETAIL_VARIANT_SELECTED_RUNTIME_MUTEX_
#error "runtime: no variant offered runtime::Mutex (the stub fallback must always offer it)"
#endif
#ifndef M5HAL_DETAIL_VARIANT_SELECTED_RUNTIME_TASK_
#error "runtime: no variant offered runtime::Task (the stub fallback must always offer it)"
#endif
#ifndef M5HAL_DETAIL_VARIANT_SELECTED_RUNTIME_EVENT_
#error "runtime: no variant offered runtime::Event (the stub fallback must always offer it)"
#endif

// currentTaskId(): a free function that must follow the RUNTIME_TASK
// winner (the same variant that provides runtime::Task), so the id it
// returns matches the task runtime::Task creates. It is a single name
// (not a namespace of functions), so it rides a using-declaration here
// keyed on the RUNTIME_TASK marker rather than the RUNTIME `using
// namespace` block (which is won by arduino/espidf for the time
// functions, neither of which can name a FreeRTOS task). Only
// freertos/posix/stub offer RUNTIME_TASK, so those three are the only
// reachable cases; the #else guards against a future task backend
// forgetting to provide currentTaskId.
namespace m5 {
namespace hal {
M5HAL_INLINE_V2 namespace v2
{
    namespace runtime {
#if M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK == M5HAL_V2_VARIANT_ID_FRAMEWORK_FREERTOS
    using ::m5::variants::frameworks::freertos::hal::v2::runtime::currentTaskId;
#elif M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX
    using ::m5::variants::frameworks::posix::hal::v2::runtime::currentTaskId;
#elif M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB
    using ::m5::variants::frameworks::stub::hal::v2::runtime::currentTaskId;
#else
#error "runtime: currentTaskId has no mapping for the selected RUNTIME_TASK variant"
#endif
    }  // namespace runtime
}
}  // namespace hal
}  // namespace m5

#endif
