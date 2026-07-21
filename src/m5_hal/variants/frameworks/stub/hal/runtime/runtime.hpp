// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_STUB_HAL_RUNTIME_RUNTIME_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_STUB_HAL_RUNTIME_RUNTIME_HPP

// runtime kind for the `stub` variant — a deterministic fake for
// native tests and the unconditional fallback. The clock starts at
// zero and advances ONLY through delayMs / delayUs; the mutex is a
// single-task owner guard.
// Authoritative contract: spec/design/runtime.md.

#include <cstddef>
#include <cstdint>

#include "../../../../../hal/v2/error.hpp"

// This header is always compiled (the unconditional runtime fallback — see
// hal/v2/runtime/runtime.hpp), including on bare-metal Arduino cores with no
// RTOS (RP2040 / SAMD51: see _checker.hpp's variant allowlist), where
// <thread> exists as a header but declares no std::this_thread /
// std::thread (same condition M5Utility's compatibility_feature.cpp
// gates on). Only pull in <thread> where it actually works.
#if !defined(ARDUINO) || defined(ESP_PLATFORM)
#define M5HAL_STUB_RUNTIME_HAS_STD_THREAD_ 1
#include <new>
#include <system_error>
#include <thread>
#include "../../../detail/thread_create_error.hpp"
#else
#define M5HAL_STUB_RUNTIME_HAS_STD_THREAD_ 0
// yield() below calls the portable Arduino ::yield() free function; this
// header must not rely on some earlier file in the aggregate TU having
// already included Arduino.h (self-contained per spec/design/variants.md).
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif
#endif

namespace m5::variants::frameworks::stub::hal::v2::runtime {

// The fake clock's backing store (mutable state forbids constexpr, so
// a function-local static places it in RAM — the stub gpio pattern).
inline uint32_t& fakeMicrosRef(void)
{
    static uint32_t s_micros = 0;
    return s_micros;
}

inline uint32_t millis(void)
{
    return fakeMicrosRef() / 1000u;
}
inline uint32_t micros(void)
{
    return fakeMicrosRef();
}
inline void delayMs(uint32_t ms)
{
    fakeMicrosRef() += ms * 1000u;  // wraps like the real clock
}
inline void delayUs(uint32_t us)
{
    fakeMicrosRef() += us;
}
inline void yield(void)
{
#if M5HAL_STUB_RUNTIME_HAS_STD_THREAD_
    std::this_thread::yield();
#else
    ::yield();  // Arduino API: cooperative yield to background tasks
#endif
}
/*! @brief Rewind the fake clock to zero (test isolation hook). */
inline void fakeReset(void)
{
    fakeMicrosRef() = 0;
}

/*!
  @brief Opaque task identity for the single-task fake: a fixed non-null
  address.

  The stub models one logical task, so every caller shares this id
  (consistent with the single-owner Mutex). Meaningful for same-value
  comparison only; never dereferenced.
 */
inline void* currentTaskId(void)
{
    static char id;
    return &id;
}

/*!
  @brief Single-task owner guard satisfying the runtime::Mutex contract.

  With no other task around to release the lock, waiting can never
  succeed — contention fails immediately whatever the timeout
  (including types::TIMEOUT_FOREVER, where real backends would block;
  returning false keeps native tests deterministic instead of hanging).
 */
class Mutex {
public:
    Mutex(void)                    = default;
    Mutex(const Mutex&)            = delete;
    Mutex& operator=(const Mutex&) = delete;

    ::m5::hal::v2::result_t<void> lock(uint32_t timeout_ms)
    {
        (void)timeout_ms;
        if (_locked) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
        }
        _locked = true;
        return {};
    }
    ::m5::hal::v2::result_t<void> unlock(void)
    {
        if (!_locked) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_STATE);
        }
        _locked = false;
        return {};
    }

private:
    bool _locked = false;
};

/*!
  @brief Single-task fake satisfying the runtime::Event contract.

  With no other task around to notify, blocking can never end — like
  the stub Mutex, wait() never blocks: a pending notify is consumed with
  success, otherwise it returns TIMEOUT_ERROR immediately whatever the timeout
  (including types::TIMEOUT_FOREVER — the documented stub exception).
  The latch itself is real, so notify-then-wait works single-task.
 */
class Event {
public:
    Event(void)                    = default;
    Event(const Event&)            = delete;
    Event& operator=(const Event&) = delete;

    ::m5::hal::v2::result_t<void> wait(uint32_t timeout_ms)
    {
        (void)timeout_ms;
        if (!_signaled) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
        }
        _signaled = false;
        return {};
    }
    void notify(void)
    {
        _signaled = true;
    }

private:
    bool _signaled = false;
};

/*!
  @brief Task implementation for targets with no usable thread creator.

  This concrete helper is available to provider tests even when the selected
  host stub uses std::thread. Bare Arduino aliases Task to it below.
 */
class ThreadlessTask {
public:
    using entry_fn_t = void (*)(void*);

    ThreadlessTask(void)                             = default;
    ThreadlessTask(const ThreadlessTask&)            = delete;
    ThreadlessTask& operator=(const ThreadlessTask&) = delete;

    ::m5::hal::v2::result_t<void> start(entry_fn_t fn, void* arg, const char* name = nullptr, size_t stack_size = 4096,
                                        int priority = 1,
                                        int core     = -1 /* types::TASK_CORE_ANY; placement ignored by this backend */)
    {
        (void)arg;
        (void)name;
        (void)stack_size;
        (void)priority;
        (void)core;
        if (fn == nullptr) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
        }
        return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::UNSUPPORTED);
    }

    void join(void)
    {
    }

    bool joinable(void) const
    {
        return false;
    }
};

#if M5HAL_STUB_RUNTIME_HAS_STD_THREAD_
/*!
  @brief std::thread-backed task satisfying the runtime::Task contract.

  `name`, `stack_size`, `priority`, and `core` are accepted for API
  parity with FreeRTOS targets and ignored by this backend.
 */
class Task {
public:
    using entry_fn_t = void (*)(void*);

    Task(void) = default;
    ~Task(void)
    {
        join();
    }
    Task(const Task&)            = delete;
    Task& operator=(const Task&) = delete;

    ::m5::hal::v2::result_t<void> start(entry_fn_t fn, void* arg, const char* name = nullptr, size_t stack_size = 4096,
                                        int priority = 1,
                                        int core     = -1 /* types::TASK_CORE_ANY; placement ignored by this backend */)
    {
        (void)name;
        (void)stack_size;
        (void)priority;
        (void)core;
        if (fn == nullptr) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
        }
        if (joinable()) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_STATE);
        }
#if !defined(__cpp_exceptions)
        (void)arg;
        return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::UNSUPPORTED);
#else
        try {
            _thread = std::thread{fn, arg};
        } catch (const std::bad_alloc&) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
        } catch (const std::system_error& e) {
            return ::m5::stl::make_unexpected(::m5::variants::frameworks::detail::mapThreadCreateError(e.code()));
        }
#endif
        return {};
    }

    void join(void)
    {
        if (_thread.joinable()) {
            _thread.join();
        }
    }

    bool joinable(void) const
    {
        return _thread.joinable();
    }

private:
    std::thread _thread;
};
#else
/*!
  @brief No-op task for targets with no threading facility (bare-metal
  Arduino core, no RTOS — see M5HAL_STUB_RUNTIME_HAS_STD_THREAD_ above).

  Satisfies the runtime::Task contract's shape, but `start` always fails:
  there is no way to run a second thread of control here. A real Task
  backend for these cores would need cooperative scheduling (e.g. a
  second RP2040 core) and does not exist yet.
 */
using Task = ThreadlessTask;
#endif  // M5HAL_STUB_RUNTIME_HAS_STD_THREAD_

}  // namespace m5::variants::frameworks::stub::hal::v2::runtime

#endif
