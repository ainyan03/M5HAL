// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_RUNTIME_RUNTIME_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_RUNTIME_RUNTIME_HPP

#include "../../../_checker.hpp"

// runtime kind for the POSIX host framework variant: time through
// CLOCK_MONOTONIC / nanosleep, mutex through std::timed_mutex. Unlike
// UART, this kind is NOT affected by the M5HAL_CONFIG_POSIX_UART
// opt-out (suppressing the host serial port must not downgrade every
// Bus to the stub fake mutex).
// Authoritative contract: spec/design/runtime.md.

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <time.h>

#if defined(M5HAL_TEST_POSIX_MUTEX_FAULTS) || defined(M5HAL_TEST_POSIX_EVENT_FAULTS)
#include <atomic>
#endif
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <new>
#include <system_error>
#include <thread>

#include "../../../../../hal/v2/error.hpp"
#include "../../../detail/thread_create_error.hpp"

namespace m5::variants::frameworks::posix::hal::v2::runtime {

#if defined(M5HAL_TEST_POSIX_MUTEX_FAULTS) && defined(__cpp_exceptions)
namespace detail {
inline std::atomic<bool> mutex_lock_system_error_once{false};
inline std::atomic<bool> mutex_unlock_error_once{false};

inline void failNextMutexLockWithSystemError(void)
{
    mutex_lock_system_error_once.store(true, std::memory_order_release);
}

inline void failNextMutexUnlockWithIoError(void)
{
    mutex_unlock_error_once.store(true, std::memory_order_release);
}
}  // namespace detail
#endif

#if defined(M5HAL_TEST_POSIX_EVENT_FAULTS) && defined(__cpp_exceptions)
namespace detail {
inline std::atomic<bool> event_wait_system_error_once{false};

inline void failNextEventWaitWithSystemError(void)
{
    event_wait_system_error_once.store(true, std::memory_order_release);
}

inline bool eventWaitSystemErrorPending(void)
{
    return event_wait_system_error_once.load(std::memory_order_acquire);
}
}  // namespace detail
#endif

inline uint32_t millis(void)
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint32_t>(static_cast<uint64_t>(ts.tv_sec) * 1000u +
                                 static_cast<uint64_t>(ts.tv_nsec) / 1000000u);
}
inline uint32_t micros(void)
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint32_t>(static_cast<uint64_t>(ts.tv_sec) * 1000000u +
                                 static_cast<uint64_t>(ts.tv_nsec) / 1000u);
}
inline void delayMs(uint32_t ms)
{
    timespec req{static_cast<time_t>(ms / 1000u), static_cast<long>((ms % 1000u) * 1000000L)};
    while (nanosleep(&req, &req) != 0 && errno == EINTR) {
    }
}
inline void delayUs(uint32_t us)
{
    timespec req{static_cast<time_t>(us / 1000000u), static_cast<long>((us % 1000000u) * 1000L)};
    while (nanosleep(&req, &req) != 0 && errno == EINTR) {
    }
}
inline void yield(void)
{
    std::this_thread::yield();
}

/*!
  @brief Opaque identity of the calling thread: the address of a
  thread_local object.

  Distinct per thread and never null; meaningful for same-value
  comparison only (never dereferenced). ServiceRunner uses it to tell a
  self-call (add/remove issued from inside a service on the runner's own
  task) from a foreign thread.
 */
inline void* currentTaskId(void)
{
    static thread_local char id;
    return &id;
}

/*!
  @brief std::timed_mutex satisfying the runtime::Mutex contract.

  `lock(types::TIMEOUT_FOREVER)` switches to a plain blocking lock().
  Non-recursive: attempting to lock again from the owning thread violates
  the runtime contract (the C++ standard makes that use of timed_mutex
  undefined). Backend failures reported as system_error are IO_ERROR.
 */
class Mutex {
public:
    Mutex(void)                    = default;
    Mutex(const Mutex&)            = delete;
    Mutex& operator=(const Mutex&) = delete;

    ::m5::hal::v2::result_t<void> lock(uint32_t timeout_ms)
    {
#if defined(__cpp_exceptions)
        try {
#if defined(M5HAL_TEST_POSIX_MUTEX_FAULTS)
            if (detail::mutex_lock_system_error_once.exchange(false, std::memory_order_acq_rel)) {
                throw std::system_error{std::make_error_code(std::errc::io_error)};
            }
#endif
#endif
            if (timeout_ms == 0) {
                if (!_mutex.try_lock()) {
                    return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
                }
                return {};
            }
            if (timeout_ms == 0xFFFFFFFFu) {  // types::TIMEOUT_FOREVER
                _mutex.lock();
                return {};
            }
            if (!_mutex.try_lock_for(std::chrono::milliseconds{timeout_ms})) {
                return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
            }
            return {};
#if defined(__cpp_exceptions)
        } catch (const std::system_error&) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::IO_ERROR);
        }
#endif
    }
    ::m5::hal::v2::result_t<void> unlock(void)
    {
        _mutex.unlock();
#if defined(M5HAL_TEST_POSIX_MUTEX_FAULTS) && defined(__cpp_exceptions)
        if (detail::mutex_unlock_error_once.exchange(false, std::memory_order_acq_rel)) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::IO_ERROR);
        }
#endif
        return {};
    }

private:
    std::timed_mutex _mutex;
};

/*!
  @brief Latching binary event satisfying the runtime::Event contract
  (spec/design/runtime.md): mutex + condition_variable + bool flag.

  The condvar alone is NOT latching — the flag carries a notify that
  arrives before the wait. Every wait path checks/consumes the flag
  under the same mutex notify() sets it under, which also provides the
  release/acquire visibility pairing the contract requires.

  A plain single-shot wait_for WITHOUT a predicate is forbidden here:
  a spurious wakeup would be misreported as a timeout. The predicate
  form re-checks the flag and keeps waiting out the remaining time.
 */
class Event {
public:
    Event(void)                    = default;
    Event(const Event&)            = delete;
    Event& operator=(const Event&) = delete;

    ::m5::hal::v2::result_t<void> wait(uint32_t timeout_ms)
    {
#if defined(__cpp_exceptions)
        try {
#if defined(M5HAL_TEST_POSIX_EVENT_FAULTS)
            if (detail::event_wait_system_error_once.exchange(false, std::memory_order_acq_rel)) {
                throw std::system_error{std::make_error_code(std::errc::io_error)};
            }
#endif
#endif
            std::unique_lock<std::mutex> lock{_mutex};
            if (timeout_ms == 0) {  // non-blocking check
                if (!_signaled) {
                    return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
                }
                _signaled = false;
                return {};
            }
            if (timeout_ms == 0xFFFFFFFFu) {  // types::TIMEOUT_FOREVER
                _cv.wait(lock, [this] { return _signaled; });
                _signaled = false;
                return {};
            }
            if (!_cv.wait_for(lock, std::chrono::milliseconds{timeout_ms}, [this] { return _signaled; })) {
                return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
            }
            _signaled = false;
            return {};
#if defined(__cpp_exceptions)
        } catch (const std::system_error&) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::IO_ERROR);
        }
#endif
    }
    void notify(void)
    {
        {
            std::lock_guard<std::mutex> lock{_mutex};
            _signaled = true;
        }
        _cv.notify_one();
    }

private:
    std::mutex _mutex;
    std::condition_variable _cv;
    bool _signaled = false;
};

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

}  // namespace m5::variants::frameworks::posix::hal::v2::runtime

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
