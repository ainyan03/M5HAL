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

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <mutex>

namespace m5::variants::frameworks::posix::hal::v1::runtime {

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

/*!
  @brief std::timed_mutex satisfying the runtime::Mutex contract.

  `lock(types::TIMEOUT_FOREVER)` switches to a plain blocking lock().
  Non-recursive: a re-lock from the holding thread waits until the
  timeout and fails (with TIMEOUT_FOREVER it deadlocks). The C++
  standard leaves an owner's try_lock(_for) undefined, but both
  deployed implementations resolve it as a plain timeout (libstdc++
  via pthread_mutex_timedlock on a NORMAL mutex, libc++ via its own
  mutex + condvar), which is what the contract specifies.
 */
class Mutex {
public:
    Mutex(void)                    = default;
    Mutex(const Mutex&)            = delete;
    Mutex& operator=(const Mutex&) = delete;

    bool lock(uint32_t timeout_ms)
    {
        if (timeout_ms == 0) {
            return _mutex.try_lock();
        }
        if (timeout_ms == 0xFFFFFFFFu) {  // types::TIMEOUT_FOREVER
            _mutex.lock();
            return true;
        }
        return _mutex.try_lock_for(std::chrono::milliseconds{timeout_ms});
    }
    void unlock(void)
    {
        _mutex.unlock();
    }

private:
    std::timed_mutex _mutex;
};

}  // namespace m5::variants::frameworks::posix::hal::v1::runtime

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
