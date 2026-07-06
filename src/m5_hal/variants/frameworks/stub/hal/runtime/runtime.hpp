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
#include <thread>

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
    std::this_thread::yield();
}
/*! @brief Rewind the fake clock to zero (test isolation hook). */
inline void fakeReset(void)
{
    fakeMicrosRef() = 0;
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

    bool lock(uint32_t timeout_ms)
    {
        (void)timeout_ms;
        if (_locked) {
            return false;
        }
        _locked = true;
        return true;
    }
    void unlock(void)
    {
        _locked = false;
    }

private:
    bool _locked = false;
};

/*!
  @brief std::thread-backed task satisfying the runtime::Task contract.

  `name`, `stack_size`, and `priority` are accepted for API parity
  with FreeRTOS targets and ignored by this backend.
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

    bool start(entry_fn_t fn, void* arg, const char* name = nullptr, size_t stack_size = 4096, int priority = 1)
    {
        (void)name;
        (void)stack_size;
        (void)priority;
        if (joinable() || fn == nullptr) {
            return false;
        }
        _thread = std::thread{fn, arg};
        return true;
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

}  // namespace m5::variants::frameworks::stub::hal::v2::runtime

#endif
