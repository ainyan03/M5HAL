// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_STUB_HAL_RUNTIME_RUNTIME_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_STUB_HAL_RUNTIME_RUNTIME_HPP

// runtime kind for the `stub` variant — a deterministic fake for
// native tests and the unconditional fallback. The clock starts at
// zero and advances ONLY through delayMs / delayUs; the mutex is a
// single-task owner guard.
// Authoritative contract: spec/design/runtime.md.

#include <cstdint>

namespace m5::variants::frameworks::stub::hal::v1::runtime {

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

}  // namespace m5::variants::frameworks::stub::hal::v1::runtime

#endif
