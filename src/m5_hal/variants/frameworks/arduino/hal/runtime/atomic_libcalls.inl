// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_RUNTIME_ATOMIC_LIBCALLS_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_RUNTIME_ATOMIC_LIBCALLS_INL

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif

#if defined(ARDUINO) && defined(ARDUINO_ARCH_ESP8266)

#include <cstdint>

// The lx106 has no atomic RMW instructions, so GCC lowers every std::atomic
// RMW to a __atomic_* libcall — and the ESP8266 core ships no libatomic, so
// each of those calls is an undefined reference at link time. Provide the
// operations M5HAL needs as interrupt-masked implementations: the lx106 is a
// single core, so masking interrupts (xt_rsil(15), restored via xt_wsr_ps —
// both from the ESP8266 core's Arduino.h) is sufficient mutual exclusion.
// Plain aligned loads/stores are already atomic there and stay inlined by
// the compiler, so no __atomic_load/store shims are needed. Operations not
// listed keep failing loudly at link instead of silently misbehaving; extend
// this list when a new call site appears. Weak, so a future core/toolchain
// that ships real implementations wins without a duplicate-symbol clash.

namespace {

struct AtomicIsrLock {
    uint32_t ps;
    AtomicIsrLock() : ps(xt_rsil(15))
    {
    }
    ~AtomicIsrLock()
    {
        xt_wsr_ps(ps);
    }
};

}  // namespace

extern "C" {

__attribute__((weak)) uint32_t __atomic_fetch_add_4(volatile void* ptr, uint32_t val, int /*memorder*/)
{
    AtomicIsrLock lock;
    auto* p            = static_cast<volatile uint32_t*>(ptr);
    const uint32_t old = *p;
    *p                 = old + val;
    return old;
}

__attribute__((weak)) uint32_t __atomic_fetch_sub_4(volatile void* ptr, uint32_t val, int /*memorder*/)
{
    AtomicIsrLock lock;
    auto* p            = static_cast<volatile uint32_t*>(ptr);
    const uint32_t old = *p;
    *p                 = old - val;
    return old;
}

__attribute__((weak)) uint8_t __atomic_exchange_1(volatile void* ptr, uint8_t val, int /*memorder*/)
{
    AtomicIsrLock lock;
    auto* p           = static_cast<volatile uint8_t*>(ptr);
    const uint8_t old = *p;
    *p                = val;
    return old;
}

__attribute__((weak)) uint32_t __atomic_exchange_4(volatile void* ptr, uint32_t val, int /*memorder*/)
{
    AtomicIsrLock lock;
    auto* p            = static_cast<volatile uint32_t*>(ptr);
    const uint32_t old = *p;
    *p                 = val;
    return old;
}

__attribute__((weak)) bool __atomic_compare_exchange_4(volatile void* ptr, void* expected, uint32_t desired,
                                                       bool /*weak*/, int /*success_memorder*/,
                                                       int /*failure_memorder*/)
{
    AtomicIsrLock lock;
    auto* p   = static_cast<volatile uint32_t*>(ptr);
    auto* exp = static_cast<uint32_t*>(expected);
    if (*p == *exp) {
        *p = desired;
        return true;
    }
    *exp = *p;
    return false;
}

}  // extern "C"

#endif  // ARDUINO && ARDUINO_ARCH_ESP8266
#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_RUNTIME_ATOMIC_LIBCALLS_INL
