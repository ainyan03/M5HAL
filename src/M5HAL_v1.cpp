// SPDX-License-Identifier: MIT

#include "M5HAL_v1.hpp"
#include "m5_hal/hal/v1/bus/bus.inl"
#include "m5_hal/hal/v1/bytecode/bytecode.inl"
#include "m5_hal/hal/v1/frame/frame.inl"
#include "m5_hal/hal/v1/remote/remote.inl"
#include "m5_hal/hal/v1/i2c/i2c.inl"
#include "m5_hal/hal/v1/memory/pool.inl"
#include "m5_hal/hal/v1/memory/allocator.inl"
#include "m5_hal/hal/v1/spi/spi.inl"
#include "m5_hal/hal/v1/uart/uart.inl"
#include "m5_hal/hal/v1/i2s/i2s.inl"

#define M5HAL_STATIC_MACRO_PATH_IMPL M5HAL_STATIC_MACRO_CONCAT(M5HAL_V1_TARGET_PLATFORM_PATH, hal.inl)

#if M5HAL_V1_TARGET_PLATFORM_VARIANT_ID != M5HAL_V1_VARIANT_ID_NONE
#include M5HAL_STATIC_MACRO_PATH_IMPL
#endif

// Pull in the Arduino-backed implementation only when Arduino is available.
#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include "./m5_hal/variants/frameworks/arduino/hal.inl"
#endif

// ESP-IDF framework variant impl hub. Arduino-on-IDF may compile this
// alongside the arduino framework variant; scan order controls defaults.
#if M5HAL_FRAMEWORK_HAS_ESPIDF
#include "./m5_hal/variants/frameworks/espidf/hal.inl"
#endif

// POSIX host framework variant impl hub (termios serial). Compiled only on a
// plain POSIX host build (see frameworks/_checker.hpp).
#if M5HAL_FRAMEWORK_HAS_POSIX
#include "./m5_hal/variants/frameworks/posix/hal.inl"
#endif

// software variant: always compiled in (provides bit-bang fallback)
#include "./m5_hal/variants/frameworks/software/hal.inl"

// ----- M5HALCore ctor + M5_Hal definition -----
//
// The includes above leave this TU in a "winner-binding complete"
// context, so `m5::hal::v1::gpio::getGPIO()` (the variant-supplied
// MCU GPIO `IGPIO*`) is resolvable here. Closing the
// `M5HALCore::ctor` body and the `M5_Hal` reference definition in
// this single spot keeps the file-stem exception confined to
// `m5_hal.hpp` (no separate `src/m5_hal/hal/v1/m5_hal.cpp`).
//
// An `addGPIO` failure inside the ctor is an invariant break — a
// correctly bound variant must succeed — so we assert /
// fail fast.

#include <cassert>

#if defined(ESP_PLATFORM)
#include <esp_heap_caps.h>
#endif

#if defined(ESP_PLATFORM)
namespace {
uint32_t m5halEspidfHeapCaps(m5::hal::v1::memory::usage_t usage)
{
    using m5::hal::v1::memory::usage_t;
    switch (usage) {
        case usage_t::persistent_slow:
            return MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

        case usage_t::temp:
        case usage_t::persistent:
        default:
            return MALLOC_CAP_DEFAULT;
    }
}

void* m5halEspidfMalloc(size_t size, m5::hal::v1::memory::usage_t usage)
{
    void* ptr = heap_caps_malloc(size, m5halEspidfHeapCaps(usage));
    if (ptr == nullptr && usage == m5::hal::v1::memory::usage_t::persistent_slow) {
        ptr = heap_caps_malloc(size, MALLOC_CAP_DEFAULT);
    }
    return ptr;
}

void* m5halEspidfRealloc(void* ptr, size_t, size_t new_size, m5::hal::v1::memory::usage_t usage)
{
    void* next = heap_caps_realloc(ptr, new_size, m5halEspidfHeapCaps(usage));
    if (next == nullptr && usage == m5::hal::v1::memory::usage_t::persistent_slow) {
        next = heap_caps_realloc(ptr, new_size, MALLOC_CAP_DEFAULT);
    }
    return next;
}

void m5halEspidfFree(void* ptr)
{
    heap_caps_free(ptr);
}
}  // namespace
#endif

namespace m5 {
namespace hal {
M5HAL_INLINE_V1 namespace v1
{
    M5HALCore::M5HALCore()
    {
#if defined(ESP_PLATFORM)
        Memory.setFallback(&m5halEspidfMalloc, &m5halEspidfRealloc, &m5halEspidfFree);
#endif
        // Cannot fail by construction: the group is empty (slot 0 free,
        // storage available) and every variant's getGPIO() returns a
        // non-null instance. The assert documents that invariant; in
        // release builds (assert compiled out) a violation would leave
        // slot 0 empty and surface as gpio_number_t lookups failing.
        auto r = Gpio.addGPIO(gpio::getGPIO(), 0);
        assert(r.has_value() && "M5HALCore::ctor: slot 0 (MCU GPIO) registration failed");
        (void)r;  // Silences a [[nodiscard]] warning when asserts are disabled in release builds.
    }

    // Reference variable backing the `M5_Hal.Gpio.foo()` caller
    // syntax. Eager-initialized; the underlying object is the
    // first-use-lazy function-local static inside `getM5_Hal()`.
    M5HALCore& M5_Hal = getM5_Hal();
}
}  // namespace hal
}  // namespace m5
