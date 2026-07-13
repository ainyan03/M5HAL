// SPDX-License-Identifier: MIT
#pragma once

// Peripheral-agnostic FreeRTOS binary-semaphore fake. Like freertos/task.h,
// this never actually parks a caller: xSemaphoreTake returns immediately
// (pdFALSE, i.e. "not given / timed out"). The wave-1 regression scenarios
// never call the consumer wait path (waitForActivity / serve()) that would
// block on this, so the fake only needs to be link-complete, not
// behaviorally faithful. See ../../README.md.

#include "FreeRTOS.h"

struct StaticSemaphore_t {
    int unused = 0;
};
using SemaphoreHandle_t = void*;

inline SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t* storage)
{
    // Non-null sentinel derived from the caller-owned static storage so each
    // bus instance gets a distinct (if unused) handle value.
    return reinterpret_cast<SemaphoreHandle_t>(storage);
}

inline BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t, BaseType_t* higher_priority_task_woken)
{
    if (higher_priority_task_woken != nullptr) {
        *higher_priority_task_woken = pdFALSE;
    }
    return pdTRUE;
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t, TickType_t)
{
    return pdFALSE;
}

// Pulled in transitively by M5HAL's own generic `freertos` framework
// -variant runtime (m5_hal/variants/frameworks/freertos/hal/runtime/
// mutex.hpp / event.hpp -- see freertos/task.h's note on why this compiles
// even though this harness never constructs a `Mutex` or `Event`).
inline SemaphoreHandle_t xSemaphoreCreateMutex()
{
    return reinterpret_cast<SemaphoreHandle_t>(0x1);
}
inline SemaphoreHandle_t xSemaphoreCreateBinary()
{
    return reinterpret_cast<SemaphoreHandle_t>(0x1);
}
inline void vSemaphoreDelete(SemaphoreHandle_t)
{
}
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t)
{
    return pdTRUE;
}
