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
    BaseType_t count = 0;
};
using SemaphoreHandle_t = void*;

// Observable argument for provider-contract tests. The fake never blocks,
// but retaining the requested budget lets the harness verify that M5HAL
// preserves zero, finite and portMAX_DELAY semantics at the FreeRTOS seam.
inline TickType_t m5hal_fake_last_semaphore_take_ticks = 0;

inline SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t* storage)
{
    // Non-null sentinel derived from the caller-owned static storage so each
    // bus instance gets a distinct (if unused) handle value.
    storage->count = 0;
    return reinterpret_cast<SemaphoreHandle_t>(storage);
}

inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t* storage)
{
    storage->count = 1;
    return reinterpret_cast<SemaphoreHandle_t>(storage);
}

inline BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t handle, BaseType_t* higher_priority_task_woken)
{
    if (handle == nullptr) return pdFALSE;
    static_cast<StaticSemaphore_t*>(handle)->count = 1;
    if (higher_priority_task_woken != nullptr) {
        *higher_priority_task_woken = pdFALSE;
    }
    return pdTRUE;
}

inline BaseType_t xSemaphoreTake(SemaphoreHandle_t handle, TickType_t ticks)
{
    m5hal_fake_last_semaphore_take_ticks = ticks;
    if (handle == nullptr) return pdFALSE;
    auto* semaphore = static_cast<StaticSemaphore_t*>(handle);
    if (semaphore->count == 0) return pdFALSE;
    semaphore->count = 0;
    return pdTRUE;
}

// Pulled in by M5HAL's generic FreeRTOS runtime. The host harness constructs
// Mutex and Event directly to pin their result and timeout-conversion contracts.
inline SemaphoreHandle_t xSemaphoreCreateMutex()
{
    static StaticSemaphore_t storage{1};
    storage.count = 1;
    return &storage;
}
inline SemaphoreHandle_t xSemaphoreCreateBinary()
{
    static StaticSemaphore_t storage{};
    storage.count = 0;
    return &storage;
}
inline void vSemaphoreDelete(SemaphoreHandle_t)
{
}
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t handle)
{
    if (handle == nullptr) return pdFALSE;
    auto* semaphore = static_cast<StaticSemaphore_t*>(handle);
    if (semaphore->count != 0) return pdFALSE;
    semaphore->count = 1;
    return pdTRUE;
}
