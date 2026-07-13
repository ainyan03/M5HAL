// SPDX-License-Identifier: MIT
#pragma once

// Peripheral-agnostic FreeRTOS task fake. This harness never schedules a
// real task: xTaskCreate reports success but does NOT invoke pxTaskCode, so
// a backend's responder task body never runs. This is a deliberate scoping
// choice (see ../../README.md "Scope and fidelity limits") -- the wave-1
// regression scenarios drive the ISR-facing state machine directly via the
// accessor + fireLastIsr(), which does not require the responder task to be
// alive. A future scenario that needs the task to actually run would call
// the captured TaskFunction_t synchronously from the test, the same way
// freertos/task.h's own release()-teardown handshake is resolved below.

#include "FreeRTOS.h"

#include <cstdint>

using TaskFunction_t         = void (*)(void*);
using TaskHandle_t           = void*;
using configSTACK_DEPTH_TYPE = uint32_t;

// Non-null sentinel: the product code only ever compares this handle against
// nullptr and passes it back into vTaskDelete / notify calls below, all of
// which are no-ops here. It is never dereferenced.
inline BaseType_t xTaskCreate(TaskFunction_t, const char*, configSTACK_DEPTH_TYPE, void*, UBaseType_t,
                              TaskHandle_t* out_handle)
{
    if (out_handle != nullptr) {
        *out_handle = reinterpret_cast<TaskHandle_t>(0x1);
    }
    return pdPASS;
}

inline void vTaskDelete(TaskHandle_t)
{
}

// No-op (not a real sleep): a backend's release() polls a task-stopped flag
// in a bounded loop with vTaskDelay(1) between checks. Since the fake task
// never runs, that flag never flips, so the loop always exhausts its retry
// budget -- making it a real sleep here would cost wall-clock time on every
// bus release() for no determinism benefit. See ../../README.md.
inline void vTaskDelay(TickType_t)
{
}

inline TickType_t xTaskGetTickCount()
{
    return 0;
}
inline TickType_t xTaskGetTickCountFromISR()
{
    return 0;
}

inline void vTaskNotifyGiveFromISR(TaskHandle_t, BaseType_t* higher_priority_task_woken)
{
    if (higher_priority_task_woken != nullptr) {
        *higher_priority_task_woken = pdFALSE;
    }
}
inline void xTaskNotifyGive(TaskHandle_t)
{
}
inline uint32_t ulTaskNotifyTake(BaseType_t, TickType_t)
{
    return 0;
}

// Pulled in transitively: __has_include(<freertos/FreeRTOS.h>) also flips on
// M5HAL's own generic `freertos` framework-variant runtime (m5_hal/variants/
// frameworks/freertos/hal/runtime/{mutex,task}.hpp), independent of the I2C
// espidf backend under test. Its `Task` class is a plain (non-template)
// class body, so it is TYPE-CHECKED as soon as that header is included --
// these symbols must exist even though nothing in this harness constructs a
// `Task`. Same never-scheduled semantics as xTaskCreate above.
constexpr BaseType_t tskNO_AFFINITY = -1;

inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char*, uint32_t, void*, UBaseType_t,
                                          TaskHandle_t* out_handle, BaseType_t)
{
    if (out_handle != nullptr) {
        *out_handle = reinterpret_cast<TaskHandle_t>(0x1);
    }
    return pdPASS;
}
inline TaskHandle_t xTaskGetCurrentTaskHandle()
{
    return reinterpret_cast<TaskHandle_t>(0x1);
}
