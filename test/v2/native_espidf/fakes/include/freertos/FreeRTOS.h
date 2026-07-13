// SPDX-License-Identifier: MIT
#pragma once

// Peripheral-agnostic FreeRTOS core fake for the native espidf host harness
// (see ../../README.md for the harness design). Covers the handful of core
// types/macros the ISR-driven backends under test/v2/native_espidf need:
// tick types, critical-section stand-ins, and the pd* result codes. No
// scheduler exists in this harness -- see freertos/task.h and
// freertos/semphr.h for what that implies for xTaskCreate / semaphores.

#include <cstdint>

using BaseType_t  = int;
using UBaseType_t = unsigned int;
using TickType_t  = uint32_t;

constexpr BaseType_t pdTRUE  = 1;
constexpr BaseType_t pdFALSE = 0;
constexpr BaseType_t pdPASS  = 1;
constexpr BaseType_t pdFAIL  = 0;

constexpr TickType_t portMAX_DELAY = 0xFFFFFFFFu;

// Fake tick rate: 1 tick == 1 ms, so pdMS_TO_TICKS is the identity. Only
// m5_hal/variants/frameworks/freertos/hal/runtime/time.hpp reads
// configTICK_RATE_HZ (timeoutMsToTicks); the state-machine code under test
// never converts a wall-clock duration, so the exact rate is not
// load-bearing for the regression scenarios -- only that it is defined.
constexpr uint32_t configTICK_RATE_HZ      = 1000;
constexpr UBaseType_t configMAX_PRIORITIES = 25;

// Single-core fake: core-placement queries resolve to core 0. Used by the
// freertos runtime Task's core argument (TASK_CORE_SAME / TASK_CORE_OPPOSITE
// both degrade to core 0 here, same as a real single-core target).
constexpr int portNUM_PROCESSORS = 1;

inline BaseType_t xPortGetCoreID()
{
    return 0;
}

inline TickType_t pdMS_TO_TICKS(uint32_t ms)
{
    return static_cast<TickType_t>(ms);
}

// Real FreeRTOS portMUX_TYPE is a spinlock record; this harness is single
// -threaded (no task ever actually runs -- see freertos/task.h), so the
// critical-section primitives below are no-ops over an opaque placeholder.
struct portMUX_TYPE {
    int unused = 0;
};
inline constexpr portMUX_TYPE portMUX_INITIALIZER_UNLOCKED{};

inline void portENTER_CRITICAL_SAFE(portMUX_TYPE*)
{
}
inline void portEXIT_CRITICAL_SAFE(portMUX_TYPE*)
{
}
inline void portENTER_CRITICAL_ISR(portMUX_TYPE*)
{
}
inline void portEXIT_CRITICAL_ISR(portMUX_TYPE*)
{
}
inline void portYIELD_FROM_ISR()
{
}
