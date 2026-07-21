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

// Use the common 100 Hz embedded configuration. A non-1:1 millisecond/tick
// ratio makes timeout conversion tests capable of detecting truncation in a
// path that promises to round finite waits up.
constexpr uint32_t configTICK_RATE_HZ      = 100;
constexpr UBaseType_t configMAX_PRIORITIES = 25;
#define configSUPPORT_STATIC_ALLOCATION 1
#define INCLUDE_vTaskSuspend            1

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
    return static_cast<TickType_t>((static_cast<uint64_t>(ms) * configTICK_RATE_HZ) / 1000u);
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

#if defined(M5HAL_TEST_ESPIDF_SPI_SLAVE_HOST_HARNESS)
#include <thread>
inline void taskYIELD()
{
    std::this_thread::yield();
}
#endif
