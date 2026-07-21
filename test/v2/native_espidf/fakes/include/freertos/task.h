// SPDX-License-Identifier: MIT
#pragma once

// Peripheral-agnostic FreeRTOS task fake. This harness never schedules a
// real task: xTaskCreate reports success but does NOT invoke pxTaskCode, so
// a backend's responder task body never runs. This is a deliberate scoping
// choice (see ../../README.md "Scope and fidelity limits") -- the wave-1
// regression scenarios drive the ISR-facing state machine directly via the
// accessor + fireLastIsr(), which does not require the responder task to be
// alive. The explicit successful-stop seam below captures TaskFunction_t and
// runs it synchronously only when a test arms that behavior.

#include "FreeRTOS.h"

#include <cstdint>

#if defined(M5HAL_TEST_ESPIDF_SPI_SLAVE_HOST_HARNESS)
#include <chrono>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <new>
#include <thread>
#endif

using TaskFunction_t         = void (*)(void*);
using TaskHandle_t           = void*;
using configSTACK_DEPTH_TYPE = uint32_t;

namespace m5hal_hostharness {

inline bool& failNextPinnedTaskCreateStorage()
{
    static bool fail = false;
    return fail;
}

inline void failNextPinnedTaskCreate()
{
    failNextPinnedTaskCreateStorage() = true;
}

inline bool consumePinnedTaskCreateFailure()
{
    const bool fail                   = failNextPinnedTaskCreateStorage();
    failNextPinnedTaskCreateStorage() = false;
    return fail;
}

inline bool& runNextPinnedTaskSynchronouslyStorage()
{
    static bool run = false;
    return run;
}

inline void runNextPinnedTaskSynchronously()
{
    runNextPinnedTaskSynchronouslyStorage() = true;
}

inline bool consumePinnedTaskSynchronousRun()
{
    const bool run                          = runNextPinnedTaskSynchronouslyStorage();
    runNextPinnedTaskSynchronouslyStorage() = false;
    return run;
}

}  // namespace m5hal_hostharness

#if defined(M5HAL_TEST_ESPIDF_SPI_SLAVE_HOST_HARNESS)

namespace m5hal_hostharness {

struct TaskDeleted final : std::exception {};

struct HostTask {
    std::mutex mutex;
    std::condition_variable cv;
    std::thread thread;
    uint32_t notifications = 0;
    bool deleted           = false;
    bool start_released    = true;
};

inline thread_local HostTask* currentTask = nullptr;
inline bool holdNewTasks                  = false;

inline void holdNextCreatedTask()
{
    holdNewTasks = true;
}

}  // namespace m5hal_hostharness

inline BaseType_t xTaskCreate(TaskFunction_t function, const char*, configSTACK_DEPTH_TYPE, void* arg, UBaseType_t,
                              TaskHandle_t* out_handle)
{
    auto* task = new (std::nothrow) m5hal_hostharness::HostTask;
    if (task == nullptr) {
        return pdFAIL;
    }
    task->start_released            = !m5hal_hostharness::holdNewTasks;
    m5hal_hostharness::holdNewTasks = false;
    task->thread                    = std::thread([task, function, arg] {
        m5hal_hostharness::currentTask = task;
        try {
            {
                std::unique_lock<std::mutex> lock(task->mutex);
                task->cv.wait(lock, [&] { return task->deleted || task->start_released; });
                if (task->deleted) {
                    throw m5hal_hostharness::TaskDeleted{};
                }
            }
            function(arg);
        } catch (const m5hal_hostharness::TaskDeleted&) {
        }
        m5hal_hostharness::currentTask = nullptr;
    });
    if (out_handle != nullptr) {
        *out_handle = task;
    }
    return pdPASS;
}

inline void vTaskDelete(TaskHandle_t handle)
{
    auto* task = static_cast<m5hal_hostharness::HostTask*>(handle);
    if (task == nullptr) {
        throw m5hal_hostharness::TaskDeleted{};
    }
    {
        std::lock_guard<std::mutex> lock(task->mutex);
        task->deleted = true;
    }
    task->cv.notify_all();
    if (task->thread.joinable()) {
        task->thread.join();
    }
    delete task;
}

inline void vTaskDelay(TickType_t ticks)
{
    const auto milliseconds = static_cast<uint64_t>(ticks) * 1000u / configTICK_RATE_HZ;
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

inline TickType_t xTaskGetTickCount()
{
    using namespace std::chrono;
    const auto elapsed_ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    return static_cast<TickType_t>(static_cast<uint64_t>(elapsed_ms) * configTICK_RATE_HZ / 1000u);
}

inline TickType_t xTaskGetTickCountFromISR()
{
    return xTaskGetTickCount();
}

inline void xTaskNotifyGive(TaskHandle_t handle)
{
    auto* task = static_cast<m5hal_hostharness::HostTask*>(handle);
    if (task == nullptr) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(task->mutex);
        task->start_released = true;
        ++task->notifications;
    }
    task->cv.notify_one();
}

inline void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t* higher_priority_task_woken)
{
    xTaskNotifyGive(handle);
    if (higher_priority_task_woken != nullptr) {
        *higher_priority_task_woken = pdFALSE;
    }
}

inline uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t timeout)
{
    auto* task = m5hal_hostharness::currentTask;
    if (task == nullptr) {
        return 0;
    }
    std::unique_lock<std::mutex> lock(task->mutex);
    const auto ready = [&] { return task->deleted || task->notifications != 0; };
    if (timeout == portMAX_DELAY) {
        task->cv.wait(lock, ready);
    } else if (!task->cv.wait_for(
                   lock, std::chrono::milliseconds(static_cast<uint64_t>(timeout) * 1000u / configTICK_RATE_HZ),
                   ready)) {
        return 0;
    }
    if (task->deleted) {
        throw m5hal_hostharness::TaskDeleted{};
    }
    const uint32_t observed = task->notifications;
    task->notifications     = clear_on_exit == pdTRUE ? 0 : task->notifications - 1;
    return observed;
}

constexpr BaseType_t tskNO_AFFINITY = -1;

inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t function, const char* name, uint32_t depth, void* arg,
                                          UBaseType_t priority, TaskHandle_t* out_handle, BaseType_t)
{
    return xTaskCreate(function, name, depth, arg, priority, out_handle);
}

inline TaskHandle_t xTaskGetCurrentTaskHandle()
{
    return m5hal_hostharness::currentTask;
}

#else

namespace m5hal_hostharness {

struct CapturedTask {
    TaskFunction_t function = nullptr;
    void* arg               = nullptr;
    bool run_on_next_delay  = false;
};

inline CapturedTask& capturedTask()
{
    static CapturedTask task;
    return task;
}

inline uint32_t& deletedTaskCountStorage()
{
    static uint32_t count = 0;
    return count;
}

inline uint32_t deletedTaskCount()
{
    return deletedTaskCountStorage();
}

// Test-facing successful-stop seam. close() first raises the backend's
// private stop flag, then waitTaskStopped() reaches vTaskDelay(). Running the
// captured task there makes its real task loop observe that flag and exit,
// without exposing backend internals or adding a scheduler to this fake.
inline void runCreatedTaskOnNextDelay()
{
    capturedTask().run_on_next_delay = true;
}

}  // namespace m5hal_hostharness

// Non-null sentinel: the product code only ever compares this handle against
// nullptr and passes it back into vTaskDelete / notify calls below, all of
// which are no-ops here. It is never dereferenced.
inline BaseType_t xTaskCreate(TaskFunction_t function, const char*, configSTACK_DEPTH_TYPE, void* arg, UBaseType_t,
                              TaskHandle_t* out_handle)
{
    m5hal_hostharness::capturedTask() = {function, arg, false};
    if (out_handle != nullptr) {
        *out_handle = reinterpret_cast<TaskHandle_t>(0x1);
    }
    return pdPASS;
}

inline void vTaskDelete(TaskHandle_t)
{
    ++m5hal_hostharness::deletedTaskCountStorage();
    m5hal_hostharness::capturedTask() = {};
}

// No-op (not a real sleep): a backend's close() polls a task-stopped flag
// in a bounded loop with vTaskDelay(1) between checks. Since the fake task
// never runs, that flag never flips, so the loop always exhausts its retry
// budget -- making it a real sleep here would cost wall-clock time on every
// bus close() for no determinism benefit. See ../../README.md.
inline void vTaskDelay(TickType_t)
{
    auto& task = m5hal_hostharness::capturedTask();
    if (task.run_on_next_delay && task.function != nullptr) {
        task.run_on_next_delay = false;
        task.function(task.arg);
    }
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

inline BaseType_t xTaskCreatePinnedToCore(TaskFunction_t function, const char*, uint32_t, void* arg, UBaseType_t,
                                          TaskHandle_t* out_handle, BaseType_t)
{
    if (m5hal_hostharness::consumePinnedTaskCreateFailure()) {
        return pdFAIL;
    }
    if (out_handle != nullptr) {
        *out_handle = reinterpret_cast<TaskHandle_t>(0x1);
    }
    if (m5hal_hostharness::consumePinnedTaskSynchronousRun()) {
        function(arg);
    }
    return pdPASS;
}
inline TaskHandle_t xTaskGetCurrentTaskHandle()
{
    return reinterpret_cast<TaskHandle_t>(0x1);
}

#endif
