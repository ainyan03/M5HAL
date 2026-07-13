// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_TASK_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_TASK_HPP

#if __has_include(<freertos/FreeRTOS.h>) && __has_include(<freertos/task.h>)

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "../../../../../hal/v2/types.hpp"

namespace m5::variants::frameworks::freertos::hal::v2::runtime {

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

    bool start(entry_fn_t fn, void* arg, const char* name = nullptr, size_t stack_size = 4096, int priority = 1,
               int core = ::m5::hal::v2::types::TASK_CORE_ANY)
    {
        if (joinable() || fn == nullptr || stack_size == 0 || stack_size > std::numeric_limits<uint32_t>::max() ||
            priority < 0 || priority >= static_cast<int>(configMAX_PRIORITIES) ||
            core < ::m5::hal::v2::types::TASK_CORE_SAME || core >= static_cast<int>(portNUM_PROCESSORS)) {
            return false;
        }

        _fn  = fn;
        _arg = arg;
        _done.store(false, std::memory_order_relaxed);

        BaseType_t core_id = tskNO_AFFINITY;
        if (core >= 0) {
            core_id = static_cast<BaseType_t>(core);
        } else if (core == ::m5::hal::v2::types::TASK_CORE_SAME) {
            core_id = static_cast<BaseType_t>(xPortGetCoreID());
        } else if (core == ::m5::hal::v2::types::TASK_CORE_OPPOSITE) {
            // Complement of the calling core. On a single-core target this
            // is the calling core itself — equivalent to no affinity.
            core_id = static_cast<BaseType_t>((xPortGetCoreID() + 1) % portNUM_PROCESSORS);
        }

        TaskHandle_t handle = nullptr;
        const BaseType_t ok =
            xTaskCreatePinnedToCore(taskEntry, name != nullptr ? name : "m5hal-task", static_cast<uint32_t>(stack_size),
                                    this, static_cast<UBaseType_t>(priority), &handle, core_id);
        if (ok != pdPASS) {
            _done.store(true, std::memory_order_relaxed);
            _fn  = nullptr;
            _arg = nullptr;
            return false;
        }
        _handle = handle;
        return true;
    }

    void join(void)
    {
        if (!joinable()) {
            return;
        }
        while (!_done.load(std::memory_order_acquire)) {
            vTaskDelay(1);
        }
        _handle = nullptr;
        _fn     = nullptr;
        _arg    = nullptr;
    }

    bool joinable(void) const
    {
        return _handle != nullptr;
    }

private:
    static void taskEntry(void* raw)
    {
        auto* self = static_cast<Task*>(raw);
        self->_fn(self->_arg);
        self->_done.store(true, std::memory_order_release);
        vTaskDelete(nullptr);
    }

    entry_fn_t _fn       = nullptr;
    void* _arg           = nullptr;
    TaskHandle_t _handle = nullptr;
    std::atomic<bool> _done{true};
};

/*!
  @brief Opaque identity of the calling task: its FreeRTOS handle.

  Value is meaningful for same-value comparison only (never
  dereferenced). ServiceRunner uses it to tell a self-call (add/remove
  issued from inside a service running on the runner's own task) from a
  foreign thread. Never null in a running task context.
 */
inline void* currentTaskId(void)
{
    return static_cast<void*>(xTaskGetCurrentTaskHandle());
}

}  // namespace m5::variants::frameworks::freertos::hal::v2::runtime

#endif  // __has_include(<freertos/FreeRTOS.h>) && __has_include(<freertos/task.h>)

#endif
