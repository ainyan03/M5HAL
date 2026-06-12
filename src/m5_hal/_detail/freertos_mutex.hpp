// SPDX-License-Identifier: MIT
#ifndef M5_HAL_DETAIL_FREERTOS_MUTEX_HPP
#define M5_HAL_DETAIL_FREERTOS_MUTEX_HPP

// Shared FreeRTOS mutex backing for the runtime kind.
//
// Both FreeRTOS-hosted variants alias this as their `runtime::Mutex`:
// arduino (arduino-esp32 is a FreeRTOS environment, [026] E4) and
// espidf. The variant headers only differ in their time functions, so
// the mutex lives here once — the same _detail/ sharing pattern as
// bsd_tcp.
//
// Availability self-gate: on platforms without the FreeRTOS headers
// this file compiles to nothing; M5HAL_DETAIL_FREERTOS_MUTEX_AVAILABLE
// tells the includer which way it went.

#if __has_include(<freertos/FreeRTOS.h>)
#define M5HAL_DETAIL_FREERTOS_MUTEX_AVAILABLE 1
#else
#define M5HAL_DETAIL_FREERTOS_MUTEX_AVAILABLE 0
#endif

#if M5HAL_DETAIL_FREERTOS_MUTEX_AVAILABLE

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstdint>

namespace m5::hal::v1::detail {

/*!
  @brief FreeRTOS-backed mutex satisfying the runtime::Mutex contract.

  - `lock(0)` is an immediate try-lock; a nonzero timeout rounds UP to
    the tick so it always waits at least one tick (granularity:
    `portTICK_PERIOD_MS`, 10 ms at the default 100 Hz tick rate).
    `lock(types::TIMEOUT_FOREVER)` blocks indefinitely (portMAX_DELAY,
    bypassing the pdMS_TO_TICKS conversion that would overflow).
  - Non-recursive: a re-lock from the holding task waits until the
    timeout and fails (with TIMEOUT_FOREVER it deadlocks — the task
    watchdog is the fail-loud backstop).
  - Task context only: priority-inheritance mutexes cannot be taken or
    given from an ISR, and `unlock` must come from the holding
    task (FreeRTOS requirement).
 */
class FreeRtosMutex {
public:
#if configSUPPORT_STATIC_ALLOCATION
    FreeRtosMutex(void) : _handle{xSemaphoreCreateMutexStatic(&_buffer)}
    {
    }
#else
    FreeRtosMutex(void) : _handle{xSemaphoreCreateMutex()}
    {
    }
#endif
    // vSemaphoreDelete knows about statically created semaphores and
    // skips the heap free for them, so one dtor covers both paths.
    ~FreeRtosMutex(void)
    {
        vSemaphoreDelete(_handle);
    }
    FreeRtosMutex(const FreeRtosMutex&)            = delete;
    FreeRtosMutex& operator=(const FreeRtosMutex&) = delete;

    bool lock(uint32_t timeout_ms)
    {
        TickType_t ticks = 0;
        if (timeout_ms == 0xFFFFFFFFu) {  // types::TIMEOUT_FOREVER
            ticks = portMAX_DELAY;
        } else if (timeout_ms != 0) {
            ticks = pdMS_TO_TICKS(timeout_ms);
            if (ticks == 0) {
                ticks = 1;
            }
        }
        return xSemaphoreTake(_handle, ticks) == pdTRUE;
    }
    void unlock(void)
    {
        (void)xSemaphoreGive(_handle);
    }

private:
#if configSUPPORT_STATIC_ALLOCATION
    StaticSemaphore_t _buffer{};
#endif
    SemaphoreHandle_t _handle;
};

}  // namespace m5::hal::v1::detail

#endif  // M5HAL_DETAIL_FREERTOS_MUTEX_AVAILABLE

#endif
