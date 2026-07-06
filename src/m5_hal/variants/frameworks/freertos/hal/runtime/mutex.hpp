// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_MUTEX_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_MUTEX_HPP

#if __has_include(<freertos/FreeRTOS.h>)

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstdint>

#include "time.hpp"

namespace m5::variants::frameworks::freertos::hal::v2::runtime {

class Mutex {
public:
#if configSUPPORT_STATIC_ALLOCATION
    Mutex(void) : _handle{xSemaphoreCreateMutexStatic(&_buffer)}
    {
    }
#else
    Mutex(void) : _handle{xSemaphoreCreateMutex()}
    {
    }
#endif
    ~Mutex(void)
    {
        vSemaphoreDelete(_handle);
    }
    Mutex(const Mutex&)            = delete;
    Mutex& operator=(const Mutex&) = delete;

    bool lock(uint32_t timeout_ms)
    {
        return xSemaphoreTake(_handle, ::m5::hal::v2::detail::timeoutMsToTicksRoundUp(timeout_ms)) == pdTRUE;
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

}  // namespace m5::variants::frameworks::freertos::hal::v2::runtime

#endif  // __has_include(<freertos/FreeRTOS.h>)

#endif
