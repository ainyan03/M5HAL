// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_MUTEX_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_MUTEX_HPP

#if __has_include(<freertos/FreeRTOS.h>)

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstdint>

#include "../../../../../hal/v2/error.hpp"
#include "time.hpp"

#if !configSUPPORT_STATIC_ALLOCATION
#error "M5HAL FreeRTOS runtime requires configSUPPORT_STATIC_ALLOCATION=1"
#endif
#if !defined(INCLUDE_vTaskSuspend) || !INCLUDE_vTaskSuspend
#error "M5HAL FreeRTOS Mutex requires INCLUDE_vTaskSuspend=1 for TIMEOUT_FOREVER"
#endif

namespace m5::variants::frameworks::freertos::hal::v2::runtime {

class Mutex {
public:
    Mutex(void) : _handle{xSemaphoreCreateMutexStatic(&_buffer)}
    {
    }
    ~Mutex(void)
    {
        if (_handle != nullptr) {
            vSemaphoreDelete(_handle);
        }
    }
    Mutex(const Mutex&)            = delete;
    Mutex& operator=(const Mutex&) = delete;

    ::m5::hal::v2::result_t<void> lock(uint32_t timeout_ms)
    {
        if (_handle == nullptr) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_STATE);
        }
        if (xSemaphoreTake(_handle, ::m5::hal::v2::detail::timeoutMsToTicksRoundUp(timeout_ms)) != pdTRUE) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
        }
        return {};
    }
    ::m5::hal::v2::result_t<void> unlock(void)
    {
        if (_handle == nullptr || xSemaphoreGive(_handle) != pdTRUE) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_STATE);
        }
        return {};
    }

private:
    StaticSemaphore_t _buffer{};
    SemaphoreHandle_t _handle;
};

}  // namespace m5::variants::frameworks::freertos::hal::v2::runtime

#endif  // __has_include(<freertos/FreeRTOS.h>)

#endif
