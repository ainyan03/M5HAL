// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_EVENT_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_EVENT_HPP

#if __has_include(<freertos/FreeRTOS.h>)

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <atomic>
#include <cstdint>

#include "../../../../../hal/v2/error.hpp"
#include "time.hpp"

#if !configSUPPORT_STATIC_ALLOCATION
#error "M5HAL FreeRTOS runtime requires configSUPPORT_STATIC_ALLOCATION=1"
#endif
#if !defined(INCLUDE_vTaskSuspend) || !INCLUDE_vTaskSuspend
#error "M5HAL FreeRTOS Event requires INCLUDE_vTaskSuspend=1 for TIMEOUT_FOREVER"
#endif

namespace m5::variants::frameworks::freertos::hal::v2::runtime {

/*!
  @brief Latching binary event satisfying the runtime::Event contract
  (spec/design/runtime.md), backed by a FreeRTOS binary semaphore.

  A binary semaphore natively provides the latch (a Give before the
  Take is consumed by the next Take) and the merge (repeated Gives
  collapse into one pending token), so wait/notify map directly onto
  Take/Give. Tick conversion rounds up, same as Mutex.

  M5HAL requires FreeRTOS static allocation so construction does not depend
  on heap availability. Null checks remain defensive around a violated port
  contract; dynamic-allocation configurations are rejected at compile time.
 */
class Event {
public:
    Event(void) : _handle{xSemaphoreCreateBinaryStatic(&_buffer)}
    {
    }
    ~Event(void)
    {
        if (_handle != nullptr) {
            vSemaphoreDelete(_handle);
        }
    }
    Event(const Event&)            = delete;
    Event& operator=(const Event&) = delete;

    ::m5::hal::v2::result_t<void> wait(uint32_t timeout_ms)
    {
        if (_handle == nullptr) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_STATE);
        }
        if (xSemaphoreTake(_handle, ::m5::hal::v2::detail::timeoutMsToTicksRoundUp(timeout_ms)) != pdTRUE) {
            return ::m5::stl::make_unexpected(::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
        }
        // A failed give on an already-latched binary semaphore need not be a
        // release operation on every FreeRTOS port. This acquire RMW reads the
        // latest publication RMW, so writes preceding merged notifications are
        // visible too, without turning the Event into a counting semaphore.
        (void)_publication.fetch_add(0, std::memory_order_acquire);
        return {};
    }
    void notify(void)
    {
        (void)_publication.fetch_add(1, std::memory_order_release);
        if (_handle != nullptr) {
            (void)xSemaphoreGive(_handle);
        }
    }

private:
    StaticSemaphore_t _buffer{};
    SemaphoreHandle_t _handle;
    std::atomic<uint32_t> _publication{0};
};

}  // namespace m5::variants::frameworks::freertos::hal::v2::runtime

#endif  // __has_include(<freertos/FreeRTOS.h>)

#endif
