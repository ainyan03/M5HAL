// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_EVENT_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_EVENT_HPP

#if __has_include(<freertos/FreeRTOS.h>)

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstdint>

#include "time.hpp"

namespace m5::variants::frameworks::freertos::hal::v2::runtime {

/*!
  @brief Latching binary event satisfying the runtime::Event contract
  (spec/design/runtime.md), backed by a FreeRTOS binary semaphore.

  A binary semaphore natively provides the latch (a Give before the
  Take is consumed by the next Take) and the merge (repeated Gives
  collapse into one pending token), so wait/notify map directly onto
  Take/Give. Tick conversion rounds up, same as Mutex.

  With dynamic allocation the create can fail; a null handle degrades
  to wait=false / notify=no-op (the Mutex null policy).
 */
class Event {
public:
#if configSUPPORT_STATIC_ALLOCATION
    Event(void) : _handle{xSemaphoreCreateBinaryStatic(&_buffer)}
    {
    }
#else
    Event(void) : _handle{xSemaphoreCreateBinary()}
    {
    }
#endif
    ~Event(void)
    {
        if (_handle != nullptr) {
            vSemaphoreDelete(_handle);
        }
    }
    Event(const Event&)            = delete;
    Event& operator=(const Event&) = delete;

    bool wait(uint32_t timeout_ms)
    {
        return _handle != nullptr &&
               xSemaphoreTake(_handle, ::m5::hal::v2::detail::timeoutMsToTicksRoundUp(timeout_ms)) == pdTRUE;
    }
    void notify(void)
    {
        if (_handle != nullptr) {
            (void)xSemaphoreGive(_handle);
        }
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
