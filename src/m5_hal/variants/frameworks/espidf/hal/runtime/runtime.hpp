// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_RUNTIME_RUNTIME_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_RUNTIME_RUNTIME_HPP

// runtime kind for the ESP-IDF framework variant: time through
// esp_timer (64-bit µs since boot, truncated to the 32-bit wrap-around
// contract), mutex through the shared FreeRTOS detail.
// Authoritative contract: spec/design/runtime.md.

#if defined(ESP_PLATFORM)

#include <esp_rom_sys.h>  // esp_rom_delay_us (IDF >= 4.3)
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>

#include "../../../../../_detail/freertos_mutex.hpp"

namespace m5::variants::frameworks::espidf::hal::v1::runtime {

inline uint32_t millis(void)
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}
inline uint32_t micros(void)
{
    return static_cast<uint32_t>(esp_timer_get_time());
}
inline void delayMs(uint32_t ms)
{
    // vTaskDelay(n) wakes between (n-1) and n ticks from the call; +1
    // guarantees "at least ms" at tick granularity. delayMs(0) keeps
    // the Arduino delay(0) behaviour (a bare yield).
    vTaskDelay(ms != 0 ? pdMS_TO_TICKS(ms) + 1 : 0);
}
inline void delayUs(uint32_t us)
{
    esp_rom_delay_us(us);
}

using Mutex = ::m5::hal::v1::detail::FreeRtosMutex;

}  // namespace m5::variants::frameworks::espidf::hal::v1::runtime

#endif  // ESP_PLATFORM

#endif
