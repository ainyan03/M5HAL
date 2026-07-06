// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_TIME_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_FREERTOS_HAL_RUNTIME_TIME_HPP

// FreeRTOS timeout conversion utilities.
//
// Shared by all FreeRTOS-hosted framework variants (arduino, espidf)
// for bus kind implementations that need native tick conversion.
// Namespace stays m5::hal::v2::detail — these are internal helpers,
// not part of the public runtime contract.

#if __has_include(<freertos/FreeRTOS.h>)

#include <freertos/FreeRTOS.h>

#include <cstdint>

namespace m5::hal::v2::detail {

constexpr uint32_t kTimeoutForeverU32 = 0xFFFFFFFFu;

inline TickType_t timeoutMsToTicks(uint32_t timeout_ms)
{
    if (timeout_ms == 0) {
        return 0;
    }
    if (timeout_ms == kTimeoutForeverU32) {
        return portMAX_DELAY;
    }
    const uint64_t ticks = (static_cast<uint64_t>(timeout_ms) * static_cast<uint64_t>(configTICK_RATE_HZ)) / 1000u;
    return ticks >= static_cast<uint64_t>(portMAX_DELAY) ? portMAX_DELAY : static_cast<TickType_t>(ticks);
}

inline TickType_t timeoutMsToTicksRoundUp(uint32_t timeout_ms)
{
    if (timeout_ms == 0) {
        return 0;
    }
    if (timeout_ms == kTimeoutForeverU32) {
        return portMAX_DELAY;
    }
    const uint64_t ticks =
        (static_cast<uint64_t>(timeout_ms) * static_cast<uint64_t>(configTICK_RATE_HZ) + 999u) / 1000u;
    return ticks >= static_cast<uint64_t>(portMAX_DELAY) ? portMAX_DELAY : static_cast<TickType_t>(ticks);
}

inline TickType_t delayMsToTicksAtLeast(uint32_t ms)
{
    TickType_t ticks = timeoutMsToTicksRoundUp(ms);
    if (ticks != 0 && ticks != portMAX_DELAY) {
        ++ticks;
        if (ticks == 0 || ticks > portMAX_DELAY) {
            ticks = portMAX_DELAY;
        }
    }
    return ticks;
}

inline uint32_t timeoutMsToUsecU32(uint32_t timeout_ms)
{
    return timeout_ms > (0xFFFFFFFFu / 1000u) ? 0xFFFFFFFFu : static_cast<uint32_t>(timeout_ms * 1000u);
}

}  // namespace m5::hal::v2::detail

#endif  // __has_include(<freertos/FreeRTOS.h>)

#endif
