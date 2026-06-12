// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_RUNTIME_RUNTIME_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_RUNTIME_RUNTIME_HPP

// runtime kind for the arduino framework variant: time through the
// Arduino core API, mutex through the shared FreeRTOS detail
// (arduino-esp32 is a FreeRTOS environment, [026] E4).
// Authoritative contract: spec/design/runtime.md.

#if defined(ARDUINO)

#include <Arduino.h>

#include <cstdint>

#include "../../../../../_detail/freertos_mutex.hpp"

namespace m5::variants::frameworks::arduino::hal::v1::runtime {

inline uint32_t millis(void)
{
    return static_cast<uint32_t>(::millis());
}
inline uint32_t micros(void)
{
    return static_cast<uint32_t>(::micros());
}
inline void delayMs(uint32_t ms)
{
    ::delay(ms);
}
inline void delayUs(uint32_t us)
{
    ::delayMicroseconds(us);
}

using Mutex = ::m5::hal::v1::detail::FreeRtosMutex;

}  // namespace m5::variants::frameworks::arduino::hal::v1::runtime

#endif  // ARDUINO

#endif
