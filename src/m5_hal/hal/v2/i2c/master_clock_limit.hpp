// SPDX-License-Identifier: MIT
#ifndef M5_HAL_I2C_MASTER_CLOCK_LIMIT_HPP_
#define M5_HAL_I2C_MASTER_CLOCK_LIMIT_HPP_

#include <cstdint>

/*!
  @file master_clock_limit.hpp
  @brief Fail-safe ceiling for the I2C master SCL clock.

  Its purpose is narrow: keep the hardware I2C master peripheral out of *abnormal
  operation*. Above its valid clock-generation ceiling the master stops emitting a
  waveform and the lines stick (a confusing, bug-looking failure that drags users
  into a debugging swamp). This is NOT a spec or bus-quality limit -- a clock that
  is too high for weak pull-ups merely fails to communicate, which is the user's
  wiring concern and is intentionally left alone.

  Requests above the ceiling are CLAMPED to it (never an error), so a caller that
  asks for e.g. 2 MHz simply gets the closest achievable clock without needing to
  know the limit (matching how a user expects "set the fastest you can" to behave).

  Override @c M5HAL_I2C_MASTER_MAX_CLOCK_HZ to raise or lower the ceiling.

  Characterization: only ESP32 (classic) is measured so far -- 1.25 MHz works,
  1.30 MHz drives the master peripheral into abnormal operation (Core2 <-> CoreS3
  HIL, logic analyzer: SCL/SDA stuck high, master not driving). 1.20 MHz is used as
  a margin. Other ESP SoCs are not yet measured and reuse the same conservative
  default; tune per board/SoC once characterized. Non-ESP targets impose no ceiling
  (0 = disabled): the software bit-bang backend has no peripheral to protect, and
  Arduino ports to other cores manage their own limits.
 */

#ifndef M5HAL_I2C_MASTER_MAX_CLOCK_HZ
#if defined(ESP_PLATFORM)
#define M5HAL_I2C_MASTER_MAX_CLOCK_HZ 1200000u
#else
#define M5HAL_I2C_MASTER_MAX_CLOCK_HZ 0u  // 0 = no ceiling
#endif
#endif

namespace m5::hal::v2::i2c {

/*!
  @brief Clamp an I2C master clock to the fail-safe ceiling.
  @param freq Requested SCL clock in Hz.
  @param did_clamp Optional out-flag set true when the value was reduced (lets the
         caller log once without this header pulling in a logging dependency).
  @return @c freq, or the ceiling when @c freq exceeds it. A ceiling of 0 disables
          clamping (returns @c freq unchanged).
 */
inline std::uint32_t clampMasterClockHz(std::uint32_t freq, bool* did_clamp = nullptr)
{
    constexpr std::uint32_t kMax = M5HAL_I2C_MASTER_MAX_CLOCK_HZ;
    if (kMax != 0u && freq > kMax) {
        if (did_clamp != nullptr) {
            *did_clamp = true;
        }
        return kMax;
    }
    if (did_clamp != nullptr) {
        *did_clamp = false;
    }
    return freq;
}

}  // namespace m5::hal::v2::i2c

#endif  // M5_HAL_I2C_MASTER_CLOCK_LIMIT_HPP_
