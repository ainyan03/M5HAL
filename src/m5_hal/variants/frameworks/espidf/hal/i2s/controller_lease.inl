// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_CONTROLLER_LEASE_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_CONTROLLER_LEASE_INL

#include "controller_lease.hpp"

#if defined(ESP_PLATFORM) && defined(SOC_I2S_SUPPORTED) && SOC_I2S_SUPPORTED

#include <atomic>
#include <soc/soc_caps.h>

namespace m5::hal::v2::detail_espidf_i2s_controller {
namespace {
std::atomic<uint32_t> s_claimed_mask{0};
#if defined(SOC_I2S_NUM)
constexpr int kControllerCount = SOC_I2S_NUM;
#else
// IDF 6 removed SOC_I2S_NUM from the public SoC caps. Its public driver still
// defines port ids 0..2 and validates the target-specific upper bound in
// i2s_new_channel(), so probe that bounded family range from high to low.
constexpr int kControllerCount = 3;
#endif

bool tryClaim(uint8_t controller)
{
    const uint32_t bit = uint32_t{1} << controller;
    uint32_t current   = s_claimed_mask.load(std::memory_order_relaxed);
    while ((current & bit) == 0) {
        if (s_claimed_mask.compare_exchange_weak(current, current | bit, std::memory_order_acq_rel,
                                                 std::memory_order_relaxed)) {
            return true;
        }
    }
    return false;
}
}  // namespace

int8_t claimStandard(uint32_t excluded_mask)
{
    for (int controller = kControllerCount - 1; controller >= 0; --controller) {
        if ((excluded_mask & (uint32_t{1} << controller)) == 0 && tryClaim(static_cast<uint8_t>(controller))) {
            return static_cast<int8_t>(controller);
        }
    }
    return -1;
}

int8_t claimPdm()
{
    return tryClaim(0) ? 0 : -1;
}

void release(int8_t controller)
{
    if (controller >= 0 && controller < kControllerCount) {
        s_claimed_mask.fetch_and(~(uint32_t{1} << controller), std::memory_order_release);
    }
}

}  // namespace m5::hal::v2::detail_espidf_i2s_controller

#endif
#endif
