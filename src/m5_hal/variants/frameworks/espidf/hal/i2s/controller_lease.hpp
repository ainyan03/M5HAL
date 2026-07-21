// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_CONTROLLER_LEASE_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_CONTROLLER_LEASE_HPP

#include "../../../../../hal/v2/types.hpp"

#include <cstdint>

namespace m5::hal::v2::detail_espidf_i2s_controller {

// Standard I2S and PDM are different public kinds but consume this same
// peripheral family. Standard claims from the highest port down so I2S0,
// the PDM-capable port on current supported SoCs, remains available when a
// second controller exists.
int8_t claimStandard(uint32_t excluded_mask = 0);
int8_t claimPdm();
void release(int8_t controller);

}  // namespace m5::hal::v2::detail_espidf_i2s_controller

#endif
