#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_INL

#include "i2c.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
#include "backend_master_gen5.inl"
#elif defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
#include "backend_master_gen4.inl"

#endif

#endif
