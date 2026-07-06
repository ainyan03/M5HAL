// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_I2C_INL

#include "i2c.hpp"
#include "slave.hpp"  // defines M5HAL_ESPIDF_I2C_SLAVE_LL used by the gate below

// The slave backend is provided whenever the SoC can drive a real SCL stretch
// (LL stretch primitive, e.g. ESP32-S3) or the v2 slave driver is configured
// (classic ESP32 fill-only path). slave.hpp derives M5HAL_ESPIDF_I2C_SLAVE_LL.
#if defined(ESP_PLATFORM) && (M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_HAS_SLAVE_V2)
#include "slave.inl"
#endif

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5
#include "backend_master_gen5.inl"
#elif defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4
#include "backend_master_gen4.inl"

#endif

#endif
