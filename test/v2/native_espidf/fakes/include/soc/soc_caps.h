// SPDX-License-Identifier: MIT
#pragma once

// Fake capability probes for the native espidf host harness, sized to make
// the espidf I2C slave backend select its LL stretch-primitive path (see
// espidf/hal/i2c/slave.hpp's M5HAL_ESPIDF_I2C_SLAVE_LL derivation). Values
// match a real stretch-cause SoC (e.g. ESP32-S3) by default.
//
// Define M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_NO_STRETCH_CAPABILITY to flip
// SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE
// to 0 instead: the same LL-path headers (hal/i2c_ll.h, soc/i2c_struct.h,
// soc/i2c_periph.h) stay __has_include-able, so the espidf slave backend's
// M5HAL_ESPIDF_I2C_SLAVE_LL_BE gate now resolves true instead of
// M5HAL_ESPIDF_I2C_SLAVE_LL -- the classic-ESP32 best-effort flavor, exercised
// by the test_espidf_i2c_slave_be env.
#define SOC_I2C_FIFO_LEN      32
#define SOC_I2C_SUPPORT_SLAVE 1
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_NO_STRETCH_CAPABILITY)
#define SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE 0
#else
#define SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE 1
#endif

// SOC_RCC_IS_INDEPENDENT=1 routes slave.inl's init() through the direct
// i2c_ll_enable_bus_clock/reset_register calls instead of the
// PERIPH_RCC_ATOMIC() block macro -- this harness does not fake
// esp_private/periph_ctrl.h's RCC-atomic machinery (see driver clock gating
// in that file's real definition), only the header's existence.
// SOC_PERIPH_CLK_CTRL_SHARED is deliberately left undefined for the same
// reason (keeps the controller-clock block on the plain i2c_ll_* call path).
#define SOC_RCC_IS_INDEPENDENT 1
