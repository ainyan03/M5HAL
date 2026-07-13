// SPDX-License-Identifier: MIT
// clang-format off
// Capability self-declaration for the remote framework variant.
//
// No include guard / #pragma once: re-included during variant scanning.
// Offers proxy buses for I2C, SPI, UART, and I2S that forward operations
// to a peer MCU via RemoteSession (mux transport). Opt-in via
// M5HAL_CONFIG_REMOTE_VARIANT build flag; scanned after posix and before software.

#define M5HAL_VARIANT_CURRENT_ALIAS_   remote
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::frameworks::remote
#define M5HAL_VARIANT_CURRENT_ID_      M5HAL_V2_VARIANT_ID_FRAMEWORK_REMOTE

#define M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_  1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_  1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_UART_ 1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_I2S_  1
// clang-format on
