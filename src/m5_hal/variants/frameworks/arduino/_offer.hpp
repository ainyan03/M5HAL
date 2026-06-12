// clang-format off
// Capability self-declaration for the Arduino framework variant.
//
// No include guard / #pragma once: M5HAL_v1.hpp re-includes this during
// variant scanning, and offer_all.inl consumes + undefs the
// M5HAL_VARIANT_CURRENT_*_ macros below on each pass.
//
// clang-format off / on keeps the aligned #define table as-is.

#define M5HAL_VARIANT_CURRENT_ALIAS_   arduino
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::frameworks::arduino
#define M5HAL_VARIANT_CURRENT_ID_      M5HAL_V1_VARIANT_ID_FRAMEWORK_ARDUINO

#define M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_ 1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_  1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_  1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_UART_ 1
// clang-format on
