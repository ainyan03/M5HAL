// clang-format off
// Capability self-declaration for the Arduino framework variant.
//
// No include guard / #pragma once: M5HAL_v1.hpp re-includes this during
// variant scanning, and offer_all.inl consumes + undefs the
// M5HAL_VARIANT_CURRENT_*_ macros below on each pass.
//
// clang-format off / on guards protect the BASE_PATH_ value: bare path
// tokens (../variants/frameworks/arduino) would otherwise be reformatted
// as a chain of C++ division operators, breaking the macro.

#define M5HAL_VARIANT_CURRENT_ALIAS_     arduino
#define M5HAL_VARIANT_CURRENT_BASE_PATH_ ../variants/frameworks/arduino
#define M5HAL_VARIANT_CURRENT_BASE_NS_   variants::frameworks::arduino

#define M5HAL_VARIANT_CURRENT_HAS_HAL_GPIO_ 1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_  1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_  1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_UART_ 1
// clang-format on
