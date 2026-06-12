// SPDX-License-Identifier: MIT
// clang-format off
//
// Software variant: bit-banged HAL implementations on top of the
// abstract pin interface. Always available; flat-injected as a fallback
// when no hardware variant offers the same HAL kind.
//
// No include guard / #pragma once: M5HAL_v1.hpp re-includes this during
// variant scanning. Consumption rules: _macro/offer_all.inl.

#define M5HAL_VARIANT_CURRENT_ALIAS_   software
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::frameworks::software
#define M5HAL_VARIANT_CURRENT_ID_      M5HAL_V1_VARIANT_ID_FRAMEWORK_SOFTWARE
#define M5HAL_VARIANT_CURRENT_HAS_HAL_I2C_ 1
#define M5HAL_VARIANT_CURRENT_HAS_HAL_SPI_ 1
// clang-format on
