// SPDX-License-Identifier: MIT
// clang-format off
// Capability self-declaration for the POSIX host framework variant.
//
// No include guard / #pragma once: M5HAL_v1.hpp re-includes this during
// variant scanning, and offer_all.inl consumes + undefs the
// M5HAL_VARIANT_CURRENT_*_ macros below on each pass.
//
// clang-format off / on keeps the aligned #define table as-is.
//
// This variant offers UART only (host serial via termios). It is active
// solely on a POSIX host build; see frameworks/_checker.hpp
// (M5HAL_FRAMEWORK_HAS_POSIX) for the gating and opt-out flag.

#define M5HAL_VARIANT_CURRENT_ALIAS_   posix
#define M5HAL_VARIANT_CURRENT_BASE_NS_ variants::frameworks::posix
#define M5HAL_VARIANT_CURRENT_ID_      M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX

#define M5HAL_VARIANT_CURRENT_HAS_HAL_UART_ 1
// clang-format on
