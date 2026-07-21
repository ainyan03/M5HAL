// SPDX-License-Identifier: MIT
// Compile-only fixture: appending another provider pass after the standard
// scan must not replace an existing ordered first-hit winner.
#include <M5HAL_v2.hpp>

static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX);
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_MUTEX == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX);
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_TASK == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX);
static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME_EVENT == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX);
static_assert(M5HAL_V2_SELECTED_VARIANT_GPIO == M5HAL_V2_VARIANT_ID_FRAMEWORK_STUB);
static_assert(M5HAL_V2_SELECTED_VARIANT_I2C == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE);
static_assert(M5HAL_V2_SELECTED_VARIANT_SPI == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE);
static_assert(M5HAL_V2_SELECTED_VARIANT_I2S == M5HAL_V2_VARIANT_ID_NONE);
static_assert(M5HAL_V2_SELECTED_VARIANT_PDM == M5HAL_V2_VARIANT_ID_NONE);
static_assert(M5HAL_V2_SELECTED_VARIANT_UART == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX);

// Synthetic scan-tail offer. The current winners must remain sealed.
#include "m5_hal/variants/frameworks/remote/_offer.hpp"
#include "m5_hal/_macro/offer_all.inl"

static_assert(M5HAL_V2_SELECTED_VARIANT_I2C == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE,
              "a provider appended to the scan must not replace the first hit");
static_assert(M5HAL_V2_SELECTED_VARIANT_SPI == M5HAL_V2_VARIANT_ID_FRAMEWORK_SOFTWARE,
              "a provider appended to the scan must not replace the first hit");
static_assert(M5HAL_V2_SELECTED_VARIANT_UART == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX,
              "a provider appended to the scan must not replace the first hit");
static_assert(M5HAL_V2_SELECTED_VARIANT_I2S == M5HAL_V2_VARIANT_ID_NONE,
              "the completed scan must seal an unoffered kind as NONE");
static_assert(M5HAL_V2_SELECTED_VARIANT_PDM == M5HAL_V2_VARIANT_ID_NONE,
              "the completed scan must seal an unoffered kind as NONE");
