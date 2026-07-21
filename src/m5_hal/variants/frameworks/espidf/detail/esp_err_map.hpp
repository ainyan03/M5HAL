// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_DETAIL_ESP_ERR_MAP_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_DETAIL_ESP_ERR_MAP_HPP

#include "../../../../hal/v2/error.hpp"

#include <esp_err.h>

namespace m5::variants::frameworks::espidf::detail {

/*!
  @brief Shared core of the `esp_err_t` -> `error_t` mapping used by the
         espidf backends' `mapEspErr()` wrappers.

  The cases below are the ones every backend agrees on. Everything
  driver-specific (the I2C NACK codes, `ESP_ERR_NOT_FOUND`, the SPI-slave
  `INVALID_STATE` split) stays in the per-backend wrapper as an explicit
  pre-switch or parameter, so a divergence is a visible decision at the
  wrapper instead of a silently drifted copy of this switch.

  @param fallback       Result for any code not listed (per peripheral:
                        `IO_ERROR` for stream-like drivers, `I2C_BUS_ERROR`
                        for the I2C masters).
  @param invalid_state  Result for `ESP_ERR_INVALID_STATE`. Most backends
                        fold it into `INVALID_ARGUMENT` (both are caller
                        bugs from the HAL user's point of view); a wrapper
                        that distinguishes driver-state conflicts passes
                        `INVALID_STATE` explicitly.
 */
constexpr ::m5::hal::v2::error::error_t mapEspErrCommon(
    ::esp_err_t err, ::m5::hal::v2::error::error_t fallback,
    ::m5::hal::v2::error::error_t invalid_state = ::m5::hal::v2::error::error_t::INVALID_ARGUMENT)
{
    using error_t = ::m5::hal::v2::error::error_t;
    switch (err) {
        case ESP_OK:
            return error_t::OK;
        case ESP_ERR_INVALID_ARG:
            return error_t::INVALID_ARGUMENT;
        case ESP_ERR_INVALID_STATE:
            return invalid_state;
        case ESP_ERR_TIMEOUT:
            return error_t::TIMEOUT_ERROR;
        case ESP_ERR_NO_MEM:
            return error_t::OUT_OF_RESOURCE;
        default:
            return fallback;
    }
}

}  // namespace m5::variants::frameworks::espidf::detail

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_DETAIL_ESP_ERR_MAP_HPP
