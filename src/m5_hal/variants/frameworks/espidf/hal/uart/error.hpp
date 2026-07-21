// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_ERROR_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_ERROR_HPP

#include "../../../../../hal/v2/error.hpp"

#if defined(ESP_PLATFORM)

#include "../../detail/esp_err_map.hpp"

#include <esp_err.h>

namespace m5::hal::v2::uart::impl_espidf {

inline error::error_t mapEspErr(::esp_err_t err)
{
    return ::m5::variants::frameworks::espidf::detail::mapEspErrCommon(err, error::error_t::IO_ERROR);
}

}  // namespace m5::hal::v2::uart::impl_espidf

#endif  // ESP_PLATFORM

#endif
