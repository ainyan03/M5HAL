// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_DETAIL_THREAD_CREATE_ERROR_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_DETAIL_THREAD_CREATE_ERROR_HPP_

#include <system_error>

#include "../../../hal/v2/error.hpp"

namespace m5::variants::frameworks::detail {

inline ::m5::hal::v2::error::error_t mapThreadCreateError(const std::error_code& code)
{
    const auto condition = code.default_error_condition();
    return condition == std::errc::resource_unavailable_try_again || condition == std::errc::not_enough_memory
               ? ::m5::hal::v2::error::error_t::OUT_OF_RESOURCE
               : ::m5::hal::v2::error::error_t::IO_ERROR;
}

}  // namespace m5::variants::frameworks::detail

#endif
