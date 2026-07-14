// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_BEGIN_RESULT_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_I2C_BEGIN_RESULT_HPP

#include <type_traits>

namespace m5::hal::v2::i2c::wire_begin_detail {

// Arduino Wire cores disagree on the return type of TwoWire::begin():
// portable cores commonly return void, while Espressif cores return bool.
// A void-returning core has no failure signal; a value-returning core does.
template <typename Begin>
bool invokeWireBegin(Begin&& begin)
{
    if constexpr (std::is_void_v<decltype(begin())>) {
        begin();
        return true;
    } else {
        return static_cast<bool>(begin());
    }
}

}  // namespace m5::hal::v2::i2c::wire_begin_detail

#endif
