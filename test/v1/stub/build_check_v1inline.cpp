// SPDX-License-Identifier: MIT
// Build check for the M5HAL_V1_INLINE flip path.
//
// Used by check_native_v1inline env, which builds with
// -DM5HAL_V0_INLINE=0 -DM5HAL_V1_INLINE=1. Under that combination, the
// inline namespace forward declaration in M5HAL_v1.hpp should make
// m5::hal::error::error_t resolve to m5::hal::v1::error::error_t (= the
// "default = v1" deprecation-flip scenario).
//
// Also verifies the mutually-exclusive guard in m5_hal_config.hpp by
// relying on the macro defines having been validated at include time.
#include <M5HAL_v1.hpp>

#include <type_traits>

#if M5HAL_V1_INLINE
static_assert(std::is_same<m5::hal::error::error_t, m5::hal::v1::error::error_t>::value,
              "M5HAL_V1_INLINE=1 but m5::hal::error::error_t did not resolve to v1");
static_assert(std::is_same<m5::hal::result_t<void>, m5::hal::v1::result_t<void>>::value,
              "M5HAL_V1_INLINE=1 but m5::hal::result_t did not resolve to v1");
#endif

// Invariant: explicit v1 access always works, independent of which
// generation is inline.
static_assert(std::is_same<m5::hal::v1::error::error_t, m5::hal::v1::error::error_t>::value,
              "v1 explicit namespace access broken");

#ifndef PIO_UNIT_TESTING
int main()
{
    return 0;
}
#endif
