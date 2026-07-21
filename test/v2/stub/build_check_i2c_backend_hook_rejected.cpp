// SPDX-License-Identifier: MIT
// Expected compile failure. Concrete provider hooks are protected seams reached
// only through the checked i2c::IBus facade. The driver loops over every
// provider/hook combination visible in a plain host build (software, remote);
// arduino/espidf headers are not host-compilable, so their hook access is
// enforced structurally by .github/scripts/check-spec-api.py instead.
#include <M5HAL_v2.hpp>

#ifndef M5HAL_CHECK_PROVIDER
#define M5HAL_CHECK_PROVIDER Bus_software
#endif
#ifndef M5HAL_CHECK_HOOK
#define M5HAL_CHECK_HOOK transferBackend
#endif

int main()
{
    auto hook = &m5::hal::v2::i2c::M5HAL_CHECK_PROVIDER::M5HAL_CHECK_HOOK;
    (void)hook;
    return 0;
}
