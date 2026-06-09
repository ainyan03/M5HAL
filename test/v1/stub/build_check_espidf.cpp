// SPDX-License-Identifier: MIT
// Build check stub for v1 ESP-IDF based ESP32 envs.
// Provides app_main() so the firmware links during build check.
#include "../build_check/build_check.hpp"

extern "C" void app_main(void)
{
    m5hal_build_check::v1::compileApiSurface();
}
