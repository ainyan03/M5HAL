// SPDX-License-Identifier: MIT
// Official ESP-IDF component build check for M5HAL across the chip family
// (esp32 / s3 / c3 / c6 / h2 / p4). Reuses the shared v1 API-surface compile
// fence (which references PinBackup / ScopedPinBackup), so building this app
// compiles both M5HAL translation units plus the public v1 API. app_main is
// never executed in CI; a clean compile + link of the firmware is the check.
#include "../../v1/build_check/build_check.hpp"

extern "C" void app_main(void)
{
    m5hal_build_check::v1::compileApiSurface();
}
