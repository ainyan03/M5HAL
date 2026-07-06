// SPDX-License-Identifier: MIT
// Build check stub for ESP-IDF based ESP32 envs.
// Provides app_main() so the firmware links during build check,
// and includes <M5HAL_v0.hpp> to ensure the public header compiles on
// ESP-IDF targets without errors. Not executed.
#include <M5HAL_v0.hpp>

extern "C" void app_main(void)
{
}
