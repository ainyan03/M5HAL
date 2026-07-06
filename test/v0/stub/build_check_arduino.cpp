// SPDX-License-Identifier: MIT
// Build check stub for Arduino-based ESP32 envs.
// Provides setup() / loop() so the firmware links during build check,
// and includes <M5HAL_v0.hpp> to ensure the public header compiles on
// Arduino targets without errors. Not executed.
#include <Arduino.h>
#include <M5HAL_v0.hpp>

void setup()
{
}
void loop()
{
}
