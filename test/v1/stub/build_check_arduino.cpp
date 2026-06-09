// SPDX-License-Identifier: MIT
// Build check stub for v1 Arduino-based ESP32 envs.
// Provides setup() / loop() so the firmware links during build check.
#include <Arduino.h>

#include "../build_check/build_check.hpp"

void setup()
{
    m5hal_build_check::v1::compileApiSurface();
}
void loop()
{
}
