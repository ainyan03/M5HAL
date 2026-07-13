// SPDX-License-Identifier: MIT
// Shared build-check stub for v2 Arduino envs that need no target assertion.
// Provides setup() / loop() so the firmware links during build check.
#include <Arduino.h>

#include "../build_check/build_check.hpp"

void setup()
{
    m5hal_build_check::v2::compileApiSurface();
}
void loop()
{
}
