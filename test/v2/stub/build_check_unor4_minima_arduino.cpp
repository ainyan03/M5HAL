// SPDX-License-Identifier: MIT
// UNO R4 Minima / RA4M1 build fence. Pin both architecture and board so this
// does not accidentally widen the support claim to every Renesas Arduino.
#include <Arduino.h>

#if !defined(ARDUINO_ARCH_RENESAS_UNO) || !defined(ARDUINO_UNOR4_MINIMA) || !defined(ARDUINO_MINIMA)
#error "v2_check_unor4_minima_arduino requires Arduino UNO R4 Minima"
#endif

#include "../build_check/build_check.hpp"

void setup()
{
    m5hal_build_check::v2::compileApiSurface();
}
void loop()
{
}
