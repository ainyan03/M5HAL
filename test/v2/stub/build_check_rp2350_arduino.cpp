// SPDX-License-Identifier: MIT
// RP2350/Pico 2 ARM build fence. Keep the chip assertion next to the linked
// sketch so a board/core routing mistake cannot silently pass as RP2040.
#include <Arduino.h>

#if !defined(PICO_RP2350) || defined(PICO_RISCV) || defined(__riscv)
#error "v2_check_rp2350_arduino requires the RP2350 ARM build"
#endif

#include "../build_check/build_check.hpp"

void setup()
{
    m5hal_build_check::v2::compileApiSurface();
}
void loop()
{
}
