// SPDX-License-Identifier: MIT
//
// Compile and link the pinned M5UnitUnified source against M5HAL's explicit
// v0 compatibility entry without modifying the consumer checkout.
#include <Arduino.h>

#include <M5UnitUnified.hpp>

#include <type_traits>

static_assert(std::is_same<m5::hal::bus::Bus, m5::hal::v0::bus::Bus>::value,
              "M5UnitUnified's unqualified M5HAL bus type must resolve to v0");
static_assert(std::is_same<m5::hal::error::error_t, m5::hal::v0::error::error_t>::value,
              "M5UnitUnified's unqualified M5HAL error type must resolve to v0");

void setup()
{
    m5::unit::UnitUnified units;
    const auto info = units.debugInfo();
    (void)info;
}

void loop()
{
}
