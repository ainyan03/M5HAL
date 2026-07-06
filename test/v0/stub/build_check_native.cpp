// SPDX-License-Identifier: MIT
// Build check stub for the native (host) env.
// Verifies that <M5HAL_v0.hpp> compiles on the host toolchain and
// provides main() so PlatformIO's native platform can produce an
// executable for check_native. Not executed.
//
// main() is gated by PIO_UNIT_TESTING so test_native's gtest main wins
// when this TU happens to be picked up by a test build.
#include <M5HAL_v0.hpp>

#ifndef PIO_UNIT_TESTING
int main()
{
    return 0;
}
#endif
