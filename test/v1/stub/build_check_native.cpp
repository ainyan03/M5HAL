// SPDX-License-Identifier: MIT
// Build check stub for the v1 native (host) env.
// Provides main() so PlatformIO's native platform can produce an
// executable for check_v1_native.
//
// main() is gated by PIO_UNIT_TESTING so test_native's gtest main wins
// when this TU happens to be picked up by a test build.
#include "../build_check/build_check.hpp"

#ifndef PIO_UNIT_TESTING
int main()
{
    m5hal_build_check::v1::compileApiSurface();
    return 0;
}
#endif
