// SPDX-License-Identifier: MIT
#include "../../build_check/build_check.hpp"

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

TEST(BuildCheck, CommonApiSurfaceRunsOnDummyBuses)
{
    m5hal_build_check::v2::compileApiSurface();
    SUCCEED();
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
