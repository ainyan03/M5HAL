// SPDX-License-Identifier: MIT
#include "../../build_check/build_check.hpp"

#include <gtest/gtest.h>

TEST(BuildCheck, CommonApiSurfaceRunsOnDummyBuses)
{
    m5hal_build_check::v1::compileApiSurface();
    SUCCEED();
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
