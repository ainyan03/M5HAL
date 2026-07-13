// SPDX-License-Identifier: MIT
// Companion to test_master_clock_limit: verifies the ceiling-DISABLED path.
//
// No M5HAL_I2C_MASTER_MAX_CLOCK_HZ is defined here, so on a non-ESP (native)
// build the header default is 0 = no ceiling -- the behavior a non-ESP Arduino
// port or the software bit-bang backend gets. Any frequency must pass through
// untouched. (Separate executable from the enabled test: the clamp function is a
// header inline whose body depends on the macro, so the two macro values cannot
// share one program without an ODR clash.)

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include <cstdint>

namespace {

using m5::hal::v2::i2c::clampMasterClockHz;

TEST(MasterClockLimitDisabled, CeilingIsZeroOnNonEspNativeBuild)
{
    EXPECT_EQ(M5HAL_I2C_MASTER_MAX_CLOCK_HZ, 0u);
}

TEST(MasterClockLimitDisabled, AnyFrequencyPassesThrough)
{
    bool clamped = true;
    EXPECT_EQ(clampMasterClockHz(2000000u, &clamped), 2000000u);
    EXPECT_FALSE(clamped);

    clamped = true;
    EXPECT_EQ(clampMasterClockHz(5000000u, &clamped), 5000000u);
    EXPECT_FALSE(clamped);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
