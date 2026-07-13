// SPDX-License-Identifier: MIT
// Unit test for the I2C master clock fail-safe ceiling (clampMasterClockHz).
//
// Define a known ceiling BEFORE including the header so its `#ifndef` guard picks
// it up (the native default is 0 = disabled; that path is covered by the sibling
// test_master_clock_limit_disabled). The clamp function is a header-only inline
// whose body depends on this macro, so the enabled and disabled cases live in
// separate test executables to avoid an ODR clash on the inline definition.
#define M5HAL_CONFIG_I2C_MASTER_MAX_CLOCK_HZ 1200000u

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include <cstdint>

namespace {

using m5::hal::v2::i2c::clampMasterClockHz;

constexpr std::uint32_t kCeiling = 1200000u;

TEST(MasterClockLimit, BelowCeilingIsUnchanged)
{
    bool clamped = true;
    EXPECT_EQ(clampMasterClockHz(100000u, &clamped), 100000u);
    EXPECT_FALSE(clamped);

    clamped = true;
    EXPECT_EQ(clampMasterClockHz(400000u, &clamped), 400000u);
    EXPECT_FALSE(clamped);
}

TEST(MasterClockLimit, AtCeilingIsUnchanged)
{
    bool clamped = true;
    EXPECT_EQ(clampMasterClockHz(kCeiling, &clamped), kCeiling);
    EXPECT_FALSE(clamped);
}

TEST(MasterClockLimit, AboveCeilingIsClamped)
{
    bool clamped = false;
    EXPECT_EQ(clampMasterClockHz(kCeiling + 1u, &clamped), kCeiling);
    EXPECT_TRUE(clamped);

    clamped = false;
    EXPECT_EQ(clampMasterClockHz(2000000u, &clamped), kCeiling);
    EXPECT_TRUE(clamped);
}

TEST(MasterClockLimit, NullDidClampFlagIsAccepted)
{
    EXPECT_EQ(clampMasterClockHz(2000000u, nullptr), kCeiling);
    EXPECT_EQ(clampMasterClockHz(100000u, nullptr), 100000u);
}

TEST(MasterClockLimit, ZeroFrequencyIsUnchanged)
{
    bool clamped = true;
    EXPECT_EQ(clampMasterClockHz(0u, &clamped), 0u);
    EXPECT_FALSE(clamped);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
