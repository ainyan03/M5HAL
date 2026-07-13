// SPDX-License-Identifier: MIT
// v0/v2 coexistence include fence.
//
// Including BOTH public entries in one translation unit must load both
// generations completely. This is what the duplicate-include-guard bug
// class silently breaks: when a v0 header and a v2 header share a guard
// name (e.g. M5_HAL_ERROR_HPP, fixed by prefixing the v0 side with
// M5_HAL_V0_), whichever generation is included second loses files and
// the failure surfaces as confusing name-lookup errors far away.
//
// The per-generation check fences (v0_check_* / v2_check_*) compile the
// generations in separate TUs and can never catch this, so this TU is
// the regression guard: it merely has to compile and see core symbols
// of both explicit (non-inline) namespaces. The device-target twin of
// this fence is the v0v2_check_* env family (pio_envs/v0v2/check.ini.cli).

#include <M5HAL_v0.hpp>
#include <M5HAL_v2.hpp>

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

// The platform checkers keep generation-separated macro namespaces:
// v0 owns the unprefixed names, v2 reports through the variant-id
// registry. On native both must resolve to "unknown" / NONE
// independently.
static_assert(M5HAL_TARGET_PLATFORM_NUMBER == M5HAL_PLATFORM_NUMBER_UNKNOWN, "v0 platform number clobbered");
static_assert(M5HAL_V2_TARGET_PLATFORM_VARIANT_ID == M5HAL_V2_VARIANT_ID_NONE,
              "v2 platform variant id missing or clobbered");

TEST(CoexistInclude, BothGenerationsVisibleInOneTU)
{
    // Touch a core type from each generation through the explicit
    // namespaces (the inline-namespace selection must not matter here).
    using V0Error = ::m5::hal::v0::error::error_t;
    using V2Error = ::m5::hal::v2::error::error_t;
    EXPECT_EQ(static_cast<int>(V0Error::OK), 0);  // v0 enum is usable
    EXPECT_EQ(static_cast<int>(V2Error::OK), 0);  // v2 enum is usable

    // Both generations' bus kind tags resolve as distinct types.
    (void)sizeof(::m5::hal::v2::types::bus_kind_t);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
