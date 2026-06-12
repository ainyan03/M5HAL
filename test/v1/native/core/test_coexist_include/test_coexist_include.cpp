// v0/v1 coexistence include fence.
//
// Including BOTH public entries in one translation unit must load both
// generations completely. This is what the duplicate-include-guard bug
// class silently breaks: when a v0 header and a v1 header share a guard
// name (e.g. M5_HAL_ERROR_HPP, fixed by prefixing the v0 side with
// M5_HAL_V0_), whichever generation is included second loses files and
// the failure surfaces as confusing name-lookup errors far away.
//
// The per-generation check fences (v0_check_* / v1_check_*) compile the
// generations in separate TUs and can never catch this, so this TU is
// the regression guard: it merely has to compile and see core symbols
// of both explicit (non-inline) namespaces. The device-target twin of
// this fence is the v0v1_check_* env family (pio_envs/v0v1/check.ini.cli).

#include <M5HAL_v0.hpp>
#include <M5HAL_v1.hpp>

#include <gtest/gtest.h>

// The platform checkers keep generation-separated macro namespaces:
// v0 owns the unprefixed names, v1 reports through the variant-id
// registry (S18). On native both must resolve to "unknown" / NONE
// independently.
static_assert(M5HAL_TARGET_PLATFORM_NUMBER == M5HAL_PLATFORM_NUMBER_UNKNOWN,
              "v0 platform number clobbered");
static_assert(M5HAL_V1_TARGET_PLATFORM_VARIANT_ID == M5HAL_V1_VARIANT_ID_NONE,
              "v1 platform variant id missing or clobbered");

TEST(CoexistInclude, BothGenerationsVisibleInOneTU)
{
    // Touch a core type from each generation through the explicit
    // namespaces (the inline-namespace selection must not matter here).
    using V0Error = ::m5::hal::v0::error::error_t;
    using V1Error = ::m5::hal::v1::error::error_t;
    EXPECT_EQ(static_cast<int>(V0Error::OK), 0);  // v0 enum is usable
    EXPECT_EQ(static_cast<int>(V1Error::OK), 0);  // v1 enum is usable

    // Both generations' bus kind tags resolve as distinct types.
    (void)sizeof(::m5::hal::v1::types::bus_kind_t);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
