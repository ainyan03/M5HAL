// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

// pool header is not yet wired into the M5_Hal umbrella (that lands
// with the BusView in a later commit); include it directly for this unit.
#include <m5_hal/hal/v2/bus/hw_pool.hpp>

// bus::HwControllerPool — the per-kind silicon budget.
// Pure lease bookkeeping over a bitmask; no hardware or I/O involved.

namespace {
namespace v2 = m5::hal::v2;
}  // namespace

TEST(HwControllerPool, AcquireUntilFullThenNone)
{
    v2::bus::HwControllerPool pool{2};
    EXPECT_EQ(pool.capacity(), 2u);
    EXPECT_EQ(pool.inUse(), 0u);
    EXPECT_EQ(pool.available(), 2u);

    int8_t a = pool.acquire();
    int8_t b = pool.acquire();
    EXPECT_EQ(a, 0);
    EXPECT_EQ(b, 1);
    EXPECT_EQ(pool.inUse(), 2u);
    EXPECT_EQ(pool.available(), 0u);

    // Full: the next acquire yields kNone.
    EXPECT_EQ(pool.acquire(), v2::bus::HwControllerPool::kNone);
    EXPECT_EQ(pool.inUse(), 2u);
}

TEST(HwControllerPool, ReleaseFreesAndReusesLowest)
{
    v2::bus::HwControllerPool pool{2};
    int8_t a = pool.acquire();  // 0
    int8_t b = pool.acquire();  // 1
    (void)b;
    pool.release(a);
    EXPECT_FALSE(pool.isLeased(a));
    EXPECT_EQ(pool.inUse(), 1u);
    // Lowest free index is reused.
    EXPECT_EQ(pool.acquire(), 0);
    EXPECT_EQ(pool.inUse(), 2u);
}

TEST(HwControllerPool, DoubleReleaseAndOutOfRangeAreHarmless)
{
    v2::bus::HwControllerPool pool{2};
    int8_t a = pool.acquire();  // 0
    pool.release(a);
    pool.release(a);   // double release -> no-op
    pool.release(5);   // out of range -> no-op
    pool.release(-1);  // kNone -> no-op
    EXPECT_EQ(pool.inUse(), 0u);
    EXPECT_EQ(pool.acquire(), 0);
}

TEST(HwControllerPool, AcquireSpecificForPinController)
{
    v2::bus::HwControllerPool pool{2};
    EXPECT_TRUE(pool.acquireSpecific(1));  // reserve controller 1
    EXPECT_TRUE(pool.isLeased(1));
    EXPECT_FALSE(pool.acquireSpecific(1));   // already leased
    EXPECT_FALSE(pool.acquireSpecific(2));   // out of range (capacity 2 -> ids 0,1)
    EXPECT_FALSE(pool.acquireSpecific(-1));  // kNone
    // Auto-acquire skips the reserved index and takes the lowest free one.
    EXPECT_EQ(pool.acquire(), 0);
    EXPECT_EQ(pool.acquire(), v2::bus::HwControllerPool::kNone);
}

TEST(HwControllerPool, ZeroCapacityAlwaysNone)
{
    v2::bus::HwControllerPool pool{0};  // software-only / host build
    EXPECT_EQ(pool.capacity(), 0u);
    EXPECT_EQ(pool.acquire(), v2::bus::HwControllerPool::kNone);
    EXPECT_FALSE(pool.acquireSpecific(0));
    EXPECT_EQ(pool.inUse(), 0u);
    EXPECT_EQ(pool.available(), 0u);
}

// --- External claim (a caller outside the intent resolver, e.g. a
// standalone slave) --------------------------------------------------------

TEST(HwControllerPool, ClaimExternalMarksLeasedAndExternal)
{
    v2::bus::HwControllerPool pool{2};
    EXPECT_TRUE(pool.claimExternal(0));
    EXPECT_TRUE(pool.isLeased(0));
    EXPECT_TRUE(pool.isExternal(0));
    EXPECT_FALSE(pool.isExternal(1));  // untouched index
    EXPECT_EQ(pool.inUse(), 1u);
}

TEST(HwControllerPool, ClaimExternalDoubleClaimFails)
{
    v2::bus::HwControllerPool pool{2};
    EXPECT_TRUE(pool.claimExternal(0));
    EXPECT_FALSE(pool.claimExternal(0));  // already claimed
    EXPECT_EQ(pool.inUse(), 1u);
}

TEST(HwControllerPool, AcquireAvoidsExternallyClaimedController)
{
    v2::bus::HwControllerPool pool{2};
    EXPECT_TRUE(pool.claimExternal(0));
    // acquireSpecific must also see the external claim as busy.
    EXPECT_FALSE(pool.acquireSpecific(0));
    // Auto-acquire skips the claimed index and takes the lowest free one.
    EXPECT_EQ(pool.acquire(), 1);
}

TEST(HwControllerPool, ReleaseAllPreservesExternalClaims)
{
    v2::bus::HwControllerPool pool{3};
    EXPECT_TRUE(pool.claimExternal(1));
    EXPECT_EQ(pool.acquire(), 0);  // ordinary lease on 0
    EXPECT_EQ(pool.acquire(), 2);  // ordinary lease on 2

    pool.releaseAll();  // the commit-time pool rebuild
    EXPECT_FALSE(pool.isLeased(0));
    EXPECT_FALSE(pool.isLeased(2));
    EXPECT_TRUE(pool.isLeased(1));  // external claim survives
    EXPECT_TRUE(pool.isExternal(1));
    EXPECT_EQ(pool.inUse(), 1u);
}

TEST(HwControllerPool, ReleaseExternalFreesController)
{
    v2::bus::HwControllerPool pool{2};
    EXPECT_TRUE(pool.claimExternal(0));
    EXPECT_TRUE(pool.releaseExternal(0));
    EXPECT_FALSE(pool.isLeased(0));
    EXPECT_FALSE(pool.isExternal(0));
    EXPECT_EQ(pool.acquire(), 0);  // free for ordinary lease again
}

TEST(HwControllerPool, ReleaseExternalOutOfRangeAndDoubleAreHarmless)
{
    v2::bus::HwControllerPool pool{2};
    EXPECT_FALSE(pool.releaseExternal(5));   // out of range -> no-op
    EXPECT_FALSE(pool.releaseExternal(-1));  // kNone -> no-op
    EXPECT_TRUE(pool.claimExternal(0));
    EXPECT_TRUE(pool.releaseExternal(0));
    EXPECT_FALSE(pool.releaseExternal(0));  // double release -> no-op
    EXPECT_FALSE(pool.isExternal(0));
    EXPECT_EQ(pool.inUse(), 0u);
}

TEST(HwControllerPool, OrdinaryReleaseDoesNotFreeExternalClaim)
{
    // Ownership classes release only through their own paths: a stray
    // resolver-side release() on an externally-claimed index must not free
    // it (the external holder is still on the controller).
    v2::bus::HwControllerPool pool{2};
    EXPECT_TRUE(pool.claimExternal(0));
    pool.release(0);
    EXPECT_TRUE(pool.isLeased(0));
    EXPECT_TRUE(pool.isExternal(0));
    EXPECT_FALSE(pool.acquireSpecific(0));  // still not up for grabs
}

TEST(HwControllerPool, ReleaseExternalDoesNotFreeOrdinaryLease)
{
    // The reverse direction: releaseExternal on an ORDINARY lease reports
    // failure and leaves the lease intact.
    v2::bus::HwControllerPool pool{2};
    EXPECT_TRUE(pool.acquireSpecific(0));
    EXPECT_FALSE(pool.releaseExternal(0));
    EXPECT_TRUE(pool.isLeased(0));
    EXPECT_FALSE(pool.isExternal(0));
}

TEST(HwControllerPool, IsExternalOutOfRangeIsFalse)
{
    v2::bus::HwControllerPool pool{2};
    EXPECT_FALSE(pool.isExternal(5));
    EXPECT_FALSE(pool.isExternal(-1));
}

TEST(HwControllerPool, ClaimExternalOutOfRangeFails)
{
    v2::bus::HwControllerPool pool{2};
    EXPECT_FALSE(pool.claimExternal(2));   // capacity 2 -> ids 0,1
    EXPECT_FALSE(pool.claimExternal(-1));  // kNone
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
