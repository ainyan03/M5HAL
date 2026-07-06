// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>

// Phase-3 pool header is not yet wired into the M5_Hal umbrella (that lands
// with the BusView in a later commit); include it directly for this unit.
#include <m5_hal/hal/v2/bus/hw_pool.hpp>

// bus::HwControllerPool (ADR 034 phase 3) — the per-kind silicon budget.
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

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
