// SPDX-License-Identifier: MIT

#include "support/gtest_watchdog.hpp"

#include <gtest/gtest.h>
#include <m5_hal/variants/frameworks/espidf/hal/i2c/detail/slave_tx_ledger.hpp>

namespace {

namespace detail = m5::hal::v2::i2c::detail;

TEST(I2cSlaveTxLedger, PartialDrainKeepsOneByteUnconfirmed)
{
    detail::SlaveTxLedger<4> ledger;
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real, 4).has_value());

    auto first = ledger.observeFifoOccupancy(2);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->real, 1u);
    EXPECT_EQ(ledger.outstanding(), 3u);

    auto second = ledger.observeFifoOccupancy(1);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->real, 1u);
    EXPECT_EQ(ledger.confirmedTotals().real, 2u);
    EXPECT_EQ(ledger.outstanding(), 2u);
}

TEST(I2cSlaveTxLedger, OneByteReadConfirmsTheOnlyDeparture)
{
    detail::SlaveTxLedger<4> ledger;
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real).has_value());
    auto observed = ledger.observeFifoOccupancy(0);
    ASSERT_TRUE(observed.has_value());
    EXPECT_EQ(observed->real, 0u);

    auto stopped = ledger.confirmBoundaryAndDiscard(0, detail::SlaveTxBoundaryEvidence::ShifterAmbiguous);
    ASSERT_TRUE(stopped.has_value());
    EXPECT_EQ(stopped->confirmed.real, 1u);
    EXPECT_EQ(stopped->unclocked.real, 0u);
}

TEST(I2cSlaveTxLedger, OneByteEarlyStopLeavesFifoByteUnclocked)
{
    detail::SlaveTxLedger<4> ledger;
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real).has_value());

    auto stopped = ledger.confirmBoundaryAndDiscard(1, detail::SlaveTxBoundaryEvidence::ShifterAmbiguous);
    ASSERT_TRUE(stopped.has_value());
    EXPECT_EQ(stopped->confirmed.real, 0u);
    EXPECT_EQ(stopped->unclocked.real, 1u);
}

TEST(I2cSlaveTxLedger, MasterEarlyNackLeavesFifoSuffixUnclocked)
{
    detail::SlaveTxLedger<4> ledger;
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real, 3).has_value());

    auto stopped = ledger.confirmBoundaryAndDiscard(2, detail::SlaveTxBoundaryEvidence::ShifterAmbiguous);
    ASSERT_TRUE(stopped.has_value());
    EXPECT_EQ(stopped->confirmed.real, 1u);
    EXPECT_EQ(stopped->unclocked.real, 2u);
    EXPECT_EQ(ledger.outstanding(), 0u);
}

TEST(I2cSlaveTxLedger, MultiByteReadPreservesOnePrefetchedDeparture)
{
    detail::SlaveTxLedger<4> ledger;
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real, 4).has_value());

    auto stopped = ledger.confirmBoundaryAndDiscard(1, detail::SlaveTxBoundaryEvidence::ShifterAmbiguous);
    ASSERT_TRUE(stopped.has_value());
    EXPECT_EQ(stopped->confirmed.real, 2u);
    EXPECT_EQ(stopped->unclocked.real, 2u);
}

TEST(I2cSlaveTxLedger, PreservesRealFillAndLateRealOrder)
{
    detail::SlaveTxLedger<4> ledger;
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real).has_value());
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Fill).has_value());
    auto first = ledger.observeFifoOccupancy(0);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->real, 1u);
    EXPECT_EQ(first->fill, 0u);

    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real, 2).has_value());
    auto stopped = ledger.confirmBoundaryAndDiscard(0, detail::SlaveTxBoundaryEvidence::ShifterDrained);
    ASSERT_TRUE(stopped.has_value());
    EXPECT_EQ(stopped->confirmed.fill, 1u);
    EXPECT_EQ(stopped->confirmed.real, 2u);
    EXPECT_EQ(ledger.confirmedTotals().real, 3u);
    EXPECT_EQ(ledger.confirmedTotals().fill, 1u);
}

TEST(I2cSlaveTxLedger, LongRefillRetainsOnlyOneAmbiguousByte)
{
    detail::SlaveTxLedger<4> ledger;
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real, 4).has_value());
    auto first = ledger.observeFifoOccupancy(2);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->real, 1u);

    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real, 2).has_value());
    auto empty = ledger.observeFifoOccupancy(0);
    ASSERT_TRUE(empty.has_value());
    EXPECT_EQ(empty->real, 4u);
    EXPECT_EQ(ledger.outstanding(), 1u);

    auto tx_empty = ledger.confirmBoundaryAndDiscard(0, detail::SlaveTxBoundaryEvidence::ShifterDrained);
    ASSERT_TRUE(tx_empty.has_value());
    EXPECT_EQ(tx_empty->confirmed.real, 1u);
    EXPECT_EQ(ledger.confirmedTotals().real, 6u);
}

TEST(I2cSlaveTxLedger, RingWrapPreservesProvenance)
{
    detail::SlaveTxLedger<3> ledger;
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real, 3).has_value());
    auto first = ledger.observeFifoOccupancy(1);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->real, 1u);

    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Fill, 2).has_value());
    auto stopped = ledger.confirmBoundaryAndDiscard(0, detail::SlaveTxBoundaryEvidence::ShifterDrained);
    ASSERT_TRUE(stopped.has_value());
    EXPECT_EQ(stopped->confirmed.real, 2u);
    EXPECT_EQ(stopped->confirmed.fill, 2u);
}

TEST(I2cSlaveTxLedger, RejectsOverflowAndInvalidObservations)
{
    detail::SlaveTxLedger<2> ledger;
    EXPECT_EQ(ledger.load(static_cast<detail::SlaveTxProvenance>(0xFF)).error(),
              m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real, 2).has_value());
    EXPECT_EQ(ledger.load(detail::SlaveTxProvenance::Fill).error(), m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(ledger.observeFifoOccupancy(3).error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    ASSERT_TRUE(ledger.observeFifoOccupancy(1).has_value());
    EXPECT_EQ(ledger.observeFifoOccupancy(2).error(), m5::hal::v2::error::error_t::INVALID_STATE);
    EXPECT_EQ(ledger.confirmBoundaryAndDiscard(2, detail::SlaveTxBoundaryEvidence::ShifterAmbiguous).error(),
              m5::hal::v2::error::error_t::INVALID_STATE);
}

TEST(I2cSlaveTxLedger, ResetClearsOutstandingAndTotals)
{
    detail::SlaveTxLedger<2> ledger;
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Fill, 2).has_value());
    ASSERT_TRUE(ledger.confirmBoundaryAndDiscard(0, detail::SlaveTxBoundaryEvidence::ShifterDrained).has_value());
    ASSERT_EQ(ledger.confirmedTotals().fill, 2u);
    ledger.reset();
    EXPECT_EQ(ledger.outstanding(), 0u);
    EXPECT_EQ(ledger.fifoOccupancy(), 0u);
    EXPECT_EQ(ledger.confirmedTotals().real, 0u);
    EXPECT_EQ(ledger.confirmedTotals().fill, 0u);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
