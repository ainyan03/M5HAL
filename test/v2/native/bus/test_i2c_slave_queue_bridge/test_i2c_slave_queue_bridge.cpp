// SPDX-License-Identifier: MIT

#include "support/gtest_watchdog.hpp"

#include <gtest/gtest.h>
#include <m5_hal/variants/frameworks/espidf/hal/i2c/detail/slave_queue_bridge.hpp>
#include <m5_hal/variants/frameworks/espidf/hal/i2c/detail/slave_tx_ledger.hpp>

#include <array>
#include <cstdint>
#include <memory>

namespace {

namespace detail = m5::hal::v2::i2c::detail;
using Bridge     = detail::SlaveQueueBridge<8, 8, 8>;
using Result     = detail::SlaveQueueBridgeResult;

template <class T>
detail::SlaveQueueRawEvent takeEvent(T& bridge, uint32_t generation, uint8_t* rx = nullptr, size_t capacity = 0)
{
    auto worker = bridge.workerBeginSession(generation);
    EXPECT_EQ(worker.status(), Result::Accepted);
    detail::SlaveQueueRawEvent event;
    EXPECT_EQ(bridge.workerPeekStep(worker, event, rx, capacity), detail::SlaveQueueWorkerStep::Ready);
    EXPECT_EQ(bridge.workerCommitStep(worker), Result::Accepted);
    return event;
}

template <class T>
Result stageTx(T& bridge, uint32_t generation, const uint8_t* data, size_t count)
{
    auto worker = bridge.workerBeginSession(generation);
    if (worker.status() != Result::Accepted) return worker.status();
    return bridge.workerStageTx(worker, data, count);
}

template <class Prepared>
void fakeRead(const uint8_t* source, Prepared& prepared, size_t& hardware_reads)
{
    for (size_t i = 0; i < prepared.first.size; ++i) prepared.first.data[i] = source[i];
    for (size_t i = 0; i < prepared.second.size; ++i) prepared.second.data[i] = source[prepared.first.size + i];
    ++hardware_reads;
}

TEST(I2cSlaveQueueBridge, RejectsZeroGeneration)
{
    Bridge bridge;
    EXPECT_FALSE(bridge.workerBegin(0, 0xFF));
    EXPECT_EQ(bridge.workerState(), detail::SlaveQueueBridgeState::Idle);
}

TEST(I2cSlaveQueueBridge, RxPrepareExposesWrappedSpansAndCommitPublishes)
{
    detail::SlaveQueueBridge<8, 4, 4> bridge;
    ASSERT_TRUE(bridge.workerBegin(1, 0xFF));
    size_t reads = 0;
    uint8_t output[4]{};

    {
        auto session          = bridge.isrBegin(1);
        const uint8_t bytes[] = {1, 2, 3};
        auto prepared         = bridge.prepareRx(session, 3);
        ASSERT_EQ(prepared.status, Result::Accepted);
        fakeRead(bytes, prepared, reads);
        ASSERT_EQ(bridge.commitRx(session), Result::Accepted);
    }
    takeEvent(bridge, 1, output, sizeof(output));

    {
        auto session          = bridge.isrBegin(1);
        const uint8_t bytes[] = {4, 5, 6};
        auto prepared         = bridge.prepareRx(session, 3);
        ASSERT_EQ(prepared.status, Result::Accepted);
        EXPECT_EQ(prepared.first.size, 1u);
        EXPECT_EQ(prepared.second.size, 2u);
        fakeRead(bytes, prepared, reads);
        ASSERT_EQ(bridge.commitRx(session), Result::Accepted);
    }
    const auto event = takeEvent(bridge, 1, output, sizeof(output));
    EXPECT_EQ(event.sequence, 3u);
    EXPECT_EQ((std::array<uint8_t, 3>{output[0], output[1], output[2]}), (std::array<uint8_t, 3>{4, 5, 6}));
    EXPECT_EQ(reads, 2u);
}

TEST(I2cSlaveQueueBridge, RxNoSpaceLeavesHardwareFifoUnreadAndRetries)
{
    detail::SlaveQueueBridge<4, 2, 4> bridge;
    ASSERT_TRUE(bridge.workerBegin(1, 0xFF));
    size_t reads          = 0;
    const uint8_t bytes[] = {1, 2};
    {
        auto session  = bridge.isrBegin(1);
        auto prepared = bridge.prepareRx(session, 2);
        ASSERT_EQ(prepared.status, Result::Accepted);
        fakeRead(bytes, prepared, reads);
        ASSERT_EQ(bridge.commitRx(session), Result::Accepted);
    }
    {
        auto session  = bridge.isrBegin(1);
        auto no_space = bridge.prepareRx(session, 1);
        EXPECT_EQ(no_space.status, Result::NoSpace);
        EXPECT_EQ(reads, 1u);
        EXPECT_EQ(bridge.cancel(session), Result::Rejected);
    }
    uint8_t output[2]{};
    takeEvent(bridge, 1, output, sizeof(output));
    {
        auto session = bridge.isrBegin(1);
        auto retry   = bridge.prepareRx(session, 1);
        EXPECT_EQ(retry.status, Result::Accepted);
        bridge.cancel(session);
    }
    EXPECT_FALSE(bridge.broken());
}

TEST(I2cSlaveQueueBridge, RxCancelPublishesNothing)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(1, 0xFF));
    auto session  = bridge.isrBegin(1);
    auto prepared = bridge.prepareRx(session, 2);
    ASSERT_EQ(prepared.status, Result::Accepted);
    prepared.first.data[0] = 1;
    ASSERT_EQ(bridge.cancel(session), Result::Accepted);
    detail::SlaveQueueRawEvent event;
    auto worker = bridge.workerBeginSession(1);
    EXPECT_EQ(bridge.workerPeekStep(worker, event), detail::SlaveQueueWorkerStep::Empty);
}

TEST(I2cSlaveQueueBridge, TxCancelLeavesCacheEventAndLedgerUnchanged)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(2, 0xEE));
    const uint8_t bytes[] = {0x10, 0x11};
    ASSERT_EQ(stageTx(bridge, 2, bytes, 2), Result::Accepted);
    uint8_t output[2]{};
    {
        auto session  = bridge.isrBegin(2);
        auto prepared = bridge.prepareTx(session, output, 2);
        ASSERT_EQ(prepared.status, Result::Accepted);
        ASSERT_EQ(bridge.cancel(session), Result::Accepted);
    }
    detail::SlaveQueueRawEvent event;
    auto worker = bridge.workerBeginSession(2);
    EXPECT_EQ(bridge.workerPeekStep(worker, event), detail::SlaveQueueWorkerStep::Empty);
    EXPECT_FALSE(bridge.boundaryRequired());
    EXPECT_EQ(bridge.workerTxRetained(), 2u);
    {
        auto session = bridge.isrBegin(2);
        auto retry   = bridge.prepareTx(session, output, 2);
        ASSERT_EQ(retry.status, Result::Accepted);
        EXPECT_EQ(output[0], 0x10);
        EXPECT_EQ(output[1], 0x11);
        EXPECT_EQ(bridge.cancel(session), Result::Accepted);
    }
    EXPECT_FALSE(bridge.broken());
}

TEST(I2cSlaveQueueBridge, DroppingPendingSessionIsBroken)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(12, 0xFF));
    {
        auto session = bridge.isrBegin(12);
        ASSERT_EQ(bridge.prepareRx(session, 1).status, Result::Accepted);
    }
    EXPECT_TRUE(bridge.broken());
}

TEST(I2cSlaveQueueBridge, TxCommitPublishesRealThenFillAndRequiresBoundary)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(3, 0xA5));
    const uint8_t byte = 0x20;
    ASSERT_EQ(stageTx(bridge, 3, &byte, 1), Result::Accepted);
    uint8_t output[2]{};
    {
        auto session  = bridge.isrBegin(3);
        auto prepared = bridge.prepareTx(session, output, 2);
        ASSERT_EQ(prepared.status, Result::Accepted);
        EXPECT_EQ(prepared.real, 1u);
        EXPECT_EQ(prepared.fill, 1u);
        ASSERT_EQ(bridge.commitTxLoaded(session), Result::Accepted);
    }
    EXPECT_TRUE(bridge.boundaryRequired());
    EXPECT_EQ(takeEvent(bridge, 3).kind, detail::SlaveQueueRawEventKind::TxLoadedReal);
    EXPECT_EQ(takeEvent(bridge, 3).kind, detail::SlaveQueueRawEventKind::TxLoadedFill);
}

TEST(I2cSlaveQueueBridge, BoundaryUsesReservedEventSlot)
{
    detail::SlaveQueueBridge<2, 4, 4> bridge;
    ASSERT_TRUE(bridge.workerBegin(4, 0xFF));
    auto session = bridge.isrBegin(4);
    ASSERT_EQ(bridge.observeFifo(session, 1), Result::Accepted);
    EXPECT_EQ(bridge.observeFifo(session, 0), Result::NoSpace);
    EXPECT_EQ(bridge.stopBoundary(session, 0), Result::Accepted);
    EXPECT_EQ(bridge.txEmptyBoundary(session, 0), Result::Rejected);
    EXPECT_FALSE(bridge.broken());
}

TEST(I2cSlaveQueueBridge, LiveSessionBlocksQuiescence)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(5, 0xFF));
    {
        auto session = bridge.isrBegin(5);
        ASSERT_EQ(session.status(), Result::Accepted);
        ASSERT_TRUE(bridge.workerRequestClose(5));
        EXPECT_FALSE(bridge.workerTryQuiesce(5));
    }
    EXPECT_TRUE(bridge.workerTryQuiesce(5));
}

TEST(I2cSlaveQueueBridge, TxLoadedWithoutBoundaryBlocksQuiescence)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(6, 0xFF));
    uint8_t output{};
    {
        auto session = bridge.isrBegin(6);
        ASSERT_EQ(bridge.prepareTx(session, &output, 1).status, Result::Accepted);
        ASSERT_EQ(bridge.commitTxLoaded(session), Result::Accepted);
    }
    ASSERT_TRUE(bridge.workerRequestClose(6));
    takeEvent(bridge, 6);
    EXPECT_TRUE(bridge.boundaryRequired());
    EXPECT_FALSE(bridge.workerTryQuiesce(6));
}

TEST(I2cSlaveQueueBridge, BoundaryMustBeReplayedAndResolvedBeforeQuiescence)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(7, 0xFF));
    {
        auto session = bridge.isrBegin(7);
        ASSERT_EQ(bridge.stopBoundary(session, 0), Result::Accepted);
    }
    ASSERT_TRUE(bridge.workerRequestClose(7));
    {
        auto worker = bridge.workerBeginSession(7);
        detail::SlaveQueueRawEvent event;
        ASSERT_EQ(bridge.workerPeekStep(worker, event), detail::SlaveQueueWorkerStep::Ready);
        EXPECT_FALSE(bridge.workerTryQuiesce(7));
        ASSERT_EQ(bridge.workerCommitStep(worker), Result::Accepted);
        EXPECT_FALSE(bridge.workerTryQuiesce(7));
        ASSERT_EQ(bridge.workerResolveTxBoundary(worker, 0, 0), Result::Accepted);
    }
    EXPECT_TRUE(bridge.workerTryQuiesce(7));
}

TEST(I2cSlaveQueueBridge, StopReplayRetainsUnclockedRealSuffix)
{
    Bridge bridge;
    detail::SlaveTxLedger<4> ledger;
    ASSERT_TRUE(bridge.workerBegin(8, 0xFF));
    const uint8_t bytes[] = {0x30, 0x31, 0x32};
    ASSERT_EQ(stageTx(bridge, 8, bytes, 3), Result::Accepted);
    uint8_t output[3]{};
    {
        auto session = bridge.isrBegin(8);
        ASSERT_EQ(bridge.prepareTx(session, output, 3).status, Result::Accepted);
        ASSERT_EQ(bridge.commitTxLoaded(session), Result::Accepted);
        ASSERT_EQ(bridge.stopBoundary(session, 2), Result::Accepted);
    }
    auto event = takeEvent(bridge, 8);
    ASSERT_TRUE(ledger.load(detail::SlaveTxProvenance::Real, event.count).has_value());
    auto worker = bridge.workerBeginSession(8);
    detail::SlaveQueueRawEvent boundary;
    ASSERT_EQ(bridge.workerPeekStep(worker, boundary), detail::SlaveQueueWorkerStep::Ready);
    auto stopped = ledger.confirmBoundaryAndDiscard(boundary.value, detail::SlaveTxBoundaryEvidence::ShifterAmbiguous);
    ASSERT_TRUE(stopped.has_value());
    ASSERT_EQ(bridge.workerCommitStep(worker), Result::Accepted);
    ASSERT_EQ(bridge.workerResolveTxBoundary(worker, static_cast<uint32_t>(stopped->confirmed.real),
                                             static_cast<uint32_t>(stopped->unclocked.real)),
              Result::Accepted);
    EXPECT_EQ(bridge.workerTxRetained(), 2u);
}

TEST(I2cSlaveQueueBridge, BrokenMustCloseQuiesceAndResetBeforeNewBegin)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(9, 0xFF));
    {
        auto session = bridge.isrBegin(9);
        EXPECT_EQ(bridge.prepareRx(session, 0).status, Result::Broken);
    }
    EXPECT_TRUE(bridge.broken());
    EXPECT_FALSE(bridge.workerBegin(10, 0xFF));
    ASSERT_TRUE(bridge.workerRequestClose(9));
    ASSERT_TRUE(bridge.workerTryQuiesce(9));
    EXPECT_TRUE(bridge.broken());
    EXPECT_FALSE(bridge.workerBegin(10, 0xFF));
    ASSERT_TRUE(bridge.workerReset());
    EXPECT_FALSE(bridge.broken());
    EXPECT_TRUE(bridge.workerBegin(10, 0xFF));
}

TEST(I2cSlaveQueueBridge, StaleGenerationSessionIsRejectedWithoutPoisoning)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(11, 0xFF));
    auto stale = bridge.isrBegin(10);
    EXPECT_EQ(stale.status(), Result::Rejected);
    EXPECT_FALSE(bridge.broken());
}

TEST(I2cSlaveQueueBridge, ForcedQuiesceWaitsForSessionAndPreservesBrokenUntilReset)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(13, 0xFF));
    {
        auto session = bridge.isrBegin(13);
        ASSERT_TRUE(bridge.workerRequestClose(13));
        EXPECT_FALSE(bridge.workerForceQuiesce(13));
    }
    EXPECT_TRUE(bridge.workerForceQuiesce(13));
    EXPECT_EQ(bridge.workerState(), detail::SlaveQueueBridgeState::Quiesced);
    EXPECT_TRUE(bridge.broken());
    EXPECT_FALSE(bridge.workerBegin(14, 0xFF));
    ASSERT_TRUE(bridge.workerReset());
    EXPECT_TRUE(bridge.workerBegin(14, 0xFF));
}

TEST(I2cSlaveQueueBridge, ForceClosesWorkerGateButLetsLiveIterationCommit)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(15, 0xFF));
    {
        auto isr = bridge.isrBegin(15);
        ASSERT_EQ(bridge.observeFifo(isr, 0), Result::Accepted);
    }
    ASSERT_TRUE(bridge.workerRequestClose(15));
    {
        auto worker = bridge.workerBeginSession(15);
        ASSERT_EQ(worker.status(), Result::Accepted);
        detail::SlaveQueueRawEvent event;
        ASSERT_EQ(bridge.workerPeekStep(worker, event), detail::SlaveQueueWorkerStep::Ready);
        EXPECT_FALSE(bridge.workerForceQuiesce(15));
        auto rejected = bridge.workerBeginSession(15);
        EXPECT_EQ(rejected.status(), Result::Rejected);
        EXPECT_EQ(bridge.workerPeekStep(rejected, event), detail::SlaveQueueWorkerStep::Rejected);
        EXPECT_EQ(bridge.workerCommitStep(worker), Result::Accepted);
    }
    EXPECT_TRUE(bridge.workerForceQuiesce(15));
    EXPECT_EQ(bridge.workerState(), detail::SlaveQueueBridgeState::Quiesced);
}

TEST(I2cSlaveQueueBridge, AbandonRejectsWrongStateGenerationAndLiveIsr)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(17, 0xA5));
    EXPECT_FALSE(bridge.workerAbandonAfterProducerStopped(17));
    ASSERT_TRUE(bridge.workerRequestClose(17));
    EXPECT_FALSE(bridge.workerAbandonAfterProducerStopped(18));
    {
        auto isr = bridge.isrBegin(17);
        ASSERT_EQ(isr.status(), Result::Rejected);
    }

    Bridge live_isr;
    ASSERT_TRUE(live_isr.workerBegin(19, 0xA5));
    auto isr = live_isr.isrBegin(19);
    ASSERT_EQ(isr.status(), Result::Accepted);
    ASSERT_TRUE(live_isr.workerRequestClose(19));
    EXPECT_FALSE(live_isr.workerAbandonAfterProducerStopped(19));
}

TEST(I2cSlaveQueueBridge, AbandonDiscardsStoppedProducerLeaseAndResetsCleanly)
{
    Bridge bridge;
    ASSERT_TRUE(bridge.workerBegin(21, 0xA5));
    auto worker = bridge.workerBeginSession(21);
    ASSERT_EQ(worker.status(), Result::Accepted);
    auto stale            = std::make_unique<decltype(worker)>(std::move(worker));
    const uint8_t bytes[] = {1, 2, 3};
    ASSERT_EQ(bridge.workerStageTx(*stale, bytes, sizeof(bytes)), Result::Accepted);
    ASSERT_TRUE(bridge.workerRequestClose(21));
    EXPECT_FALSE(bridge.workerForceQuiesce(21));

    ASSERT_TRUE(bridge.workerAbandonAfterProducerStopped(21));
    EXPECT_EQ(bridge.workerState(), detail::SlaveQueueBridgeState::Quiesced);
    EXPECT_TRUE(bridge.broken());
    auto rejected = bridge.workerBeginSession(21);
    EXPECT_EQ(rejected.status(), Result::Rejected);
    ASSERT_TRUE(bridge.workerReset());
    EXPECT_FALSE(bridge.broken());
    ASSERT_TRUE(bridge.workerBegin(21, 0x5A));
    EXPECT_EQ(bridge.workerTxWritable(), 8u);
    {
        auto current = bridge.workerBeginSession(21);
        ASSERT_EQ(current.status(), Result::Accepted);
        stale.reset();
        ASSERT_EQ(bridge.workerStageTx(current, bytes, sizeof(bytes)), Result::Accepted);
    }
    ASSERT_TRUE(bridge.workerRequestClose(21));
    ASSERT_TRUE(bridge.workerForceQuiesce(21));
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
