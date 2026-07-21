// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <M5HAL_v2.hpp>

#include "m5_hal/hal/v2/bus/bus.inl"
#include "m5_hal/hal/v2/data/ring.inl"
#include "m5_hal/hal/v2/data/stream.inl"
#include "m5_hal/hal/v2/service/service.inl"
#include "m5_hal/hal/v2/spi/slave.inl"
#include "m5_hal/variants/frameworks/espidf/hal/spi/slave.inl"

#include <driver/gpio.h>
#include <driver/spi_slave.h>
#include <freertos/task.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <thread>

namespace {

using namespace m5::hal::v2;

spi::SlaveBusConfig busConfig()
{
    spi::SlaveBusConfig config;
    config.pin_clk  = 1;
    config.pin_mosi = 2;
    config.pin_miso = 3;
    config.pin_cs   = 4;
    return config;
}

spi::SlaveAccessConfig byteConfig(uint32_t transaction_bytes = 4)
{
    spi::SlaveAccessConfig config;
    config.transaction_bytes = transaction_bytes;
    config.tx_mode           = slave::QueueMode::Byte;
    config.rx_mode           = slave::QueueMode::Byte;
    return config;
}

struct Harness {
    spi::SpiSlaveBus_espidf bus;
    slave::StaticSlaveQueueStorage<64, 64, 4, 4> storage;
    spi::SpiSlaveAccessor accessor;

    explicit Harness(const spi::SlaveAccessConfig& config = byteConfig())
        : accessor{bus, storage.tx(), storage.rx(), config}
    {
        m5hal_hostharness::resetSpiSlave();
        m5hal_hostharness::spiCsLevel = 1;
        auto initialized              = bus.init(busConfig());
        if (!initialized.has_value()) {
            ADD_FAILURE() << error::toString(initialized.error());
        }
    }
};

void expectBegin(Harness& harness, uint32_t timeout_ms = 1000)
{
    auto begun = harness.accessor.beginAccess(timeout_ms);
    ASSERT_TRUE(begun.has_value()) << error::toString(begun.error());
    ASSERT_TRUE(m5hal_hostharness::waitForQueuedTransaction());
}

}  // namespace

TEST(EspidfSpiSlaveLifecycle, ByteTransferCompletesAndGracefulEndRegresses)
{
    Harness harness;
    const std::array<uint8_t, 3> tx{0x10, 0x11, 0x12};
    ASSERT_TRUE(harness.accessor.write({tx.data(), tx.size()}).has_value());
    expectBegin(harness);

    const std::array<uint8_t, 4> rx{0x21, 0x22, 0x23, 0x24};
    ASSERT_TRUE(m5hal_hostharness::completeSpiTransaction(rx.data(), rx.size()));
    ASSERT_TRUE(m5hal_hostharness::waitForQueuedTransaction());
    EXPECT_EQ(harness.accessor.writable(), 64u);
    EXPECT_EQ(harness.accessor.readable(), rx.size());

    std::array<uint8_t, 4> observed{};
    auto read = harness.accessor.read({observed.data(), observed.size()});
    ASSERT_TRUE(read.has_value());
    EXPECT_EQ(*read, rx.size());
    EXPECT_EQ(observed, rx);
    EXPECT_TRUE(harness.accessor.endAccess(1000).has_value());
    EXPECT_FALSE(harness.accessor.inAccess());
}

TEST(EspidfSpiSlaveLifecycle, FrameTransferPreservesFrameBoundaryAndReportsTruncation)
{
    auto config    = byteConfig(3);
    config.tx_mode = slave::QueueMode::Frame;
    config.rx_mode = slave::QueueMode::Frame;
    Harness harness{config};

    const std::array<uint8_t, 4> tx{1, 2, 3, 4};
    auto reservation = harness.accessor.txFrames().reserveFrame(tx.size());
    ASSERT_TRUE(reservation.has_value());
    std::copy(tx.begin(), tx.end(), reservation->first.data);
    ASSERT_TRUE(harness.accessor.txFrames().commitFrame(*reservation, {}).has_value());
    expectBegin(harness);

    const std::array<uint8_t, 3> rx{7, 8, 9};
    ASSERT_TRUE(m5hal_hostharness::completeSpiTransaction(rx.data(), rx.size()));
    ASSERT_TRUE(m5hal_hostharness::waitForQueuedTransaction());
    EXPECT_NE(static_cast<uint32_t>(harness.accessor.txStatus().sticky_events), 0u);
    auto frame = harness.accessor.rxFrames().peekFrame();
    ASSERT_TRUE(frame.has_value());
    EXPECT_EQ(frame->first.size + frame->second.size, rx.size());
    EXPECT_TRUE(harness.accessor.rxFrames().popFrame().has_value());
    EXPECT_TRUE(harness.accessor.endAccess(1000).has_value());
}

TEST(EspidfSpiSlaveLifecycle, InFlightTimeoutDefersDriverCleanupUntilCsIsIdle)
{
    Harness harness;
    expectBegin(harness);
    m5hal_hostharness::spiCsLevel = 0;

    auto ended = harness.accessor.endAccess(0);
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::UNSUPPORTED);
    EXPECT_FALSE(harness.accessor.inAccess());
    EXPECT_EQ(m5hal_hostharness::fakeSpiSlave().disable_count, 1u);
    EXPECT_EQ(m5hal_hostharness::fakeSpiSlave().reset_count, 0u);
    EXPECT_EQ(m5hal_hostharness::fakeSpiSlave().free_count, 0u);
    auto blocked = harness.accessor.beginAccess(0);
    ASSERT_FALSE(blocked.has_value());
    EXPECT_EQ(blocked.error(), error::error_t::IO_ERROR);

    auto first_close = harness.bus.close();
    ASSERT_FALSE(first_close.has_value());
    EXPECT_EQ(first_close.error(), error::error_t::UNSUPPORTED);
    m5hal_hostharness::spiCsLevel = 1;
    ASSERT_TRUE(harness.bus.close().has_value());
    ASSERT_TRUE(harness.bus.init(busConfig()).has_value());
    ASSERT_TRUE(harness.accessor.beginAccess(1000).has_value());
    ASSERT_TRUE(m5hal_hostharness::waitForQueuedTransaction());
    ASSERT_TRUE(harness.accessor.endAccess(1000).has_value());
}

TEST(EspidfSpiSlaveLifecycle, LateCallbackAfterAccessorDestructionDoesNotTouchCallerStorage)
{
    m5hal_hostharness::resetSpiSlave();
    m5hal_hostharness::spiCsLevel = 0;
    spi::SpiSlaveBus_espidf bus;
    ASSERT_TRUE(bus.init(busConfig()).has_value());
    {
        slave::StaticSlaveQueueStorage<64, 64, 4, 4> storage;
        auto accessor = std::make_unique<spi::SpiSlaveAccessor>(bus, storage.tx(), storage.rx(), byteConfig());
        ASSERT_TRUE(accessor->beginAccess(1000).has_value());
        ASSERT_TRUE(m5hal_hostharness::waitForQueuedTransaction());
        auto ended = accessor->endAccess(0);
        ASSERT_FALSE(ended.has_value());
        EXPECT_EQ(ended.error(), error::error_t::UNSUPPORTED);
    }

    const std::array<uint8_t, 4> rx{9, 8, 7, 6};
    EXPECT_TRUE(m5hal_hostharness::completeSpiTransaction(rx.data(), rx.size()));
    m5hal_hostharness::spiCsLevel = 1;
    EXPECT_TRUE(bus.close().has_value());
}

TEST(EspidfSpiSlaveLifecycle, BeginTimeoutUsesAbortCleanupAndDetachesCaller)
{
    Harness harness;
    m5hal_hostharness::holdNextCreatedTask();
    auto begun = harness.accessor.beginAccess(0);
    ASSERT_FALSE(begun.has_value());
    EXPECT_EQ(begun.error(), error::error_t::TIMEOUT_ERROR);
    EXPECT_FALSE(harness.accessor.inAccess());
    EXPECT_EQ(m5hal_hostharness::fakeSpiSlave().free_count, 1u);
    EXPECT_FALSE(m5hal_hostharness::fireRetiredCallback());
    EXPECT_EQ(m5hal_hostharness::fakeSpiSlave().retired, nullptr);
}

TEST(EspidfSpiSlaveLifecycle, CallbackRacingDeadlineLeavesNoLiveCallerPath)
{
    Harness harness;
    expectBegin(harness);
    m5hal_hostharness::spiCsLevel = 0;
    const std::array<uint8_t, 4> rx{1, 2, 3, 4};
    std::thread completion([&] { (void)m5hal_hostharness::completeSpiTransaction(rx.data(), rx.size()); });
    auto ended = harness.accessor.endAccess(0);
    completion.join();
    EXPECT_FALSE(harness.accessor.inAccess());
    if (!ended.has_value()) {
        EXPECT_TRUE(ended.error() == error::error_t::UNSUPPORTED || ended.error() == error::error_t::TIMEOUT_ERROR);
    }
    (void)m5hal_hostharness::fireRetiredCallback();
    m5hal_hostharness::spiCsLevel = 1;
    EXPECT_TRUE(harness.bus.close().has_value());
}

TEST(EspidfSpiSlaveLifecycle, DriverFreeFailureStillDetachesAndCloseRetryRecoversWithoutHang)
{
    Harness harness;
    expectBegin(harness);
    m5hal_hostharness::spiCsLevel               = 1;
    m5hal_hostharness::fakeSpiSlave().fail_free = true;

    auto ended = harness.accessor.endAccess(0);
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), error::error_t::INVALID_STATE);
    EXPECT_FALSE(harness.accessor.inAccess());
    EXPECT_TRUE(m5hal_hostharness::fireRetiredCallback());
    EXPECT_EQ(harness.accessor.readable(), 0u);

    auto first_close = harness.bus.close();
    ASSERT_FALSE(first_close.has_value());
    EXPECT_EQ(first_close.error(), error::error_t::INVALID_STATE);

    m5hal_hostharness::fakeSpiSlave().fail_free = false;
    const auto started                          = std::chrono::steady_clock::now();
    EXPECT_TRUE(harness.bus.close().has_value());
    EXPECT_LT(std::chrono::steady_clock::now() - started, std::chrono::seconds(1));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
