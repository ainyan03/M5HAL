// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

// bus::BusGroup — the non-owning slot->bus registry. Every kind (I2C / SPI /
// UART / I2S) now reaches buses through the owning registry view (
//), so BusGroup is no longer a per-kind access surface on M5_Hal; it
// remains a standalone utility and is exercised here directly. The fake only
// needs to BE a kind bus; no I/O here.

namespace {

class FakeSpiBus : public m5::hal::v2::spi::IBus {};

}  // namespace

TEST(BusGroup, AddGetRemoveRoundTrip)
{
    m5::hal::v2::spi::BusGroup group;
    FakeSpiBus bus;

    EXPECT_FALSE(group.hasBus(1));
    EXPECT_EQ(group.getBus(1), nullptr);

    ASSERT_TRUE(group.addBus(&bus, 1).has_value());
    EXPECT_TRUE(group.hasBus(1));
    EXPECT_EQ(group.getBus(1), &bus);

    ASSERT_TRUE(group.removeBus(1).has_value());
    EXPECT_FALSE(group.hasBus(1));
    EXPECT_EQ(group.getBus(1), nullptr);  // the object itself is untouched
}

TEST(BusGroup, AliasingTheSameBusInSeveralSlots)
{
    // "slot 1 = SD, slot 2 = LCD, both are one physical bus": register
    // the same pointer twice.
    m5::hal::v2::spi::BusGroup group;
    FakeSpiBus bus;

    ASSERT_TRUE(group.addBus(&bus, 1).has_value());
    ASSERT_TRUE(group.addBus(&bus, 2).has_value());
    EXPECT_EQ(group.getBus(1), group.getBus(2));

    // Removing one alias leaves the other.
    ASSERT_TRUE(group.removeBus(1).has_value());
    EXPECT_EQ(group.getBus(2), &bus);
}

TEST(BusGroup, RejectsNullOutOfRangeAndOccupiedSlots)
{
    m5::hal::v2::spi::BusGroup group;
    FakeSpiBus bus_a;
    FakeSpiBus bus_b;

    EXPECT_FALSE(group.addBus(nullptr, 0).has_value());
    EXPECT_FALSE(group.addBus(&bus_a, m5::hal::v2::spi::BusGroup::kSlotCount).has_value());

    ASSERT_TRUE(group.addBus(&bus_a, 0).has_value());
    auto r = group.addBus(&bus_b, 0);  // occupied
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(group.getBus(0), &bus_a);  // unchanged

    EXPECT_FALSE(group.removeBus(5).has_value());                              // empty slot
    EXPECT_EQ(group.getBus(m5::hal::v2::spi::BusGroup::kSlotCount), nullptr);  // out of range = nullptr, not UB
}

// (The former LivesPerKindOnM5Hal case is gone: no M5_Hal.<KIND> is a BusGroup
// anymore -- every kind is a registry view, covered by test_bus_registry /
// test_spi_bus / test_i2s_bus. BusGroup itself is exercised above as a
// standalone utility.)

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
