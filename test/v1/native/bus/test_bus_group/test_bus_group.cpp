// SPDX-License-Identifier: MIT
#include <M5HAL_v1.hpp>
#include <gtest/gtest.h>

// bus::BusGroup — the non-owning slot->bus registry (one per kind on
// M5_Hal). The fakes only need to BE a kind bus; no I/O happens here.

namespace {

class FakeI2cBus : public m5::hal::v1::i2c::IBus {};

}  // namespace

TEST(BusGroup, AddGetRemoveRoundTrip)
{
    m5::hal::v1::i2c::BusGroup group;
    FakeI2cBus bus;

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
    m5::hal::v1::i2c::BusGroup group;
    FakeI2cBus bus;

    ASSERT_TRUE(group.addBus(&bus, 1).has_value());
    ASSERT_TRUE(group.addBus(&bus, 2).has_value());
    EXPECT_EQ(group.getBus(1), group.getBus(2));

    // Removing one alias leaves the other.
    ASSERT_TRUE(group.removeBus(1).has_value());
    EXPECT_EQ(group.getBus(2), &bus);
}

TEST(BusGroup, RejectsNullOutOfRangeAndOccupiedSlots)
{
    m5::hal::v1::i2c::BusGroup group;
    FakeI2cBus bus_a;
    FakeI2cBus bus_b;

    EXPECT_FALSE(group.addBus(nullptr, 0).has_value());
    EXPECT_FALSE(group.addBus(&bus_a, m5::hal::v1::i2c::BusGroup::kSlotCount).has_value());

    ASSERT_TRUE(group.addBus(&bus_a, 0).has_value());
    auto r = group.addBus(&bus_b, 0);  // occupied
    EXPECT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(group.getBus(0), &bus_a);  // unchanged

    EXPECT_FALSE(group.removeBus(5).has_value());  // empty slot
    EXPECT_EQ(group.getBus(m5::hal::v1::i2c::BusGroup::kSlotCount), nullptr);  // out of range = nullptr, not UB
}

TEST(BusGroup, LivesPerKindOnM5Hal)
{
    // The registries are reachable as M5_Hal.<KIND>; this test uses the
    // singleton, so it cleans up after itself.
    auto& hal = m5::hal::v1::getM5_Hal();
    FakeI2cBus bus;

    ASSERT_TRUE(hal.I2C.addBus(&bus, 3).has_value());
    EXPECT_EQ(hal.I2C.getBus(3), &bus);
    EXPECT_FALSE(hal.SPI.hasBus(3));  // kinds are independent tables
    ASSERT_TRUE(hal.I2C.removeBus(3).has_value());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
