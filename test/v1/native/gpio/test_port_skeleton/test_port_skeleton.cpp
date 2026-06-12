// SPDX-License-Identifier: MIT
// Skeleton tests for the Pin / Port abstraction (encoded-value
// hiding + assert-based contracts). The stub variant's Port
// (`m5::hal::v1::gpio::Port_stub`) drives
// these native gtests, covering:
//   - Direct Port operations (write / read / setMode).
//   - Pin operations (write / read / setMode / getLocalPin).
//   - The `getPin` factory.
//   - Consistency across both paths (a Pin write must be observable
//     through a direct Port read).
//   - Independence between multiple Port instances at different
//     base offsets (verified through pin resolution).
//   - Default-constructed `Pin{}` is invalid.
//   - Operating on a default-constructed `Pin{}` is a contract
//     violation and triggers a debug assert.
// The contract is defined in spec/design/gpio.md.

#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

namespace {

using ::m5::hal::v1::types::gpio_mode_t;
using StubPort = ::m5::hal::v1::gpio::Port_stub;

TEST(PortSkeleton, PortDirectWriteRead)
{
    StubPort port{32, 0};
    port.setMode(5, gpio_mode_t::Output);
    port.write(5, true);
    EXPECT_TRUE(port.read(5));
    port.write(5, false);
    EXPECT_FALSE(port.read(5));
}

TEST(PortSkeleton, PinFactoryWriteRead)
{
    StubPort port{32, 0};
    auto pin = port.getPin(7);
    EXPECT_TRUE(pin.isValid());
    EXPECT_EQ(pin.getLocalPin(), 7);
    EXPECT_EQ(pin.getPort(), &port);

    pin.setMode(gpio_mode_t::Output);
    pin.write(true);
    EXPECT_TRUE(pin.read());
    pin.write(false);
    EXPECT_FALSE(pin.read());
}

TEST(PortSkeleton, PinAndPortDirectConsistency)
{
    StubPort port{32, 0};
    auto pin = port.getPin(12);

    pin.write(true);
    EXPECT_TRUE(port.read(12));  // Pin write → Port direct read

    port.write(12, false);
    EXPECT_FALSE(pin.read());  // Port write → Pin read
}

TEST(PortSkeleton, MultiplePortsWithDifferentBase)
{
    StubPort port_bank0{32, 0};  // covers gpio_num 0..31
    StubPort port_bank1{8, 32};  // covers gpio_num 32..39

    auto pin_a = port_bank0.getPin(15);
    auto pin_b = port_bank1.getPin(35);

    pin_a.write(true);
    pin_b.write(false);

    EXPECT_TRUE(pin_a.read());
    EXPECT_FALSE(pin_b.read());
    EXPECT_EQ(pin_a.getLocalPin(), 15);
    EXPECT_EQ(pin_b.getLocalPin(), 35);
    EXPECT_EQ(pin_a.getPort(), &port_bank0);
    EXPECT_EQ(pin_b.getPort(), &port_bank1);

    // bank0 and bank1 keep independent state — they don't bleed into each other.
    EXPECT_FALSE(port_bank1.read(35));
    EXPECT_TRUE(port_bank0.read(15));
}

TEST(PortSkeleton, DefaultPinIsInvalid)
{
    ::m5::hal::v1::gpio::Pin pin{};
    EXPECT_FALSE(pin.isValid());
    EXPECT_EQ(pin.getPort(), nullptr);
}

#if !defined(NDEBUG)

TEST(PortSkeletonDeathTest, DefaultPinWriteAsserts)
{
    ::m5::hal::v1::gpio::Pin pin{};
    EXPECT_DEATH({ pin.write(true); }, "Pin is invalid");
}

#endif  // !defined(NDEBUG)

TEST(PortSkeleton, AllBitsIndependent)
{
    // Every one of the 32 bits should be writable / readable independently
    // (sanity check on the encoded-value layout).
    StubPort port{32, 0};
    for (uint8_t i = 0; i < 32; ++i) {
        port.write(static_cast<::m5::hal::v1::types::gpio_number_t>(i), true);
    }
    for (uint8_t i = 0; i < 32; ++i) {
        EXPECT_TRUE(port.read(static_cast<::m5::hal::v1::types::gpio_number_t>(i))) << "bit " << static_cast<int>(i);
    }
    for (uint8_t i = 0; i < 32; ++i) {
        port.write(static_cast<::m5::hal::v1::types::gpio_number_t>(i), false);
    }
    for (uint8_t i = 0; i < 32; ++i) {
        EXPECT_FALSE(port.read(static_cast<::m5::hal::v1::types::gpio_number_t>(i))) << "bit " << static_cast<int>(i);
    }
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
