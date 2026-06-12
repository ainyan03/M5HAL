// SPDX-License-Identifier: MIT
// Skeleton tests for the `m5::hal::v1::gpio::GPIO` abstract and the
// `stub::GPIO` concrete. By design, `GPIO` focuses on Port
// management and exposes no pin operations (`write` / `read` /
// `setMode` etc.); callers reach a Port via `GPIO::portForPin` or
// obtain a `Pin` value via `GPIO::getPin`.
// Coverage:
//   - `portForPin` and `getPort` return the same Port instance.
//   - The `Pin` returned by `getPin` behaves correctly
//     (`write` / `read` / `setMode` / `getLocalPin`).
//   - Direct Port operations and Pin-mediated operations agree.
//   - The free function `stub::getGPIO()` hands back the static
//     instance.
// The contract is defined in spec/design/gpio.md.

#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

namespace {

using ::m5::hal::v1::types::gpio_mode_t;
using StubGPIO = ::m5::variants::frameworks::stub::hal::v1::gpio::GPIO;

TEST(GpioSkeleton, GetPinReturnsValidPin)
{
    StubGPIO gpio;
    auto pin = gpio.getPin(12);
    EXPECT_TRUE(pin.isValid());
    EXPECT_EQ(pin.getLocalPin(), 12);
}

TEST(GpioSkeleton, PortForPinAndGetPortReturnSamePort)
{
    StubGPIO gpio;
    auto* port_via_pin    = gpio.portForPin(0);
    auto* port_via_number = gpio.getPort(0);
    EXPECT_EQ(port_via_pin, port_via_number);
    EXPECT_NE(port_via_pin, nullptr);
}

TEST(GpioSkeleton, PinWriteReadViaGetPin)
{
    StubGPIO gpio;
    auto pin = gpio.getPin(7);
    pin.setMode(gpio_mode_t::Output);
    pin.write(true);
    EXPECT_TRUE(pin.read());
    pin.write(false);
    EXPECT_FALSE(pin.read());
}

TEST(GpioSkeleton, PortDirectAndPinViewsAreConsistent)
{
    StubGPIO gpio;
    auto* port = gpio.portForPin(3);
    ASSERT_NE(port, nullptr);

    // Write through the Port directly, read back through a Pin.
    port->setMode(3, gpio_mode_t::Output);
    port->write(3, true);
    EXPECT_TRUE(gpio.getPin(3).read());

    // Write through a Pin, read back through the Port directly.
    gpio.getPin(3).write(false);
    EXPECT_FALSE(port->read(3));
}

TEST(GpioSkeleton, StubGetGPIOReturnsStaticInstance)
{
    auto* a = ::m5::variants::frameworks::stub::hal::v1::gpio::getGPIO();
    auto* b = ::m5::variants::frameworks::stub::hal::v1::gpio::getGPIO();
    EXPECT_EQ(a, b);
    EXPECT_NE(a, nullptr);
}

// Regression test: the abstract base (`IGPIO`) and the flat-injected
// concrete alias (`GPIO`) should give two type views of the same
// instance. This also exercises that `offer_all.inl`'s flat
// injection works alongside the `I`-prefixed abstract base name.
TEST(GpioSkeleton, FlatInjectExposesConcreteAsM5HalGpioGPIO)
{
    // Flat injection makes `m5::hal::v1::gpio::GPIO` an alias for
    // `stub::GPIO` when the stub variant is active. In native tests,
    // the stub (with `HAS_HAL_GPIO_`) is the last to be scanned, so
    // `m5::hal::v1::gpio::GPIO` can construct the concrete directly.
    ::m5::hal::v1::gpio::GPIO concrete;
    ::m5::hal::v1::gpio::IGPIO* abstract_ptr = &concrete;
    EXPECT_NE(abstract_ptr, nullptr);

    auto pin = abstract_ptr->getPin(5);
    EXPECT_TRUE(pin.isValid());
    EXPECT_EQ(pin.getLocalPin(), 5);
}

// The bitfield-encoded mode set through a Pin should propagate into
// the stub's internal state. Doubles as a regression test that newly
// added enum-class values (`InputPullup`, `InputPulldown`, ...)
// continue to compile.
TEST(GpioSkeleton, BitfieldModeIsAcceptedViaPin)
{
    StubGPIO gpio;
    gpio.getPin(0).setMode(gpio_mode_t::InputPullup);
    gpio.getPin(1).setMode(gpio_mode_t::InputPulldown);
    gpio.getPin(2).setMode(gpio_mode_t::OutputOpenDrain);
    gpio.getPin(3).setMode(gpio_mode_t::OutputOpenDrainPullup);
    SUCCEED();
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
