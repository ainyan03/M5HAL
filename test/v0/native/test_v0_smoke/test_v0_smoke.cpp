// SPDX-License-Identifier: MIT
// Narrow executable regression surface for the frozen v0 compatibility API.
// This is intentionally not a new v0 feature suite: it protects generic
// behavior that has required a post-release bug fix and can regress while all
// compile/link fences remain green.

#include <M5HAL_v0.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

namespace {

namespace v0 = m5::hal::v0;

class FakePin : public v0::interface::gpio::Pin {
public:
    void write(bool value) override
    {
        level = value;
    }

    bool read(void) override
    {
        return level;
    }

    v0::types::gpio_number_t getGpioNumber(void) const override
    {
        return gpio_num;
    }

    void setMode(v0::types::gpio_mode_t value) override
    {
        mode = value;
    }

    bool level                        = true;
    v0::types::gpio_number_t gpio_num = 0;
    v0::types::gpio_mode_t mode       = v0::types::gpio_mode_t::Input;
};

TEST(V0SoftwareI2C, RejectsMissingPins)
{
    v0::bus::I2CBusConfig cfg{};
    v0::bus::i2c::SoftwareI2CBus bus;

    EXPECT_EQ(bus.init(cfg), v0::error::error_t::INVALID_ARGUMENT);
}

TEST(V0SoftwareI2C, ZeroLengthWriteDoesNotDereferenceNull)
{
    FakePin scl;
    FakePin sda;
    scl.gpio_num = 1;
    sda.gpio_num = 2;

    v0::bus::I2CBusConfig bus_cfg{};
    bus_cfg.pin_scl = &scl;
    bus_cfg.pin_sda = &sda;
    v0::bus::i2c::SoftwareI2CBus bus;
    ASSERT_EQ(bus.init(bus_cfg), v0::error::error_t::OK);

    v0::bus::I2CMasterAccessConfig access_cfg{};
    access_cfg.i2c_addr = 0x42;
    auto opened         = bus.beginAccess(access_cfg);
    ASSERT_TRUE(opened.has_value());
    auto* accessor = static_cast<v0::bus::i2c::SoftwareI2CMasterAccessor*>(opened.value());

    auto written = accessor->write(nullptr, 0);
    ASSERT_TRUE(written.has_value());
    EXPECT_EQ(written.value(), 0u);
    EXPECT_EQ(bus.endAccess(accessor), v0::error::error_t::OK);
}

TEST(V0SoftwareI2C, AccessWindowCanBeReopenedAfterRelease)
{
    FakePin scl;
    FakePin sda;
    v0::bus::I2CBusConfig bus_cfg{};
    bus_cfg.pin_scl = &scl;
    bus_cfg.pin_sda = &sda;
    v0::bus::i2c::SoftwareI2CBus bus;
    ASSERT_EQ(bus.init(bus_cfg), v0::error::error_t::OK);

    v0::bus::I2CMasterAccessConfig access_cfg{};
    access_cfg.i2c_addr = 0x42;
    auto first          = bus.beginAccess(access_cfg);
    ASSERT_TRUE(first.has_value());
    EXPECT_FALSE(bus.beginAccess(access_cfg).has_value());
    ASSERT_EQ(bus.endAccess(first.value()), v0::error::error_t::OK);
    EXPECT_TRUE(bus.beginAccess(access_cfg).has_value());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
