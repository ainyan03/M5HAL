// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

namespace {

using namespace m5::hal::v2;

template <typename R>
void expectError(const R& r, error::error_t expected)
{
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), expected) << "err=" << error::toString(r.error());
}

void expectOk(const result_t<void>& r)
{
    ASSERT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
}

template <typename R>
void expectOkValue(const R& r)
{
    ASSERT_TRUE(r.has_value()) << "err=" << error::toString(r.error());
}

void expectLocalI2CAcquire(Hal& hal)
{
    auto acq = hal.I2C.acquire(i2c::BusConfig{i2c::Scl{22}, i2c::Sda{21}});
    expectOkValue(acq);
}

}  // namespace

TEST(HalRemoteFacade, UnboundUserHalReportsNotConnected)
{
    Hal hal;

    EXPECT_FALSE(hal.hasRemoteConnection());

    expectError(hal.I2C.acquire(i2c::BusConfig{i2c::Scl{22}, i2c::Sda{21}}), error::error_t::NOT_CONNECTED);
    expectError(hal.I2C.acquire(i2c::LogicalBusConfig{i2c::Scl{22}, i2c::Sda{21}}), error::error_t::NOT_CONNECTED);
    expectError(hal.I2C.commitBuses(), error::error_t::NOT_CONNECTED);

    expectError(hal.SPI.acquire(spi::BusConfig{spi::Clk{18}, spi::Mosi{23}, spi::Miso{19}}),
                error::error_t::NOT_CONNECTED);
    expectError(hal.SPI.acquire(spi::LogicalBusConfig{spi::Clk{18}, spi::Mosi{23}, spi::Miso{19}}),
                error::error_t::NOT_CONNECTED);
    expectError(hal.SPI.commitBuses(), error::error_t::NOT_CONNECTED);

    expectError(hal.UART.acquire(uart::BusConfig{}), error::error_t::NOT_CONNECTED);
    expectError(hal.UART.acquire(uart::LogicalBusConfig{uart::Tx{17}, uart::Rx{16}}), error::error_t::NOT_CONNECTED);
    expectError(hal.UART.commitBuses(), error::error_t::NOT_CONNECTED);

    expectError(hal.I2S.acquire(i2s::IBusConfig{i2s::Bclk{12}, i2s::Ws{0}, i2s::Dout{2}}),
                error::error_t::NOT_CONNECTED);
    expectError(hal.I2S.acquire(i2s::LogicalBusConfig{i2s::Bclk{12}, i2s::Ws{0}, i2s::Dout{2}}),
                error::error_t::NOT_CONNECTED);
    expectError(hal.I2S.commitBuses(), error::error_t::NOT_CONNECTED);

    expectError(hal.pumpRemote(), error::error_t::NOT_CONNECTED);
}

TEST(HalRemoteFacade, InitBindsLocalAndIsIdempotent)
{
    Hal hal;

    expectOk(hal.init());
    EXPECT_FALSE(hal.hasRemoteConnection());
    ASSERT_NE(hal.backend(), nullptr);
    expectLocalI2CAcquire(hal);
    EXPECT_TRUE(hal.Gpio.hasGPIO(0));

    auto* backend = hal.backend();
    expectOk(hal.init());
    EXPECT_EQ(hal.backend(), backend);
    expectLocalI2CAcquire(hal);
}

TEST(HalRemoteFacade, ConnectLocalEndpointsBindLocalAndAreIdempotent)
{
    const char* endpoints[] = {nullptr, "", "local"};

    for (const char* endpoint : endpoints) {
        Hal hal;
        expectOk(hal.connect(endpoint));
        ASSERT_NE(hal.backend(), nullptr);
        EXPECT_TRUE(hal.Gpio.hasGPIO(0));
        expectLocalI2CAcquire(hal);

        auto* backend = hal.backend();
        expectOk(hal.connect(endpoint));
        EXPECT_EQ(hal.backend(), backend);
        expectLocalI2CAcquire(hal);
    }
}

TEST(HalRemoteFacade, ConnectRejectsInvalidEndpointGrammar)
{
    Hal hal;

    expectError(hal.connect("bogus"), error::error_t::INVALID_ARGUMENT);
    expectError(hal.connect("uart"), error::error_t::INVALID_ARGUMENT);
    expectError(hal.connect("LOCAL"), error::error_t::INVALID_ARGUMENT);
    expectError(hal.connect("tcp:hostonly"), error::error_t::INVALID_ARGUMENT);
    expectError(hal.connect("tcp:127.0.0.1:0"), error::error_t::INVALID_ARGUMENT);
}

TEST(HalRemoteFacade, FailedRemoteConnectKeepsExistingLocalBinding)
{
    Hal hal;

    expectOk(hal.init());
    auto* backend = hal.backend();
    expectLocalI2CAcquire(hal);

    auto remote = hal.connect("uart:/nonexistent/path");
    ASSERT_FALSE(remote.has_value());
    EXPECT_FALSE(hal.hasRemoteConnection());
    EXPECT_EQ(hal.backend(), backend);
    EXPECT_TRUE(hal.Gpio.hasGPIO(0));
    expectLocalI2CAcquire(hal);
}

TEST(HalRemoteFacade, M5HalInitKeepsExistingBinding)
{
    auto& hal     = getM5_Hal();
    auto* backend = hal.backend();

    expectOk(hal.init());
    EXPECT_EQ(hal.backend(), backend);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
