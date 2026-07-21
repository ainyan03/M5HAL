// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <memory>

// spi::Bus facade + spi::BusView intern tests.
//
// Wire I/O is not exercised: the software backend resolves pins through the
// stub GPIO port (native build), so init succeeds for arbitrary pin numbers.
// Tests focus on facade construction, backend query forwarding, and BusView
// interning by (CLK, MOSI, MISO).

namespace {
namespace v2 = m5::hal::v2;
}

// ---- spi::Bus facade -------------------------------------------

TEST(SpiBusFacade, InitWithSoftwareBackendSucceeds)
{
    v2::spi::Bus facade;
    v2::spi::BusConfig cfg;
    cfg.pin_clk  = 18;
    cfg.pin_mosi = 23;
    cfg.pin_miso = 19;
    auto r       = facade.init(cfg);
    ASSERT_TRUE(r.has_value()) << "software init should succeed on native build";
}

TEST(SpiBusFacade, QueryApiForwardedToSoftwareBackend)
{
    v2::spi::Bus facade;
    v2::spi::BusConfig cfg;
    cfg.pin_clk  = 18;
    cfg.pin_mosi = 23;
    cfg.pin_miso = 19;
    ASSERT_TRUE(facade.init(cfg).has_value());

    // The software backend reports Software / no controller.
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(facade.controllerId(), -1);
}

TEST(SpiBusFacade, QueryApiBeforeInitReturnsBaseDefaults)
{
    v2::spi::Bus facade;
    // No backend yet — the base-class safe defaults apply.
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(facade.controllerId(), -1);
    EXPECT_EQ(facade.maxFrequency(), 0u);
    EXPECT_EQ(facade.backendGeneration(), 0u);
}

TEST(SpiBusFacade, GetConfigReflectsPins)
{
    v2::spi::Bus facade;
    v2::spi::BusConfig cfg;
    cfg.pin_clk  = 14;
    cfg.pin_mosi = 13;
    cfg.pin_miso = 12;
    ASSERT_TRUE(facade.init(cfg).has_value());

    const auto& stored = facade.getConfig();
    EXPECT_EQ(stored.pin_clk, 14);
    EXPECT_EQ(stored.pin_mosi, 13);
    EXPECT_EQ(stored.pin_miso, 12);
}

// ---- spi::BusView intern ----------------------------------------

TEST(SpiBusView, SamePinsReturnSameInstance)
{
    auto& hal = v2::getM5_Hal();

    // Pin numbers must be < 32 (stub GPIO kMaxWidth = 32; pins 0-31 are valid).
    v2::spi::BusConfig cfg;
    cfg.pin_clk  = 20;
    cfg.pin_mosi = 21;
    cfg.pin_miso = 22;

    auto a = hal.SPI.acquire(cfg);
    ASSERT_TRUE(a.has_value()) << "first acquire should succeed";
    ASSERT_TRUE(a.value()) << "returned shared_ptr must be non-null";

    auto b = hal.SPI.acquire(cfg);
    ASSERT_TRUE(b.has_value()) << "second acquire with same pins should succeed";
    EXPECT_EQ(a.value().get(), b.value().get()) << "same wiring -> same interned instance";
}

TEST(SpiBusView, SameIdentityWithDifferentExtraPinsIsRejected)
{
    auto& hal = v2::getM5_Hal();

    v2::spi::BusConfig cfg_a;
    cfg_a.pin_clk  = 9;
    cfg_a.pin_mosi = 10;
    cfg_a.pin_miso = 11;
    cfg_a.pin_dc   = 12;

    v2::spi::BusConfig cfg_b = cfg_a;
    cfg_b.pin_dc             = 13;  // not identity, but first-config-wins would hide this mismatch

    auto a = hal.SPI.acquire(cfg_a);
    ASSERT_TRUE(a.has_value());

    auto b = hal.SPI.acquire(cfg_b);
    ASSERT_FALSE(b.has_value());
    EXPECT_EQ(b.error(), v2::error::error_t::INVALID_STATE);
}

TEST(SpiBusView, DifferentClkPinsReturnDistinctInstances)
{
    auto& hal = v2::getM5_Hal();

    v2::spi::BusConfig cfg_a;
    cfg_a.pin_clk  = 24;
    cfg_a.pin_mosi = 25;
    cfg_a.pin_miso = 26;

    v2::spi::BusConfig cfg_b;
    cfg_b.pin_clk  = 27;  // different CLK -> different bus
    cfg_b.pin_mosi = 25;
    cfg_b.pin_miso = 26;

    auto a = hal.SPI.acquire(cfg_a);
    auto b = hal.SPI.acquire(cfg_b);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_NE(a.value().get(), b.value().get()) << "different CLK -> distinct instances";
}

TEST(SpiBusView, BackendRejectsMissingClockPin)
{
    auto& hal = v2::getM5_Hal();

    v2::spi::BusConfig cfg;
    // BusView accepts -1 as part of the identity; this software backend still
    // requires a physical clock pin and rejects the config during init.
    cfg.pin_clk  = -1;
    cfg.pin_mosi = 4;
    cfg.pin_miso = 5;

    auto r = hal.SPI.acquire(cfg);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), v2::error::error_t::INVALID_ARGUMENT);
}

TEST(SpiBusView, AcquiredBusReturnsKindSPI)
{
    auto& hal = v2::getM5_Hal();

    v2::spi::BusConfig cfg;
    cfg.pin_clk  = 28;
    cfg.pin_mosi = 29;
    cfg.pin_miso = 30;

    auto r = hal.SPI.acquire(cfg);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value()->getBusKind(), v2::types::bus_kind_t::SPI);
}

// Co-own: a MasterAccessor built from the acquire temporary keeps the bus
// alive after the temporary shared_ptr drops (spi accessor co-owns through the
// ManagedBusFacade base path).
TEST(SpiBusViewCoOwn, AccessorOutlivesAcquireTemporary)
{
    auto& hal = v2::getM5_Hal();

    v2::spi::BusConfig cfg;
    cfg.pin_clk  = 24;
    cfg.pin_mosi = 25;
    cfg.pin_miso = 26;
    v2::spi::MasterAccessConfig acc;

    // Co-own straight from the acquire temporary (no `*value()` deref). The
    // registry holds only a weak ref, so the accessor is the sole strong owner.
    v2::spi::MasterAccessor dev{hal.SPI.acquire(cfg).value(), acc};
    // Reading the bus config goes through the live bus object: proof the bus
    // survived because the accessor co-owns it.
    EXPECT_EQ(dev.getBusConfig().getBusKind(), v2::types::bus_kind_t::SPI);
}

// gtest main (PlatformIO collects this via test_filter=v2/native/*).
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
