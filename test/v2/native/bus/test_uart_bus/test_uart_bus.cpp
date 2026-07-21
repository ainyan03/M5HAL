// SPDX-License-Identifier: MIT
#include "../static_bus_view_contract.hpp"

#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/bus/local_backend.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <memory>

// uart::Bus facade + uart::BusView intern tests.
//
// The real UART backends (arduino / espidf / posix) open a hardware/serial
// port on init, so a host test uses a TEST-LOCAL fake backend (no public
// surface change). Tests cover facade construction, the query API, BusView
// interning by (TX, RX), and -- the UART-specific point -- that the facade
// still exposes INDEPENDENT TX / RX channel locks (the 2-mutex model is
// inherited from uart::IBus and stays on the facade).

namespace m5::hal::v2::uart {

// Test-only portable provider. No I/O: the lock tests only open access
// windows (begin/endAccess), which exercise the channel mutexes.
using FakeBusConfig = IBusConfig;

class FakeBus : public IBus {
public:
    m5::hal::v2::result_t<void> init(const FakeBusConfig& cfg)
    {
        _config = cfg;  // slice pins/kind for getConfig()
        return {};
    }
    // write / read / readableBytes keep the base NOT_IMPLEMENTED defaults; the
    // tests here do not move data.
};

result_t<std::unique_ptr<IBus>> makeFakeBackend(const bus::LocalResourceContext&, const IBusConfig& cfg)
{
    std::unique_ptr<FakeBus> backend{new (std::nothrow) FakeBus()};
    if (!backend) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    auto initialized = backend->init(cfg);
    if (!initialized.has_value()) {
        return m5::stl::make_unexpected(initialized.error());
    }
    return std::unique_ptr<IBus>{std::move(backend)};
}

result_t<void> initFakeFacade(Bus& facade, const IBusConfig& cfg)
{
    auto backend = makeFakeBackend({}, cfg);
    if (!backend.has_value()) {
        return m5::stl::make_unexpected(backend.error());
    }
    return facade.adoptPortableBackend(std::move(backend.value()), cfg);
}

struct FakeHal {
    bus::LocalBackend backend;
    bus::LocalPortableProvider<BusTraits> provider{&makeFakeBackend};
    BusView UART{&backend};

    FakeHal()
    {
        backend.registerPortableProvider(provider);
    }
};

}  // namespace m5::hal::v2::uart

namespace {
namespace v2 = m5::hal::v2;
}

// ---- uart::Bus facade ----------------------------------------------

TEST(UartBusFacade, InitWithFakeBackendSucceeds)
{
    v2::uart::Bus facade;
    v2::uart::FakeBusConfig cfg;
    cfg.pin_tx = 1;
    cfg.pin_rx = 3;
    ASSERT_TRUE(v2::uart::initFakeFacade(facade, cfg).has_value());
    EXPECT_EQ(facade.getConfig().pin_tx, 1);
    EXPECT_EQ(facade.getConfig().pin_rx, 3);
}

TEST(UartBusFacade, QueryApiBeforeInitReturnsDefaults)
{
    v2::uart::Bus facade;
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(facade.controllerId(), -1);
    EXPECT_EQ(facade.maxFrequency(), 0u);
}

// ---- uart::BusView intern ------------------------------------------

TEST(UartBusView, SamePinsReturnSameInstance)
{
    v2::uart::FakeHal hal;
    v2::uart::FakeBusConfig cfg;
    cfg.pin_tx = 10;
    cfg.pin_rx = 11;

    auto a = hal.UART.acquire(cfg);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(a.value());
    auto b = hal.UART.acquire(cfg);
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a.value().get(), b.value().get()) << "same wiring -> same interned instance";
}

TEST(UartBusView, SameIdentityWithDifferentBufferConfigIsRejected)
{
    v2::uart::FakeHal hal;
    v2::uart::FakeBusConfig cfg_a;
    cfg_a.pin_tx         = 22;
    cfg_a.pin_rx         = 23;
    cfg_a.rx_buffer_size = 512;

    v2::uart::FakeBusConfig cfg_b = cfg_a;
    cfg_b.rx_buffer_size          = 1024;  // not identity, but changes the opened driver resources

    auto a = hal.UART.acquire(cfg_a);
    ASSERT_TRUE(a.has_value());

    auto b = hal.UART.acquire(cfg_b);
    ASSERT_FALSE(b.has_value());
    EXPECT_EQ(b.error(), v2::error::error_t::INVALID_STATE);
}

TEST(UartBusView, DifferentTxReturnDistinctInstances)
{
    v2::uart::FakeHal hal;
    v2::uart::FakeBusConfig cfg_a;
    cfg_a.pin_tx = 12;
    cfg_a.pin_rx = 13;
    v2::uart::FakeBusConfig cfg_b;
    cfg_b.pin_tx = 14;  // different TX -> different bus
    cfg_b.pin_rx = 13;

    auto a = hal.UART.acquire(cfg_a);
    auto b = hal.UART.acquire(cfg_b);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_NE(a.value().get(), b.value().get());
}

TEST(UartBusView, UnsetPinRemainsPartOfIdentity)
{
    v2::uart::FakeHal hal;
    v2::uart::FakeBusConfig cfg;
    cfg.pin_tx = -1;
    cfg.pin_rx = 5;
    auto r     = hal.UART.acquire(cfg);
    ASSERT_TRUE(r.has_value()) << "err=" << v2::error::toString(r.error());
    EXPECT_EQ(r.value()->getConfig().pin_tx, -1);
    EXPECT_EQ(r.value()->getConfig().pin_rx, 5);
}

TEST(UartBusView, StaticPolicyCommitIsNoop)
{
    v2::uart::FakeHal hal;
    v2::test::bus_contract::expectStaticCommitSurface(hal.UART);
}

TEST(UartBusView, LogicalAcquireSurfaceExistsButIsStaticPolicy)
{
    v2::uart::FakeHal hal;
    v2::uart::LogicalBusConfig req{v2::uart::Tx{18}, v2::uart::Rx{19}};
    v2::uart::LogicalBusConfig unset_pin{v2::uart::Tx{-1}, v2::uart::Rx{19}};

    v2::test::bus_contract::expectStaticLogicalAcquireContract(hal.UART, req, unset_pin);
}

// ---- UART-specific: independent TX / RX channel locks through the facade -----

TEST(UartBusView, ChannelLocksAreIndependent)
{
    v2::uart::FakeHal hal;
    v2::uart::FakeBusConfig cfg;
    cfg.pin_tx = 16;
    cfg.pin_rx = 17;
    auto bus   = hal.UART.acquire(cfg);
    ASSERT_TRUE(bus.has_value());

    v2::uart::AccessConfig acc;
    v2::uart::TxAccessor tx{*bus.value(), acc};
    v2::uart::RxAccessor rx{*bus.value(), acc};

    // Opening the TX window locks only the TX channel, so the RX window can be
    // opened concurrently -- the 2-mutex model survives through the facade.
    ASSERT_TRUE(tx.beginAccess().has_value());
    ASSERT_TRUE(rx.beginAccess().has_value());
    ASSERT_TRUE(rx.endAccess().has_value());
    ASSERT_TRUE(tx.endAccess().has_value());
}

// Co-own: a Tx/Rx accessor (and the bundling Accessor) built from the acquire
// temporary keeps the bus alive after the temporary shared_ptr drops. The
// 2-mutex / StreamWriter+Reader multiple-inheritance accessors must forward
// the shared_ptr ctor to the base correctly.
TEST(UartBusViewCoOwn, AccessorOutlivesAcquireTemporary)
{
    v2::uart::FakeHal hal;
    v2::uart::FakeBusConfig cfg;
    cfg.pin_tx = 16;
    cfg.pin_rx = 17;
    v2::uart::AccessConfig acc;

    // Co-own straight from the acquire temporary (no `*value()` deref).
    v2::uart::TxAccessor tx{hal.UART.acquire(cfg).value(), acc};
    // Reading the bus config goes through the live bus object; an open/close
    // window exercises the facade lock. Both prove the bus survived.
    EXPECT_EQ(tx.getBusConfig().getBusKind(), v2::types::bus_kind_t::UART);
    EXPECT_TRUE(tx.beginAccess().has_value());
    EXPECT_TRUE(tx.endAccess().has_value());

    // The bundling Accessor co-owns through both channels. Use a DISTINCT
    // wiring so this instance's only strong owner is `both` (not `tx` above,
    // which would otherwise keep the same interned instance alive and confound
    // the test).
    v2::uart::FakeBusConfig cfg2;
    cfg2.pin_tx = 20;
    cfg2.pin_rx = 21;
    v2::uart::Accessor both{hal.UART.acquire(cfg2).value(), acc};
    EXPECT_EQ(both.getBus().getBusKind(), v2::types::bus_kind_t::UART);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
