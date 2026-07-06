// SPDX-License-Identifier: MIT
#include "../managed_bus_view_contract.hpp"

#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/bus/local_backend.hpp>
#include <gtest/gtest.h>

#include <new>

// ADR 034 phase 3 — intent-driven hardware allocation for SPI. These tests
// drive the resolver (spi::BusView::commitBuses) over a LOCAL registry with
// injected fake factories + a 2-controller silicon budget, mirroring the i2c
// intent tests (test_bus_intent.cpp). The fakes perform no I/O and never touch
// GPIO, so the test is about allocation, the controller assignments, and the
// swap generations — the real software/espidf wire paths are covered elsewhere.
//
// SPI is at i2c's degenerate point (uniform controllers + a software
// placeholder), so the cross-kind AllocationCore-seam paths (null placeholder,
// the capability eligibility filter) are exercised by the i2c-file's
// AllocationCoreSeam tests and not repeated here.

// Test-local config + backend for the TYPED acquire<CfgT> path (an explicit
// backend choice). The BackendFor specialization lets `BusView::acquire(cfg)`
// build it through `Bus::init<CfgT>`; such a bus is NOT intent-managed, so
// commitBuses() must leave it on this backend.
namespace m5::hal::v2::spi {
struct TypedFakeConfig : public IBusConfig {
    using IBusConfig::IBusConfig;
};
class TypedFakeBackend : public IBus {
public:
    m5::hal::v2::result_t<void> init(const TypedFakeConfig& cfg)
    {
        _config = cfg;
        return {};
    }
    // backendKind() keeps the base default (Software).
};
template <>
struct BackendFor<TypedFakeConfig> {
    using type = TypedFakeBackend;
};

// Typed (unmanaged) HARDWARE backend pinned to controller 0: commitBuses()
// must reserve its controller so a managed bus is not assigned the same one.
struct TypedFakeHwConfig : public IBusConfig {
    using IBusConfig::IBusConfig;
};
class TypedFakeHwBackend : public IBus {
public:
    m5::hal::v2::result_t<void> init(const TypedFakeHwConfig& cfg)
    {
        _config = cfg;
        return {};
    }
    m5::hal::v2::types::backend_kind_t backendKind(void) const override
    {
        return m5::hal::v2::types::backend_kind_t::Hardware;
    }
    int8_t controllerId(void) const override
    {
        return 0;
    }
};
template <>
struct BackendFor<TypedFakeHwConfig> {
    using type = TypedFakeHwBackend;
};
}  // namespace m5::hal::v2::spi

namespace {
namespace v2 = m5::hal::v2;

// A fake backend with a fixed kind + controller, no I/O. As hardware it reports
// its leased controller; as software it has none -- enough for the resolver and
// the facade's query API to treat it correctly.
class FakeBackend : public v2::spi::IBus {
public:
    FakeBackend(v2::types::backend_kind_t kind, int8_t controller) : _kind{kind}, _controller{controller}
    {
    }
    v2::types::backend_kind_t backendKind(void) const override
    {
        return _kind;
    }
    int8_t controllerId(void) const override
    {
        return _kind == v2::types::backend_kind_t::Hardware ? _controller : static_cast<int8_t>(-1);
    }
    uint32_t maxFrequency(void) const override
    {
        return _kind == v2::types::backend_kind_t::Hardware ? 80000000u : 0u;
    }

private:
    v2::types::backend_kind_t _kind;
    int8_t _controller;
};

// Factories injected into the test BusView. The hardware factory plays the role
// espidf fills in a real build; the software factory plays the always-present
// bit-bang fallback. Neither touches GPIO (this test is about allocation).
v2::spi::IBus* fakeSwFactory(const v2::spi::LogicalBusConfig& /*logical*/)
{
    return new (std::nothrow) FakeBackend(v2::types::backend_kind_t::Software, -1);
}
v2::spi::IBus* fakeHwFactory(const v2::spi::LogicalBusConfig& /*logical*/, int8_t controller)
{
    return new (std::nothrow) FakeBackend(v2::types::backend_kind_t::Hardware, controller);
}

v2::spi::LogicalBusConfig req(v2::types::gpio_number_t clk, v2::types::gpio_number_t mosi,
                              v2::types::gpio_number_t miso, v2::types::AllocationIntent intent)
{
    return v2::spi::LogicalBusConfig{v2::spi::Clk{clk}, v2::spi::Mosi{mosi}, v2::spi::Miso{miso}, intent};
}

constexpr auto kHw = v2::types::backend_kind_t::Hardware;

v2::spi::LogicalBusConfig reqByIndex(size_t index, v2::types::AllocationIntent intent)
{
    static constexpr v2::types::gpio_number_t pins[][3] = {
        {18, 23, 19},
        {14, 13, 12},
        {25, 26, 27},
    };
    return req(pins[index][0], pins[index][1], pins[index][2], intent);
}

struct SpiIntentHarness {
    using Adapter = v2::bus::LocalKindAdapter<v2::spi::BusTraits>;

    v2::bus::LocalBackend backend;
    Adapter adapter;
    v2::spi::BusView view;

    SpiIntentHarness(Adapter::SwFactory sw, Adapter::HwFactory hw = nullptr, uint8_t hw_capacity = 0)
        : adapter{backend.busRegistry(), sw, hw, hw_capacity}, view{&backend}
    {
        backend.registerKind(adapter);
    }
};

v2::spi::TypedFakeConfig makeTypedFakeConfig(void)
{
    v2::spi::TypedFakeConfig typed;
    typed.pin_clk  = 14;
    typed.pin_mosi = 13;
    typed.pin_miso = 12;
    return typed;
}

v2::spi::TypedFakeHwConfig makeTypedFakeHwConfig(void)
{
    v2::spi::TypedFakeHwConfig pinned_cfg;
    pinned_cfg.pin_clk  = 10;
    pinned_cfg.pin_mosi = 11;
    pinned_cfg.pin_miso = 9;
    return pinned_cfg;
}

}  // namespace

TEST(SpiBusIntent, ThreeBusAllocationThenThirdPromotion)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectThreeBusAllocationThenThirdPromotion(h.view, &reqByIndex);
}

TEST(SpiBusIntent, RequireOverSubscriptionFailsWithoutHalfChange)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectRequireOverSubscriptionFailsWithoutHalfChange(h.view, &reqByIndex);
}

TEST(SpiBusIntent, NoHardwareFactoryKeepsEverythingSoftware)
{
    // A software-only / host build: no hardware factory, zero silicon budget.
    SpiIntentHarness h{&fakeSwFactory};
    v2::test::bus_contract::expectNoHardwareFactoryKeepsEverythingSoftware(h.view, &reqByIndex);
}

TEST(SpiBusIntent, RequireControllerClaimsSpecificController)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectRequireControllerClaimsSpecificController(h.view, &reqByIndex);
}

TEST(SpiBusIntent, TypedAcquireIsNotManagedByCommit)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectTypedAcquireIsNotManagedByCommit(h.view, &reqByIndex, &makeTypedFakeConfig);
}

TEST(SpiBusIntent, UnmanagedHardwareBusReservesItsController)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectUnmanagedHardwareBusReservesItsController(h.view, &reqByIndex,
                                                                            &makeTypedFakeHwConfig);
}

TEST(SpiBusIntent, SoftwareForbidKeepsHardwareOff)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectSoftwareForbidKeepsHardwareOff(h.view, &reqByIndex);
}

TEST(SpiBusIntent, PreferControllerTakesRequestedWhenFree)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectPreferControllerTakesRequestedWhenFree(h.view, &reqByIndex);
}

TEST(SpiBusIntent, PreferControllerFallsBackWhenTaken)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectPreferControllerFallsBackWhenTaken(h.view, &reqByIndex);
}

TEST(SpiBusIntent, RequireAndForbidConflictIsInvalid)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectRequireAndForbidConflictIsInvalid(h.view, &reqByIndex);
}

TEST(SpiBusIntent, NegativeSpecificControllerIsInvalid)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectNegativeSpecificControllerIsInvalid(h.view, &reqByIndex);
}

TEST(SpiBusIntent, DeterministicTieBreakLowestController)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectDeterministicTieBreakLowestController(h.view, &reqByIndex);
}

TEST(SpiBusIntent, MisoLessWiringIsValidIdentity)
{
    // SPI write-only (display class) buses have no MISO; the bus is still
    // identified by CLK / MOSI, and the intent path must accept it.
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};

    v2::spi::LogicalBusConfig lcd{v2::spi::Clk{18}, v2::spi::Mosi{23}, v2::spi::requireHardware()};
    auto bus = h.view.acquire(lcd);
    ASSERT_TRUE(bus.has_value());

    ASSERT_TRUE(h.view.commitBuses().has_value());
    EXPECT_EQ(bus.value()->backendKind(), kHw);
    EXPECT_EQ(bus.value()->controllerId(), 0);
}
