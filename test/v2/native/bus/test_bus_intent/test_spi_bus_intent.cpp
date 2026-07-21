// SPDX-License-Identifier: MIT
#include "../managed_bus_view_contract.hpp"

#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/bus/local_backend.hpp>
#include <gtest/gtest.h>

#include <new>

// intent-driven hardware allocation for SPI. These tests
// drive the resolver (spi::BusView::commitBuses) over a LOCAL registry with
// injected fake factories + a 2-controller silicon budget, mirroring the i2c
// intent tests (test_bus_intent.cpp). The fakes perform no I/O and never touch
// GPIO, so the test is about allocation, the controller assignments, and the
// swap generations — the real software/espidf wire paths are covered elsewhere.
//
// SPI is normally at i2c's degenerate point (uniform controllers + a software
// placeholder). The shared allocation contract stays in the common tests;
// SPI-specific feature filtering is exercised near the end of this file.

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
v2::spi::IBus* fakeSwFactory(const v2::bus::LocalResourceContext& /*resources*/,
                             const v2::spi::LogicalBusConfig& /*logical*/)
{
    return new (std::nothrow) FakeBackend(v2::types::backend_kind_t::Software, -1);
}
v2::spi::IBus* fakeHwFactory(const v2::bus::LocalResourceContext& /*resources*/,
                             const v2::spi::LogicalBusConfig& /*logical*/, int8_t controller)
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

    SpiIntentHarness(Adapter::SwFactory sw, Adapter::HwFactory hw = nullptr, uint8_t hw_capacity = 0,
                     Adapter::Topology topology = {})
        : adapter{backend.busRegistry(), sw, hw, hw_capacity, topology}, view{&backend}
    {
        backend.registerKind(adapter);
    }
};

v2::types::backend_caps_t hardwareWithoutSharedRx(int8_t)
{
    return v2::spi::caps::HARDWARE;
}

SpiIntentHarness::Adapter::Topology topologyWithoutSharedRx(void)
{
    SpiIntentHarness::Adapter::Topology topology;
    topology.controller_caps = &hardwareWithoutSharedRx;
    return topology;
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

TEST(SpiBusIntent, BusViewClaimControllerSurfaceRoundTrips)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto first = h.view.claimController();
    ASSERT_TRUE(first.has_value()) << "err=" << v2::error::toString(first.error());
    EXPECT_EQ(first.value(), 0);

    auto second = h.view.claimController();
    ASSERT_TRUE(second.has_value()) << "err=" << v2::error::toString(second.error());
    EXPECT_EQ(second.value(), 1);

    auto exhausted = h.view.claimController();
    ASSERT_FALSE(exhausted.has_value());
    EXPECT_EQ(exhausted.error(), v2::error::error_t::OUT_OF_RESOURCE);

    auto released = h.view.releaseClaimedController(first.value());
    ASSERT_TRUE(released.has_value()) << "err=" << v2::error::toString(released.error());
    auto reused = h.view.claimController();
    ASSERT_TRUE(reused.has_value()) << "err=" << v2::error::toString(reused.error());
    EXPECT_EQ(reused.value(), 0);
}

TEST(SpiBusIntent, ExternalClaimSurvivesCommitAndMasterAutoAvoidsIt)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto claim = h.view.claimController();
    ASSERT_TRUE(claim.has_value()) << "err=" << v2::error::toString(claim.error());
    ASSERT_EQ(claim.value(), 0);

    auto bus = h.view.acquire(reqByIndex(0, v2::spi::automatic()));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    auto committed = h.view.commitBuses();
    ASSERT_TRUE(committed.has_value()) << "err=" << v2::error::toString(committed.error());
    EXPECT_EQ(bus.value()->backendKind(), kHw);
    EXPECT_EQ(bus.value()->controllerId(), 1);
}

TEST(SpiBusIntent, MasterRequireClaimedControllerFailsOutOfResource)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto claim = h.view.claimController(v2::spi::requireController(0));
    ASSERT_TRUE(claim.has_value()) << "err=" << v2::error::toString(claim.error());

    auto bus = h.view.acquire(reqByIndex(0, v2::spi::requireController(0)));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    auto committed = h.view.commitBuses();
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), v2::error::error_t::OUT_OF_RESOURCE);
}

TEST(SpiBusIntent, ReleaseClaimedControllerAllowsMasterToUseIt)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto claim = h.view.claimController(v2::spi::requireController(0));
    ASSERT_TRUE(claim.has_value()) << "err=" << v2::error::toString(claim.error());
    auto released = h.view.releaseClaimedController(claim.value());
    ASSERT_TRUE(released.has_value()) << "err=" << v2::error::toString(released.error());

    auto bus = h.view.acquire(reqByIndex(0, v2::spi::requireController(0)));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    auto committed = h.view.commitBuses();
    ASSERT_TRUE(committed.has_value()) << "err=" << v2::error::toString(committed.error());
    EXPECT_EQ(bus.value()->backendKind(), kHw);
    EXPECT_EQ(bus.value()->controllerId(), 0);
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

TEST(SpiFeatureIntent, HelperComposesWithHardwarePreference)
{
    const auto intent = v2::spi::requireMosiSharedRx(v2::spi::preferHardware());

    EXPECT_EQ(intent.require, v2::spi::caps::MOSI_SHARED_RX);
    EXPECT_EQ(intent.prefer, v2::spi::caps::HARDWARE);
    EXPECT_EQ(intent.forbid, 0u);
}

TEST(SpiFeatureIntent, CapableHardwareIsSelected)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/1};
    auto bus = h.view.acquire(reqByIndex(0, v2::spi::requireMosiSharedRx()));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());

    auto committed = h.view.commitBuses(0);
    ASSERT_TRUE(committed.has_value()) << "err=" << v2::error::toString(committed.error());
    EXPECT_EQ(bus.value()->backendKind(), kHw);
    EXPECT_EQ(bus.value()->controllerId(), 0);
}

TEST(SpiFeatureIntent, UnsupportedHardwareFallsBackToSoftware)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/1, topologyWithoutSharedRx()};
    auto bus = h.view.acquire(reqByIndex(0, v2::spi::requireMosiSharedRx(v2::spi::preferHardware())));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());

    auto committed = h.view.commitBuses(0);
    ASSERT_TRUE(committed.has_value()) << "err=" << v2::error::toString(committed.error());
    EXPECT_EQ(bus.value()->backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(bus.value()->controllerId(), -1);
}

TEST(SpiFeatureIntent, RequiredHardwareDoesNotFallBackWhenFeatureIsUnsupported)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/1, topologyWithoutSharedRx()};
    auto bus = h.view.acquire(reqByIndex(0, v2::spi::requireMosiSharedRx(v2::spi::requireHardware())));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());

    auto committed = h.view.commitBuses(0);
    ASSERT_FALSE(committed.has_value());
    EXPECT_EQ(committed.error(), v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(bus.value()->backendKind(), v2::types::backend_kind_t::Software);
}

TEST(SpiFeatureIntent, ReacquireDemotesUnsupportedHardwareBeforeUse)
{
    SpiIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/1, topologyWithoutSharedRx()};
    auto bus = h.view.acquire(reqByIndex(0, v2::spi::automatic()));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    auto first_commit = h.view.commitBuses(0);
    ASSERT_TRUE(first_commit.has_value()) << "err=" << v2::error::toString(first_commit.error());
    ASSERT_EQ(bus.value()->backendKind(), kHw);
    const uint32_t hardware_generation = bus.value()->backendGeneration();

    auto retagged = h.view.acquire(reqByIndex(0, v2::spi::requireMosiSharedRx()));
    ASSERT_TRUE(retagged.has_value()) << "err=" << v2::error::toString(retagged.error());
    ASSERT_EQ(retagged.value().get(), bus.value().get());
    auto second_commit = h.view.commitBuses(0);
    ASSERT_TRUE(second_commit.has_value()) << "err=" << v2::error::toString(second_commit.error());

    EXPECT_EQ(bus.value()->backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_GT(bus.value()->backendGeneration(), hardware_generation);
}
