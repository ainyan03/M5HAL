// SPDX-License-Identifier: MIT
#ifndef TEST_V2_NATIVE_BUS_MANAGED_BUS_VIEW_CONTRACT_HPP_
#define TEST_V2_NATIVE_BUS_MANAGED_BUS_VIEW_CONTRACT_HPP_

#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>

namespace m5::hal::v2::test::bus_contract {

constexpr auto kManagedRequire = bus::requireHardware();
constexpr auto kManagedPrefer  = bus::preferHardware();
constexpr auto kManagedAuto    = bus::automatic();
constexpr auto kManagedHw      = types::backend_kind_t::Hardware;
constexpr auto kManagedSw      = types::backend_kind_t::Software;

template <class View, class MakeReq>
void expectThreeBusAllocationThenThirdPromotion(View& view, MakeReq make_req)
{
    auto internal = view.acquire(make_req(0, kManagedRequire));
    auto middle   = view.acquire(make_req(1, kManagedPrefer));
    auto third    = view.acquire(make_req(2, kManagedAuto));
    ASSERT_TRUE(internal.has_value());
    ASSERT_TRUE(middle.has_value());
    ASSERT_TRUE(third.has_value());
    EXPECT_EQ(internal.value()->backendKind(), kManagedSw);
    EXPECT_EQ(middle.value()->backendKind(), kManagedSw);
    EXPECT_EQ(third.value()->backendKind(), kManagedSw);

    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(internal.value()->backendKind(), kManagedHw);
    EXPECT_EQ(internal.value()->controllerId(), 0);
    EXPECT_EQ(middle.value()->backendKind(), kManagedHw);
    EXPECT_EQ(middle.value()->controllerId(), 1);
    EXPECT_EQ(third.value()->backendKind(), kManagedSw);
    EXPECT_EQ(view.hardwareInUse(), 2u);

    const uint32_t internal_gen = internal.value()->backendGeneration();
    const uint32_t middle_gen   = middle.value()->backendGeneration();

    auto third_again = view.acquire(make_req(2, kManagedRequire));
    ASSERT_TRUE(third_again.has_value());
    EXPECT_EQ(third_again.value().get(), third.value().get());

    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(third.value()->backendKind(), kManagedHw);
    EXPECT_EQ(middle.value()->backendKind(), kManagedSw);
    EXPECT_EQ(internal.value()->backendKind(), kManagedHw);
    EXPECT_EQ(internal.value()->controllerId(), 0);
    EXPECT_EQ(view.hardwareInUse(), 2u);
    EXPECT_EQ(internal.value()->backendGeneration(), internal_gen);
    EXPECT_EQ(middle.value()->backendGeneration(), middle_gen + 1);
    EXPECT_EQ(third.value()->controllerId(), 1);
}

template <class View, class MakeReq>
void expectRequireOverSubscriptionFailsWithoutHalfChange(View& view, MakeReq make_req)
{
    auto a = view.acquire(make_req(0, kManagedRequire));
    auto b = view.acquire(make_req(1, kManagedRequire));
    auto c = view.acquire(make_req(2, kManagedRequire));
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    ASSERT_TRUE(c.has_value());

    auto r = view.commitBuses();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(a.value()->backendKind(), kManagedSw);
    EXPECT_EQ(b.value()->backendKind(), kManagedSw);
    EXPECT_EQ(c.value()->backendKind(), kManagedSw);
    EXPECT_EQ(view.hardwareInUse(), 0u);
}

template <class View, class MakeReq>
void expectNoHardwareFactoryKeepsEverythingSoftware(View& view, MakeReq make_req)
{
    auto prefer = view.acquire(make_req(0, kManagedPrefer));
    ASSERT_TRUE(prefer.has_value());
    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(prefer.value()->backendKind(), kManagedSw);

    auto require = view.acquire(make_req(1, kManagedRequire));
    ASSERT_TRUE(require.has_value());
    auto r = view.commitBuses();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), error::error_t::OUT_OF_RESOURCE);
}

template <class View, class MakeReq>
void expectRequireControllerClaimsSpecificController(View& view, MakeReq make_req)
{
    auto a = view.acquire(make_req(0, bus::requireController(1)));
    ASSERT_TRUE(a.has_value());

    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(a.value()->backendKind(), kManagedHw);
    EXPECT_EQ(a.value()->controllerId(), 1);
}

template <class View, class MakeReq, class MakeTypedConfig>
void expectTypedAcquireIsNotManagedByCommit(View& view, MakeReq make_req, MakeTypedConfig make_typed_config)
{
    auto managed_bus = view.acquire(make_req(0, kManagedRequire));
    auto typed_bus   = view.acquire(make_typed_config());
    ASSERT_TRUE(managed_bus.has_value());
    ASSERT_TRUE(typed_bus.has_value());

    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(managed_bus.value()->backendKind(), kManagedHw);
    EXPECT_EQ(typed_bus.value()->backendKind(), kManagedSw);
    EXPECT_EQ(typed_bus.value()->backendGeneration(), 0u);
    EXPECT_EQ(view.hardwareInUse(), 1u);
}

template <class View, class MakeReq, class MakeTypedHwConfig>
void expectUnmanagedHardwareBusReservesItsController(View& view, MakeReq make_req, MakeTypedHwConfig make_hw_config)
{
    auto pinned  = view.acquire(make_hw_config());
    auto req_bus = view.acquire(make_req(0, kManagedRequire));
    ASSERT_TRUE(pinned.has_value());
    ASSERT_TRUE(req_bus.has_value());

    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(req_bus.value()->backendKind(), kManagedHw);
    EXPECT_EQ(req_bus.value()->controllerId(), 1);
    EXPECT_EQ(pinned.value()->backendKind(), kManagedHw);
    EXPECT_EQ(pinned.value()->controllerId(), 0);
    EXPECT_EQ(pinned.value()->backendGeneration(), 0u);
    EXPECT_EQ(view.hardwareInUse(), 2u);
}

template <class View, class MakeReq>
void expectSoftwareForbidKeepsHardwareOff(View& view, MakeReq make_req)
{
    auto forced = view.acquire(make_req(0, bus::software()));
    auto needs  = view.acquire(make_req(1, kManagedRequire));
    ASSERT_TRUE(forced.has_value());
    ASSERT_TRUE(needs.has_value());

    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(forced.value()->backendKind(), kManagedSw);
    EXPECT_EQ(needs.value()->backendKind(), kManagedHw);
    EXPECT_EQ(view.hardwareInUse(), 1u);
}

template <class View, class MakeReq>
void expectPreferControllerTakesRequestedWhenFree(View& view, MakeReq make_req)
{
    auto a = view.acquire(make_req(0, bus::preferController(1)));
    ASSERT_TRUE(a.has_value());

    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(a.value()->backendKind(), kManagedHw);
    EXPECT_EQ(a.value()->controllerId(), 1);
}

template <class View, class MakeReq>
void expectPreferControllerFallsBackWhenTaken(View& view, MakeReq make_req)
{
    auto pinned = view.acquire(make_req(0, bus::requireController(1)));
    auto pref   = view.acquire(make_req(1, bus::preferController(1)));
    ASSERT_TRUE(pinned.has_value());
    ASSERT_TRUE(pref.has_value());

    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(pinned.value()->controllerId(), 1);
    EXPECT_EQ(pref.value()->backendKind(), kManagedHw);
    EXPECT_EQ(pref.value()->controllerId(), 0);
}

template <class View, class MakeReq>
void expectRequireAndForbidConflictIsInvalid(View& view, MakeReq make_req)
{
    types::AllocationIntent bad;
    bad.require = types::backend_caps::HARDWARE;
    bad.forbid  = types::backend_caps::HARDWARE;
    EXPECT_FALSE(bad.valid());

    // D3/F5: an invalid intent is rejected up front at BusView acquire, not
    // deferred to commitBuses -- so the bus is never interned and no controller
    // is leased.
    auto a = view.acquire(make_req(0, bad));
    ASSERT_FALSE(a.has_value());
    EXPECT_EQ(a.error(), error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(view.hardwareInUse(), 0u);
}

template <class View, class MakeReq>
void expectNegativeSpecificControllerIsInvalid(View& view, MakeReq make_req)
{
    // D3/F6: Require/Prefer with a negative controller_id is an impossible
    // request, not "any controller". valid() rejects it and acquire fails up
    // front instead of silently relaxing it to a generic hardware request.
    types::AllocationIntent req_bad;
    req_bad.controller_mode = types::ControllerMode::Require;
    req_bad.controller_id   = -1;
    EXPECT_FALSE(req_bad.valid());
    auto a = view.acquire(make_req(0, req_bad));
    ASSERT_FALSE(a.has_value());
    EXPECT_EQ(a.error(), error::error_t::INVALID_ARGUMENT);

    types::AllocationIntent pref_bad;
    pref_bad.controller_mode = types::ControllerMode::Prefer;
    pref_bad.controller_id   = -1;
    EXPECT_FALSE(pref_bad.valid());
    auto b = view.acquire(make_req(1, pref_bad));
    ASSERT_FALSE(b.has_value());
    EXPECT_EQ(b.error(), error::error_t::INVALID_ARGUMENT);

    EXPECT_EQ(view.hardwareInUse(), 0u);
}

template <class View, class MakeReq>
void expectDeterministicTieBreakLowestController(View& view, MakeReq make_req)
{
    auto first  = view.acquire(make_req(0, kManagedRequire));
    auto second = view.acquire(make_req(1, kManagedRequire));
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());

    ASSERT_TRUE(view.commitBuses().has_value());
    EXPECT_EQ(first.value()->controllerId(), 0);
    EXPECT_EQ(second.value()->controllerId(), 1);
}

}  // namespace m5::hal::v2::test::bus_contract

#endif  // TEST_V2_NATIVE_BUS_MANAGED_BUS_VIEW_CONTRACT_HPP_
