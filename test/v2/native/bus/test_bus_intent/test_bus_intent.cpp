// SPDX-License-Identifier: MIT
#include "../managed_bus_view_contract.hpp"

#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/bus/local_backend.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <new>
#include <string>
#include <vector>

// intent-driven hardware allocation. These tests drive the
// resolver (i2c::BusView::commitBuses) over a LOCAL registry with injected fake
// factories + a 2-controller silicon budget, reproducing the M5StickC
// three-bus case (internal / PortA / HAT) without real hardware. The assertions
// are on allocation, the controller assignments, and the swap generations; the
// real software backend's wire path is covered by the test_bus_registry
// software-I2C tests, so the fakes here perform no I/O and keep the resolver
// test independent of GPIO pin resolution.

// Test-local config + backend for the TYPED acquire<CfgT> path (an explicit
// backend choice). The BackendFor specialization lets `BusView::acquire(cfg)`
// build it through `Bus::init<CfgT>`; such a bus is NOT intent-managed, so
// commitBuses() must leave it on this backend.
namespace m5::hal::v2::i2c {
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
}  // namespace m5::hal::v2::i2c

namespace {
namespace v2 = m5::hal::v2;

// A fake backend with a fixed kind + controller, no I/O. As hardware it reports
// its leased controller; as software it has none -- enough for the resolver and
// the facade's query API to treat it correctly.
class FakeBackend : public v2::i2c::IBus {
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
        return _kind == v2::types::backend_kind_t::Hardware ? 400000u : 0u;
    }

private:
    v2::types::backend_kind_t _kind;
    int8_t _controller;
};

// Factories injected into the test BusView. The hardware factory plays the role
// espidf fills in a real build; the software factory plays the always-present
// bit-bang fallback. Neither touches GPIO (this test is about allocation).
v2::i2c::IBus* fakeSwFactory(const v2::i2c::LogicalBusConfig& /*logical*/)
{
    return new (std::nothrow) FakeBackend(v2::types::backend_kind_t::Software, -1);
}
v2::i2c::IBus* fakeHwFactory(const v2::i2c::LogicalBusConfig& /*logical*/, int8_t controller)
{
    return new (std::nothrow) FakeBackend(v2::types::backend_kind_t::Hardware, controller);
}

v2::i2c::LogicalBusConfig req(v2::types::gpio_number_t scl, v2::types::gpio_number_t sda,
                              v2::types::AllocationIntent intent)
{
    return v2::i2c::LogicalBusConfig{v2::i2c::Scl{scl}, v2::i2c::Sda{sda}, intent};
}

constexpr auto kHw = v2::types::backend_kind_t::Hardware;
constexpr auto kSw = v2::types::backend_kind_t::Software;

v2::i2c::LogicalBusConfig reqByIndex(size_t index, v2::types::AllocationIntent intent)
{
    static constexpr v2::types::gpio_number_t pins[][2] = {
        {22, 21},
        {33, 32},
        {44, 43},
    };
    return req(pins[index][0], pins[index][1], intent);
}

struct I2cIntentHarness {
    using Adapter = v2::bus::LocalKindAdapter<v2::i2c::BusTraits>;

    v2::bus::LocalBackend backend;
    Adapter adapter;
    v2::i2c::BusView view;

    I2cIntentHarness(Adapter::SwFactory sw, Adapter::HwFactory hw = nullptr, uint8_t hw_capacity = 0)
        : adapter{backend.busRegistry(), sw, hw, hw_capacity}, view{&backend}
    {
        backend.registerKind(adapter);
    }
};

v2::i2c::TypedFakeConfig makeTypedFakeConfig(void)
{
    v2::i2c::TypedFakeConfig typed;
    typed.pin_scl = 33;
    typed.pin_sda = 32;
    return typed;
}

v2::i2c::TypedFakeHwConfig makeTypedFakeHwConfig(void)
{
    v2::i2c::TypedFakeHwConfig pinned_cfg;
    pinned_cfg.pin_scl = 10;
    pinned_cfg.pin_sda = 11;
    return pinned_cfg;
}

}  // namespace

TEST(I2cBusIntent, ThreeBusAllocationThenHatPromotion)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectThreeBusAllocationThenThirdPromotion(h.view, &reqByIndex);
}

TEST(I2cBusIntent, RequireOverSubscriptionFailsWithoutHalfChange)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectRequireOverSubscriptionFailsWithoutHalfChange(h.view, &reqByIndex);
}

TEST(I2cBusIntent, NoHardwareFactoryKeepsEverythingSoftware)
{
    // A software-only / host build: no hardware factory, zero silicon budget.
    I2cIntentHarness h{&fakeSwFactory};
    v2::test::bus_contract::expectNoHardwareFactoryKeepsEverythingSoftware(h.view, &reqByIndex);
}

TEST(I2cBusIntent, RequireControllerClaimsSpecificController)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectRequireControllerClaimsSpecificController(h.view, &reqByIndex);
}

TEST(I2cBusIntent, TypedAcquireIsNotManagedByCommit)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectTypedAcquireIsNotManagedByCommit(h.view, &reqByIndex, &makeTypedFakeConfig);
}

TEST(I2cBusIntent, UnmanagedHardwareBusReservesItsController)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectUnmanagedHardwareBusReservesItsController(h.view, &reqByIndex,
                                                                            &makeTypedFakeHwConfig);
}

// --- New capability-model behaviours (Phase A) -------------------------------

TEST(I2cBusIntent, SoftwareForbidKeepsHardwareOff)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectSoftwareForbidKeepsHardwareOff(h.view, &reqByIndex);
}

TEST(I2cBusIntent, PreferControllerTakesRequestedWhenFree)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectPreferControllerTakesRequestedWhenFree(h.view, &reqByIndex);
}

TEST(I2cBusIntent, PreferControllerFallsBackWhenTaken)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectPreferControllerFallsBackWhenTaken(h.view, &reqByIndex);
}

TEST(I2cBusIntent, RequireAndForbidConflictIsInvalid)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectRequireAndForbidConflictIsInvalid(h.view, &reqByIndex);
}

TEST(I2cBusIntent, NegativeSpecificControllerIsInvalid)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectNegativeSpecificControllerIsInvalid(h.view, &reqByIndex);
}

TEST(I2cBusIntent, DeterministicTieBreakLowestController)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    v2::test::bus_contract::expectDeterministicTieBreakLowestController(h.view, &reqByIndex);
}

// --- External claim x commitBuses (a caller outside the intent resolver,
// e.g. a standalone slave, claims a controller through
// AllocationCore::claimController -- the resolver must treat it as occupied
// across commits, exactly like an unmanaged hardware bus's controller). ---

TEST(I2cBusIntent, ExternalClaimSurvivesCommitAndMasterAutoAvoidsIt)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto* core = h.adapter.allocationCore();
    auto claim = core->claimController(v2::bus::automatic());
    ASSERT_TRUE(claim.has_value()) << "err=" << v2::error::toString(claim.error());
    EXPECT_EQ(claim.value(), 0);  // Auto: lowest free controller

    auto bus = h.view.acquire(reqByIndex(0, v2::bus::automatic()));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    ASSERT_TRUE(h.view.commitBuses().has_value());
    // The claim must survive commitBuses()'s pool rebuild (_syncPoolFromLive
    // -> releaseAll) and keep the master's automatic request off controller 0.
    EXPECT_EQ(bus.value()->backendKind(), kHw);
    EXPECT_EQ(bus.value()->controllerId(), 1);
}

TEST(I2cBusIntent, MasterRequireClaimedControllerFailsOutOfResource)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto* core = h.adapter.allocationCore();
    auto claim = core->claimController(v2::bus::automatic());
    ASSERT_TRUE(claim.has_value()) << "err=" << v2::error::toString(claim.error());
    ASSERT_EQ(claim.value(), 0);

    auto bus = h.view.acquire(reqByIndex(0, v2::i2c::requireController(0)));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    auto res = h.view.commitBuses();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error(), v2::error::error_t::OUT_OF_RESOURCE);
}

TEST(I2cBusIntent, BusViewClaimControllerSurfaceRoundTrips)
{
    // Exercises the public BusView::claimController/releaseClaimedController
    // surface (BusViewCore -> IHalBackend -> LocalBackend -> AllocationCore)
    // rather than reaching into AllocationCore directly, matching how a real
    // consumer (e.g. a standalone slave) would call it.
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto claim = h.view.claimController();  // omitted intent = Auto
    ASSERT_TRUE(claim.has_value()) << "err=" << v2::error::toString(claim.error());
    EXPECT_EQ(claim.value(), 0);

    auto claim2 = h.view.claimController();
    ASSERT_TRUE(claim2.has_value()) << "err=" << v2::error::toString(claim2.error());
    EXPECT_EQ(claim2.value(), 1);

    // Pool exhausted: a third Auto claim fails.
    auto claim3 = h.view.claimController();
    ASSERT_FALSE(claim3.has_value());
    EXPECT_EQ(claim3.error(), v2::error::error_t::OUT_OF_RESOURCE);

    ASSERT_TRUE(h.view.releaseClaimedController(claim.value()).has_value());
    auto claim4 = h.view.claimController();
    ASSERT_TRUE(claim4.has_value()) << "err=" << v2::error::toString(claim4.error());
    EXPECT_EQ(claim4.value(), 0);  // freed index reused
}

TEST(I2cBusIntent, ReleaseClaimedControllerAllowsMasterToUseIt)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto* core = h.adapter.allocationCore();
    auto claim = core->claimController(v2::bus::automatic());
    ASSERT_TRUE(claim.has_value()) << "err=" << v2::error::toString(claim.error());
    ASSERT_EQ(claim.value(), 0);
    ASSERT_TRUE(core->releaseClaimedController(claim.value()).has_value());

    auto bus = h.view.acquire(reqByIndex(0, v2::i2c::requireController(0)));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    ASSERT_TRUE(h.view.commitBuses().has_value());
    EXPECT_EQ(bus.value()->backendKind(), kHw);
    EXPECT_EQ(bus.value()->controllerId(), 0);
}

TEST(I2cBusIntent, ClaimControllerRequireSeparatesConfigErrorFromShortage)
{
    // Require(id) error taxonomy: naming a controller that does not exist is
    // a configuration error (INVALID_ARGUMENT), while naming one that exists
    // but is already held is a resource shortage (OUT_OF_RESOURCE).
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto* core = h.adapter.allocationCore();

    auto oor = core->claimController(v2::i2c::requireController(2));  // capacity is 2: ids are 0..1
    ASSERT_FALSE(oor.has_value());
    EXPECT_EQ(oor.error(), v2::error::error_t::INVALID_ARGUMENT);

    auto first = core->claimController(v2::i2c::requireController(1));
    ASSERT_TRUE(first.has_value()) << "err=" << v2::error::toString(first.error());
    EXPECT_EQ(first.value(), 1);
    auto busy = core->claimController(v2::i2c::requireController(1));
    ASSERT_FALSE(busy.has_value());
    EXPECT_EQ(busy.error(), v2::error::error_t::OUT_OF_RESOURCE);
}

// --- Shared-core seam paths that i2c never reaches (Phase B) ------------------
// A software-LESS, NON-uniform-capability fake kind drives the AllocationCore
// branches i2c leaves dead: a null placeholder (demote -> swapPending -> pending)
// and the capability eligibility filter. This validates the seam with a second
// kind before a real i2s backend exists.
namespace {

constexpr v2::types::backend_caps_t kFakeDac = 1u << 1;  // a kind-local capability bit

struct FakeCoreConfig : public v2::bus::IBusConfig {
    FakeCoreConfig(void) : v2::bus::IBusConfig{v2::types::bus_kind_t::I2S}
    {
    }
};

// The hardware backend the fake kind's factory hands back (reports its controller).
class FakeCoreHwBackend : public v2::bus::IBus {
public:
    explicit FakeCoreHwBackend(int8_t controller) : _controller{controller}
    {
    }
    const v2::bus::IBusConfig& getConfig(void) const override
    {
        return _cfg;
    }
    v2::types::backend_kind_t backendKind(void) const override
    {
        return v2::types::backend_kind_t::Hardware;
    }
    int8_t controllerId(void) const override
    {
        return _controller;
    }

private:
    FakeCoreConfig _cfg;
    int8_t _controller;
};

// A managed bus that is both the query surface (IBus) and the resolver surface
// (IManagedBus); it tracks its backend state across swaps so the resolver reads
// the right kind/controller. A software-less kind's demote lands it pending.
class FakeManagedBus : public v2::bus::IBus, public v2::bus::IManagedBus {
public:
    // `wiring` is a fake stand-in for pins (this generic seam has no
    // per-kind pin model): a pin-domain-restricted kind's
    // `controllerAcceptsBus()` compares it against a fixed value instead of
    // real GPIO numbers. Defaulted so every pre-existing call site (which
    // does not care about pin domains) is unaffected.
    explicit FakeManagedBus(v2::types::AllocationIntent intent, int wiring = 0) : _intent{intent}, _wiring{wiring}
    {
    }
    int wiring(void) const
    {
        return _wiring;
    }
    const v2::bus::IBusConfig& getConfig(void) const override
    {
        return _cfg;
    }
    v2::types::backend_kind_t backendKind(void) const override
    {
        return _kind;
    }
    int8_t controllerId(void) const override
    {
        return _controller;
    }
    uint32_t backendGeneration(void) const override
    {
        return _generation;
    }
    const v2::types::AllocationIntent& intent(void) const override
    {
        return _intent;
    }
    bool managed(void) const override
    {
        return true;
    }
    v2::result_t<void> swapBackend(std::unique_ptr<v2::bus::IBus> backend, uint32_t) override
    {
        _kind       = backend->backendKind();
        _controller = backend->controllerId();
        ++_generation;
        if (_fail_next_swap_backend_after_apply) {
            _fail_next_swap_backend_after_apply = false;
            return m5::stl::make_unexpected(v2::error::error_t::IO_ERROR);
        }
        return {};
    }
    v2::result_t<void> swapPending(uint32_t) override
    {
        _kind       = v2::types::backend_kind_t::Software;
        _controller = -1;
        ++_generation;
        return {};
    }
    void failNextSwapBackendAfterApply(void)
    {
        _fail_next_swap_backend_after_apply = true;
    }

private:
    FakeCoreConfig _cfg;
    v2::types::AllocationIntent _intent;
    int _wiring                              = 0;
    v2::types::backend_kind_t _kind          = v2::types::backend_kind_t::Software;
    int8_t _controller                       = -1;
    uint32_t _generation                     = 0;
    bool _fail_next_swap_backend_after_apply = false;
};

// A kind with NO software placeholder and NON-equivalent controllers: only
// controller 0 offers the (fake) DAC capability.
class FakeSwlessKind : public v2::bus::IAllocationKind {
public:
    explicit FakeSwlessKind(uint8_t capacity) : _capacity{capacity}
    {
    }
    v2::types::bus_kind_t kind(void) const override
    {
        return v2::types::bus_kind_t::I2S;
    }
    uint8_t controllerCapacity(void) const override
    {
        return _capacity;
    }
    bool hasHardware(void) const override
    {
        return true;
    }
    v2::bus::IManagedBus& toManaged(v2::bus::IBus& bus) const override
    {
        return static_cast<FakeManagedBus&>(bus);
    }
    v2::bus::IBus* makePlaceholder(v2::bus::IManagedBus&) const override
    {
        return nullptr;  // software-less: a demoted bus becomes pending
    }
    v2::bus::IBus* makeHardware(v2::bus::IManagedBus&, int8_t controller) const override
    {
        return new (std::nothrow) FakeCoreHwBackend(controller);
    }
    // Fake commit*: this stub has no real facade lock, so it just builds + swaps
    // (the production AllocationKind builds under the lock via swapBackendWith).
    // The fake bus's swapBackend/swapPending still record the result, so the
    // allocation-logic assertions (and failNextSwapBackendAfterApply) are intact.
    v2::result_t<void> commitPlaceholder(v2::bus::IManagedBus& mb, uint32_t timeout_ms) const override
    {
        v2::bus::IBus* raw = makePlaceholder(mb);
        return raw ? mb.swapBackend(std::unique_ptr<v2::bus::IBus>(raw), timeout_ms) : mb.swapPending(timeout_ms);
    }
    v2::result_t<void> commitHardware(v2::bus::IManagedBus& mb, int8_t controller, uint32_t timeout_ms) const override
    {
        v2::bus::IBus* raw = makeHardware(mb, controller);
        if (raw == nullptr) {
            return m5::stl::make_unexpected(v2::error::error_t::OUT_OF_RESOURCE);
        }
        return mb.swapBackend(std::unique_ptr<v2::bus::IBus>(raw), timeout_ms);
    }
    bool uniformControllers(void) const override
    {
        return false;  // controllers differ -> the eligibility filter is consulted
    }
    v2::types::backend_caps_t controllerCaps(int8_t controller) const override
    {
        return controller == 0 ? (v2::types::backend_caps::HARDWARE | kFakeDac) : v2::types::backend_caps::HARDWARE;
    }

private:
    uint8_t _capacity;
};

FakeManagedBus* internFake(v2::bus::BusRegistry& reg, std::shared_ptr<v2::bus::IBus>& hold,
                           v2::types::gpio_number_t pin, v2::types::AllocationIntent intent, int wiring = 0)
{
    v2::bus::IdentityKey id;
    id.pins[0] = pin;
    id.pins[1] = static_cast<v2::types::gpio_number_t>(pin + 100);
    auto r     = reg.acquireOrFind(
        v2::types::bus_kind_t::I2S, id, [&intent, wiring]() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
            return std::shared_ptr<v2::bus::IBus>(new (std::nothrow) FakeManagedBus(intent, wiring));
        });
    hold = r.value();
    return static_cast<FakeManagedBus*>(hold.get());
}

v2::types::AllocationIntent autoIntent(void)
{
    return v2::types::AllocationIntent{};
}
v2::types::AllocationIntent requireHw(void)
{
    v2::types::AllocationIntent a;
    a.require = v2::types::backend_caps::HARDWARE;
    return a;
}
v2::types::AllocationIntent requireHwDac(void)
{
    v2::types::AllocationIntent a;
    a.require = v2::types::backend_caps::HARDWARE | kFakeDac;
    return a;
}
v2::types::AllocationIntent requireCtrl(int8_t c)
{
    v2::types::AllocationIntent a;
    a.require         = v2::types::backend_caps::HARDWARE;
    a.controller_id   = c;
    a.controller_mode = v2::types::ControllerMode::Require;
    return a;
}

}  // namespace

TEST(AllocationCoreSeam, SoftwareLessDemoteGoesPending)
{
    v2::bus::BusRegistry reg;
    FakeSwlessKind kind{/*capacity=*/1};
    v2::bus::AllocationCore core{reg, kind};

    // Bus A (automatic) takes the only controller.
    std::shared_ptr<v2::bus::IBus> ha;
    auto* a = internFake(reg, ha, 10, autoIntent());
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(a->backendKind(), kHw);
    EXPECT_EQ(a->controllerId(), 0);

    // Bus B requires controller 0; A must yield it. With no software placeholder,
    // A becomes PENDING (Software / -1), not a software backend.
    std::shared_ptr<v2::bus::IBus> hb;
    auto* b = internFake(reg, hb, 20, requireCtrl(0));
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(b->backendKind(), kHw);
    EXPECT_EQ(b->controllerId(), 0);
    EXPECT_EQ(a->backendKind(), kSw);  // pending: detached, no controller
    EXPECT_EQ(a->controllerId(), -1);
    EXPECT_EQ(core.hardwareInUse(), 1u);
}

TEST(AllocationCoreSeam, CapabilityFilterRoutesAndLimits)
{
    v2::bus::BusRegistry reg;
    FakeSwlessKind kind{/*capacity=*/2};
    v2::bus::AllocationCore core{reg, kind};

    // X needs DAC (only controller 0 has it); Y just needs hardware.
    std::shared_ptr<v2::bus::IBus> hx, hy;
    auto* x = internFake(reg, hx, 30, requireHwDac());
    auto* y = internFake(reg, hy, 40, requireHw());
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(x->controllerId(), 0);  // the eligibility filter routes DAC to controller 0
    EXPECT_EQ(y->controllerId(), 1);

    // A second DAC bus cannot be satisfied (only one DAC controller, held by X).
    std::shared_ptr<v2::bus::IBus> hz;
    auto* z = internFake(reg, hz, 50, requireHwDac());
    (void)z;
    auto r = core.commitBuses();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(x->controllerId(), 0);  // incumbent DAC bus kept its controller
}

TEST(AllocationCoreSeam, PoolResyncsAfterPartialPromoteError)
{
    v2::bus::BusRegistry reg;
    FakeSwlessKind kind{/*capacity=*/1};
    v2::bus::AllocationCore core{reg, kind};

    std::shared_ptr<v2::bus::IBus> hold;
    auto* bus = internFake(reg, hold, 60, requireHw());
    bus->failNextSwapBackendAfterApply();

    auto r = core.commitBuses();
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), v2::error::error_t::IO_ERROR);
    EXPECT_EQ(bus->backendKind(), kHw);
    EXPECT_EQ(bus->controllerId(), 0);
    EXPECT_EQ(core.hardwareInUse(), 1u);
}

// --- Opt-in capability filtering (low-power domain controllers) -------------
// A kind can mark a capability bit "opt-in": a controller offering it is
// excluded from automatic allocation, and only chosen when an intent names
// the bit (require/prefer) or the controller itself (ControllerMode). This
// models a controller that lives in a restricted domain (e.g. a low-power
// peripheral instance) and must never be handed out to a generic request.
namespace {

// Non-uniform, 3-controller kind: controllers 0/1 offer plain HARDWARE,
// controller 2 additionally offers the opt-in LOW_POWER bit.
class FakeOptInKind : public v2::bus::IAllocationKind {
public:
    explicit FakeOptInKind(uint8_t capacity) : _capacity{capacity}
    {
    }
    v2::types::bus_kind_t kind(void) const override
    {
        return v2::types::bus_kind_t::I2S;
    }
    uint8_t controllerCapacity(void) const override
    {
        return _capacity;
    }
    bool hasHardware(void) const override
    {
        return true;
    }
    v2::bus::IManagedBus& toManaged(v2::bus::IBus& bus) const override
    {
        return static_cast<FakeManagedBus&>(bus);
    }
    v2::bus::IBus* makePlaceholder(v2::bus::IManagedBus&) const override
    {
        return nullptr;  // software-less: a demoted bus becomes pending
    }
    v2::bus::IBus* makeHardware(v2::bus::IManagedBus&, int8_t controller) const override
    {
        return new (std::nothrow) FakeCoreHwBackend(controller);
    }
    v2::result_t<void> commitPlaceholder(v2::bus::IManagedBus& mb, uint32_t timeout_ms) const override
    {
        v2::bus::IBus* raw = makePlaceholder(mb);
        return raw ? mb.swapBackend(std::unique_ptr<v2::bus::IBus>(raw), timeout_ms) : mb.swapPending(timeout_ms);
    }
    v2::result_t<void> commitHardware(v2::bus::IManagedBus& mb, int8_t controller, uint32_t timeout_ms) const override
    {
        v2::bus::IBus* raw = makeHardware(mb, controller);
        if (raw == nullptr) {
            return m5::stl::make_unexpected(v2::error::error_t::OUT_OF_RESOURCE);
        }
        return mb.swapBackend(std::unique_ptr<v2::bus::IBus>(raw), timeout_ms);
    }
    bool uniformControllers(void) const override
    {
        return false;
    }
    v2::types::backend_caps_t controllerCaps(int8_t controller) const override
    {
        return controller == 2 ? (v2::types::backend_caps::HARDWARE | v2::types::backend_caps::LOW_POWER)
                               : v2::types::backend_caps::HARDWARE;
    }
    v2::types::backend_caps_t optInCaps(void) const override
    {
        return v2::types::backend_caps::LOW_POWER;
    }

private:
    uint8_t _capacity;
};

}  // namespace

TEST(AllocationCoreSeam, AutomaticIntentNeverPicksOptInController)
{
    v2::bus::BusRegistry reg;
    FakeOptInKind kind{/*capacity=*/3};
    v2::bus::AllocationCore core{reg, kind};

    std::shared_ptr<v2::bus::IBus> hp, hq, hr;
    auto* p = internFake(reg, hp, 70, autoIntent());
    auto* q = internFake(reg, hq, 80, autoIntent());
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(p->controllerId(), 0);
    EXPECT_EQ(q->controllerId(), 1);

    // Controllers 0 and 1 are both taken; the opt-in controller 2 is free but
    // must not be handed to a plain automatic request.
    auto* r = internFake(reg, hr, 90, autoIntent());
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(r->backendKind(), kSw);  // stays on the placeholder, not controller 2
    EXPECT_EQ(r->controllerId(), -1);
    EXPECT_EQ(p->controllerId(), 0);
    EXPECT_EQ(q->controllerId(), 1);
}

TEST(AllocationCoreSeam, PlainRequireHardwareFailsRatherThanPickingOptInController)
{
    v2::bus::BusRegistry reg;
    FakeOptInKind kind{/*capacity=*/3};
    v2::bus::AllocationCore core{reg, kind};

    std::shared_ptr<v2::bus::IBus> hp, hq, hr;
    auto* p = internFake(reg, hp, 70, requireHw());
    auto* q = internFake(reg, hq, 80, requireHw());
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(p->controllerId(), 0);
    EXPECT_EQ(q->controllerId(), 1);

    // Controller 2 is free but opt-in only: a plain requireHardware() request
    // must fail exactly as it would if no hardware controller were free at all.
    auto* r = internFake(reg, hr, 90, requireHw());
    (void)r;
    auto res = core.commitBuses();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error(), v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(p->controllerId(), 0);
    EXPECT_EQ(q->controllerId(), 1);
}

TEST(AllocationCoreSeam, RequireLowPowerClaimsOptInController)
{
    v2::bus::BusRegistry reg;
    FakeOptInKind kind{/*capacity=*/3};
    v2::bus::AllocationCore core{reg, kind};

    std::shared_ptr<v2::bus::IBus> hs;
    auto* s = internFake(reg, hs, 70, v2::bus::requireLowPower());
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(s->backendKind(), kHw);
    EXPECT_EQ(s->controllerId(), 2);  // the only controller offering LOW_POWER
}

TEST(AllocationCoreSeam, RequireLowPowerFailsWhenOptInControllerBusy)
{
    v2::bus::BusRegistry reg;
    FakeOptInKind kind{/*capacity=*/3};
    v2::bus::AllocationCore core{reg, kind};

    std::shared_ptr<v2::bus::IBus> hs, ht;
    auto* s = internFake(reg, hs, 70, v2::bus::requireLowPower());
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(s->controllerId(), 2);

    // A second low-power request cannot be satisfied: same failure shape as
    // an ordinary over-subscribed requireHardware().
    auto* t = internFake(reg, ht, 80, v2::bus::requireLowPower());
    (void)t;
    auto res = core.commitBuses();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error(), v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(s->controllerId(), 2);  // incumbent kept its controller
}

TEST(AllocationCoreSeam, PreferLowPowerTakesOptInWhenFreeAndFallsBackWhenBusy)
{
    {
        v2::bus::BusRegistry reg;
        FakeOptInKind kind{/*capacity=*/3};
        v2::bus::AllocationCore core{reg, kind};

        std::shared_ptr<v2::bus::IBus> hu;
        auto* u = internFake(reg, hu, 70, v2::bus::preferLowPower());
        ASSERT_TRUE(core.commitBuses().has_value());
        EXPECT_EQ(u->controllerId(), 2);  // free opt-in controller wins when preferred
    }
    {
        v2::bus::BusRegistry reg;
        FakeOptInKind kind{/*capacity=*/3};
        v2::bus::AllocationCore core{reg, kind};

        std::shared_ptr<v2::bus::IBus> hs, hv;
        auto* s = internFake(reg, hs, 70, v2::bus::requireLowPower());
        ASSERT_TRUE(core.commitBuses().has_value());
        EXPECT_EQ(s->controllerId(), 2);

        auto* v = internFake(reg, hv, 80, v2::bus::preferLowPower());
        ASSERT_TRUE(core.commitBuses().has_value());
        EXPECT_EQ(v->backendKind(), kHw);
        EXPECT_NE(v->controllerId(), 2);  // opt-in controller busy: falls back to plain hardware
    }
}

TEST(AllocationCoreSeam, RequireSpecificOptInControllerWithoutCapsDeclared)
{
    v2::bus::BusRegistry reg;
    FakeOptInKind kind{/*capacity=*/3};
    v2::bus::AllocationCore core{reg, kind};

    // No LOW_POWER declared in require/prefer, but ControllerMode::Require
    // names controller 2 directly: that is opt-in enough on its own.
    std::shared_ptr<v2::bus::IBus> hw_ctrl2;
    auto* w = internFake(reg, hw_ctrl2, 70, requireCtrl(2));
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(w->backendKind(), kHw);
    EXPECT_EQ(w->controllerId(), 2);
}

// --- Uniform-kind caps check no longer bypassed by the short-circuit -------
// Before this change, `_eligible()` returned true for EVERY controller of a
// uniform kind before even looking at caps -- an unmet require bit (e.g.
// LOW_POWER on a plain-hardware-only build) silently succeeded onto a
// controller that cannot actually offer it. The caps check now runs first
// for uniform kinds too, so this fails exactly like an over-subscribed
// requireHardware().
TEST(I2cBusIntent, UniformKindRequireLowPowerFailsInsteadOfSilentSuccess)
{
    I2cIntentHarness h{&fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    auto bus = h.view.acquire(reqByIndex(0, v2::i2c::requireLowPower()));
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());

    auto res = h.view.commitBuses();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error(), v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(bus.value()->backendKind(), kSw);
}

// --- Pin-domain eligibility ---------------------------------------------
// Models a low-power-domain controller (e.g. LP_I2C) whose wiring is
// restricted to one fixed candidate: non-uniform caps (only controller 2
// offers the opt-in LOW_POWER bit, same as FakeOptInKind above) PLUS a
// pin-domain restriction enforced through `controllerAcceptsBus()`. This
// generic seam has no per-kind pin model, so `FakeManagedBus::wiring()` (an
// arbitrary int) stands in for "the candidate pin pair"; the real per-pin
// check is exercised separately by the i2c LP integration tests below.
namespace {

constexpr int kFixedWiring = 42;

class FakePinDomainKind : public FakeOptInKind {
public:
    using FakeOptInKind::FakeOptInKind;

    bool controllerAcceptsBus(const v2::bus::IManagedBus& bus, int8_t controller) const override
    {
        if (controller != 2) {
            return true;  // plain hardware controllers accept any wiring
        }
        return static_cast<const FakeManagedBus&>(bus).wiring() == kFixedWiring;
    }
};

}  // namespace

TEST(AllocationCoreSeam, PinDomainRequireControllerRejectsWrongWiring)
{
    v2::bus::BusRegistry reg;
    FakePinDomainKind kind{/*capacity=*/3};
    v2::bus::AllocationCore core{reg, kind};

    // requireCtrl(2) is tier 0 (named controller, no fallback): caps pass
    // (controller 2 offers LOW_POWER and is named, so the opt-in bypass
    // applies), but the wiring does not match the fixed pin domain -- D8
    // says this is a configuration error (INVALID_ARGUMENT), not a resource
    // shortage (OUT_OF_RESOURCE).
    std::shared_ptr<v2::bus::IBus> hold;
    (void)internFake(reg, hold, 70, requireCtrl(2), /*wiring=*/999);
    auto res = core.commitBuses();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error(), v2::error::error_t::INVALID_ARGUMENT);
}

TEST(AllocationCoreSeam, PinDomainSkipsWrongWiringForAutomaticAndPrefer)
{
    v2::bus::BusRegistry reg;
    FakePinDomainKind kind{/*capacity=*/3};
    v2::bus::AllocationCore core{reg, kind};

    // preferLowPower(): controller 2 is capability-eligible (opt-in bit
    // named via require/prefer) but pin-domain-ineligible (wrong wiring), so
    // it must be skipped in favour of a plain hardware controller -- not
    // left on the software placeholder (that would conflate a pin-domain
    // reject with a capability-only opt-in reject).
    std::shared_ptr<v2::bus::IBus> hold;
    auto* bus = internFake(reg, hold, 70, v2::bus::preferLowPower(), /*wiring=*/999);
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(bus->backendKind(), kHw);
    EXPECT_NE(bus->controllerId(), 2);
}

TEST(AllocationCoreSeam, IncumbencyYieldsWrongWiringController)
{
    v2::bus::BusRegistry reg;
    FakePinDomainKind kind{/*capacity=*/3};
    v2::bus::AllocationCore core{reg, kind};

    // Force the bus onto controller 2 directly (bypassing the resolver) to
    // simulate an already-incumbent hardware assignment, with wiring that
    // does NOT match controller 2's fixed pin domain. preferController(2)
    // names the controller (so the opt-in cap bypass applies) without being
    // tier 0, so this exercises the INCUMBENCY branch specifically.
    std::shared_ptr<v2::bus::IBus> hold;
    auto* bus = internFake(reg, hold, 70, v2::bus::preferController(2), /*wiring=*/999);
    ASSERT_TRUE(
        bus->swapBackend(std::unique_ptr<v2::bus::IBus>(new (std::nothrow) FakeCoreHwBackend(2)), 0).has_value());
    ASSERT_EQ(bus->controllerId(), 2);

    // The pin-domain check applies to the incumbent controller too: wrong
    // wiring means it cannot keep controller 2, and it is reassigned to a
    // plain controller instead of clinging to its current one.
    ASSERT_TRUE(core.commitBuses().has_value());
    EXPECT_EQ(bus->backendKind(), kHw);
    EXPECT_NE(bus->controllerId(), 2);
}

// --- LocalKindAdapter<Traits>::Topology --------------------------------------

TEST(I2cBusIntent, DefaultTopologyIsUniformNoOptIn)
{
    // The default (omitted) Topology argument must reproduce the original
    // uniform / no-opt-in behaviour exactly -- every existing
    // I2cIntentHarness-based test above already exercises this implicitly;
    // this asserts it directly against the IAllocationKind surface.
    v2::bus::BusRegistry reg;
    using Adapter = v2::bus::LocalKindAdapter<v2::i2c::BusTraits>;
    Adapter adapter{reg, &fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2};
    const v2::bus::IAllocationKind& k = adapter;
    EXPECT_TRUE(k.uniformControllers());
    EXPECT_EQ(k.optInCaps(), 0u);
    EXPECT_EQ(k.controllerCaps(0), v2::i2c::caps::HARDWARE);
    EXPECT_EQ(k.controllerCaps(1), v2::i2c::caps::HARDWARE);
}

// --- i2c LP integration: LocalKindAdapter + LocalBackend + BusView ----------
// Exercises the real i2c logical-acquire path (BusView::acquire(LogicalBusConfig)
// -> IHalBackend::completeLogicalRequest -> ILocalKindAdapter::completeLogical)
// with a Topology modeling a fixed-pin low-power controller (the C5/C6 shape:
// controller 1 accepts only one SDA/SCL pair, auto-filled when omitted).
namespace {

constexpr v2::types::gpio_number_t kLpFixedScl = 9;
constexpr v2::types::gpio_number_t kLpFixedSda = 8;

v2::result_t<void> fakeCompleteLogicalFixedPin(v2::i2c::LogicalBusConfig& cfg)
{
    if ((cfg.intent.require & v2::i2c::caps::LOW_POWER) == 0) {
        // Only requireLowPower() triggers auto-fill (D4): preferLowPower()
        // may still fall back to HP at commit time, after this identity is
        // already fixed.
        return {};
    }
    if (cfg.pin_scl < 0 && cfg.pin_sda < 0) {
        cfg.pin_scl = kLpFixedScl;
        cfg.pin_sda = kLpFixedSda;
        return {};
    }
    if (cfg.pin_scl != kLpFixedScl || cfg.pin_sda != kLpFixedSda) {
        return m5::stl::make_unexpected(v2::error::error_t::INVALID_ARGUMENT);
    }
    return {};
}

v2::types::backend_caps_t fakeLpControllerCaps(int8_t controller)
{
    return controller == 1 ? (v2::i2c::caps::HARDWARE | v2::i2c::caps::LOW_POWER) : v2::i2c::caps::HARDWARE;
}

bool fakeLpPinsAllowed(const v2::i2c::LogicalBusConfig& cfg, int8_t controller)
{
    if (controller != 1) {
        return true;
    }
    return cfg.pin_scl == kLpFixedScl && cfg.pin_sda == kLpFixedSda;
}

v2::bus::LocalKindAdapter<v2::i2c::BusTraits>::Topology fakeLpTopology(void)
{
    v2::bus::LocalKindAdapter<v2::i2c::BusTraits>::Topology topo;
    topo.controller_caps  = &fakeLpControllerCaps;
    topo.opt_in           = v2::i2c::caps::LOW_POWER;
    topo.complete_logical = &fakeCompleteLogicalFixedPin;
    topo.pins_allowed     = &fakeLpPinsAllowed;
    return topo;
}

struct I2cLpHarness {
    using Adapter = v2::bus::LocalKindAdapter<v2::i2c::BusTraits>;

    v2::bus::LocalBackend backend;
    Adapter adapter;
    v2::i2c::BusView view;

    I2cLpHarness()
        : adapter{backend.busRegistry(), &fakeSwFactory, &fakeHwFactory, /*hw_capacity=*/2, fakeLpTopology()},
          view{&backend}
    {
        backend.registerKind(adapter);
    }
};

}  // namespace

TEST(I2cBusIntent, LowPowerLogicalAcquireAutoFillsFixedPins)
{
    I2cLpHarness h;
    v2::i2c::LogicalBusConfig req{v2::i2c::Scl{-1}, v2::i2c::Sda{-1}, v2::i2c::requireLowPower()};
    auto bus = h.view.acquire(req);
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    const auto& cfg = static_cast<const v2::i2c::IBusConfig&>(bus.value()->getConfig());
    EXPECT_EQ(cfg.pin_scl, kLpFixedScl);
    EXPECT_EQ(cfg.pin_sda, kLpFixedSda);

    ASSERT_TRUE(h.view.commitBuses().has_value());
    EXPECT_EQ(bus.value()->backendKind(), kHw);
    EXPECT_EQ(bus.value()->controllerId(), 1);
}

TEST(I2cBusIntent, LowPowerLogicalAcquireRejectsMismatchedExplicitPins)
{
    I2cLpHarness h;
    v2::i2c::LogicalBusConfig req{v2::i2c::Scl{7}, v2::i2c::Sda{6}, v2::i2c::requireLowPower()};
    auto bus = h.view.acquire(req);
    ASSERT_FALSE(bus.has_value());
    EXPECT_EQ(bus.error(), v2::error::error_t::INVALID_ARGUMENT);
}

TEST(I2cBusIntent, NonLowPowerLogicalAcquireStillRequiresExplicitPins)
{
    // requireHardware() never triggers auto-fill: an omitted pin pair is
    // rejected exactly as it always was (id.valid() catches it).
    I2cLpHarness h;
    v2::i2c::LogicalBusConfig req{v2::i2c::Scl{-1}, v2::i2c::Sda{-1}, v2::i2c::requireHardware()};
    auto bus = h.view.acquire(req);
    ASSERT_FALSE(bus.has_value());
    EXPECT_EQ(bus.error(), v2::error::error_t::INVALID_ARGUMENT);
}

TEST(I2cBusIntent, AutoClaimAvoidsOptInLowPowerController)
{
    // Same opt-in exclusion rule claimController shares with the resolver's
    // _eligible: an Auto claim must never land on a controller that offers
    // an opt-in-only capability (LOW_POWER) unless the intent names it.
    I2cLpHarness h;
    auto claim = h.adapter.allocationCore()->claimController(v2::types::AllocationIntent{});
    ASSERT_TRUE(claim.has_value()) << "err=" << v2::error::toString(claim.error());
    EXPECT_EQ(claim.value(), 0);  // controller 1 is the opt-in LOW_POWER port
}

TEST(I2cBusIntent, ClaimControllerHonorsCapabilityPreference)
{
    // The resolver's preferred-capability pass, mirrored by claimController:
    // preferLowPower() lands on the LP controller while it is free, and
    // falls back to a plain HP controller once it is taken (a preference,
    // unlike a require, may go unmet).
    I2cLpHarness h;
    auto* core = h.adapter.allocationCore();
    auto lp    = core->claimController(v2::bus::preferLowPower());
    ASSERT_TRUE(lp.has_value()) << "err=" << v2::error::toString(lp.error());
    EXPECT_EQ(lp.value(), 1);  // controller 1 is the opt-in LOW_POWER port

    auto fallback = core->claimController(v2::bus::preferLowPower());
    ASSERT_TRUE(fallback.has_value()) << "err=" << v2::error::toString(fallback.error());
    EXPECT_EQ(fallback.value(), 0);
}

// --- Backend hot-swap order + rollback (release-before-make) ----------------
// Exercises the REAL LocalKindAdapter<Traits>::commitPlaceholder /
// commitHardware (managed_facade.hpp's swapBackendWith rollback overload),
// unlike the AllocationCoreSeam fakes above (FakeSwlessKind / FakeOptInKind
// implement their own commitPlaceholder/commitHardware and never call
// swapBackendWith). LocalKindAdapter's SwFactory/HwFactory are plain
// function pointers (no captures), so the order log and one-shot failure
// injection a test wants the factories to see live behind a namespace-scope
// pointer; ScopedFakeState installs it on construction and clears it on
// destruction (including on an early ASSERT_* return), so a failing
// assertion in one test can never leave a dangling pointer for the next.
namespace {

// A fake backend that records "make"/"release" events (tagged by kind and
// controller) into a shared log and performs no real I/O.
class OrderedFakeBackend : public v2::i2c::IBus {
public:
    OrderedFakeBackend(std::vector<std::string>* log, v2::types::backend_kind_t kind, int8_t controller)
        : _log{log}, _kind{kind}, _controller{controller}
    {
        if (_log != nullptr) {
            _log->push_back(_tag("make"));
        }
    }
    v2::types::backend_kind_t backendKind(void) const override
    {
        return _kind;
    }
    int8_t controllerId(void) const override
    {
        return _kind == v2::types::backend_kind_t::Hardware ? _controller : static_cast<int8_t>(-1);
    }
    v2::result_t<void> release(void) override
    {
        if (_log != nullptr) {
            _log->push_back(_tag("release"));
        }
        return {};
    }

private:
    std::string _tag(const char* verb) const
    {
        std::string s = verb;
        s += _kind == v2::types::backend_kind_t::Hardware ? (":hw" + std::to_string(_controller)) : ":sw";
        return s;
    }
    std::vector<std::string>* _log;
    v2::types::backend_kind_t _kind;
    int8_t _controller;
};

struct FakeState {
    std::vector<std::string> log;
    bool fail_next_sw = false;  // next placeholder build returns null (consumed on use)
    bool fail_next_hw = false;  // next hardware build returns null (consumed on use)
};
FakeState* g_state = nullptr;

struct ScopedFakeState {
    FakeState state;
    ScopedFakeState(void)
    {
        g_state = &state;
    }
    ~ScopedFakeState(void)
    {
        g_state = nullptr;
    }
    ScopedFakeState(const ScopedFakeState&)            = delete;
    ScopedFakeState& operator=(const ScopedFakeState&) = delete;
};

v2::i2c::IBus* orderedSwFactory(const v2::i2c::LogicalBusConfig&)
{
    if (g_state->fail_next_sw) {
        g_state->fail_next_sw = false;
        return nullptr;
    }
    return new (std::nothrow) OrderedFakeBackend(&g_state->log, v2::types::backend_kind_t::Software, -1);
}
v2::i2c::IBus* orderedHwFactory(const v2::i2c::LogicalBusConfig&, int8_t controller)
{
    if (g_state->fail_next_hw) {
        g_state->fail_next_hw = false;
        return nullptr;
    }
    return new (std::nothrow) OrderedFakeBackend(&g_state->log, v2::types::backend_kind_t::Hardware, controller);
}

}  // namespace

TEST(I2cBusIntentSwapRollback, DemoteReleasesOldHardwareBeforeBuildingPlaceholder)
{
    // guard is declared before h so its dtor (clearing g_state) runs AFTER
    // h's dtor -- g_state stays valid for every backend h creates/destroys.
    ScopedFakeState guard;
    // hw_capacity=1 forces a full demote (nowhere to hop) rather than a
    // demote immediately followed by a same-pass re-promote elsewhere.
    I2cIntentHarness h{&orderedSwFactory, &orderedHwFactory, /*hw_capacity=*/1};

    auto a = h.view.acquire(reqByIndex(0, v2::bus::automatic()));
    ASSERT_TRUE(a.has_value()) << "err=" << v2::error::toString(a.error());
    ASSERT_TRUE(h.view.commitBuses().has_value());
    EXPECT_EQ(a.value()->backendKind(), kHw);
    EXPECT_EQ(a.value()->controllerId(), 0);

    // B requires controller 0: with hw_capacity=1, A has nowhere to hop and
    // must fully demote to its software placeholder.
    auto b = h.view.acquire(reqByIndex(1, v2::i2c::requireController(0)));
    ASSERT_TRUE(b.has_value()) << "err=" << v2::error::toString(b.error());
    // Clear AFTER acquiring B: acquire itself builds B's initial software
    // backend ("make:sw"), which is not part of the swap under test.
    guard.state.log.clear();
    ASSERT_TRUE(h.view.commitBuses().has_value());
    EXPECT_EQ(b.value()->backendKind(), kHw);
    EXPECT_EQ(b.value()->controllerId(), 0);
    EXPECT_EQ(a.value()->backendKind(), kSw);

    ASSERT_GE(guard.state.log.size(), 2u);
    EXPECT_EQ(guard.state.log[0], "release:hw0");  // A's old hardware released FIRST
    EXPECT_EQ(guard.state.log[1], "make:sw");      // ... then the placeholder is built
}

TEST(I2cBusIntentSwapRollback, DemoteFailureRollsBackToSameHardwareController)
{
    ScopedFakeState guard;
    I2cIntentHarness h{&orderedSwFactory, &orderedHwFactory, /*hw_capacity=*/1};

    auto a = h.view.acquire(reqByIndex(0, v2::bus::automatic()));
    ASSERT_TRUE(a.has_value()) << "err=" << v2::error::toString(a.error());
    ASSERT_TRUE(h.view.commitBuses().has_value());
    ASSERT_EQ(a.value()->controllerId(), 0);

    auto b = h.view.acquire(reqByIndex(1, v2::i2c::requireController(0)));
    ASSERT_TRUE(b.has_value()) << "err=" << v2::error::toString(b.error());

    // A's demote needs a placeholder build; make it fail once. swapBackendWith
    // rolls back to a freshly-made hardware backend on the SAME controller
    // instead of leaving A without a backend.
    guard.state.fail_next_sw = true;
    auto res                 = h.view.commitBuses();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error(), v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(a.value()->backendKind(), kHw);
    EXPECT_EQ(a.value()->controllerId(), 0);  // rolled back onto the same controller

    // B never got promoted: the resolver's demote step (still holding
    // controller 0 in its pool bookkeeping because the release-on-failure
    // path is skipped for a failed demote) leaves no free controller for
    // B's promote this pass -- not a double lease, just no controller freed.
    EXPECT_EQ(b.value()->backendKind(), kSw);
    EXPECT_EQ(h.adapter.allocationCore()->hardwareInUse(), 1u);  // only A's controller 0
}

TEST(I2cBusIntentSwapRollback, PromoteFailureKeepsPlaceholderAndControllerIsNotLeaked)
{
    ScopedFakeState guard;
    I2cIntentHarness h{&orderedSwFactory, &orderedHwFactory, /*hw_capacity=*/1};

    auto x = h.view.acquire(reqByIndex(0, v2::bus::automatic()));
    ASSERT_TRUE(x.has_value()) << "err=" << v2::error::toString(x.error());
    EXPECT_EQ(x.value()->backendKind(), kSw);  // software until commit

    // The hardware build the promote needs fails once. swapBackendWith
    // rolls back to a freshly-made placeholder instead of leaving X without
    // a backend, and the resolver returns the claimed controller to the pool.
    guard.state.fail_next_hw = true;
    auto res                 = h.view.commitBuses();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error(), v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(x.value()->backendKind(), kSw);
    EXPECT_EQ(x.value()->controllerId(), -1);
    EXPECT_EQ(h.adapter.allocationCore()->hardwareInUse(), 0u);  // controller 0 not leaked

    // The controller is obtainable on a later commit once the build works.
    ASSERT_TRUE(h.view.commitBuses().has_value());
    EXPECT_EQ(x.value()->backendKind(), kHw);
    EXPECT_EQ(x.value()->controllerId(), 0);
}

TEST(I2cBusIntentSwapRollback, PromoteRollbackAlsoFailingLeavesPendingWithLoudError)
{
    ScopedFakeState guard;
    I2cIntentHarness h{&orderedSwFactory, &orderedHwFactory, /*hw_capacity=*/1};

    auto x = h.view.acquire(reqByIndex(0, v2::bus::automatic()));
    ASSERT_TRUE(x.has_value()) << "err=" << v2::error::toString(x.error());

    // Both the hardware build AND the placeholder rollback fail on the same
    // swap: swapBackendWith's fallback adopts null ("pending") rather than
    // leaving a stale pointer, and the failure is still reported.
    guard.state.fail_next_hw = true;
    guard.state.fail_next_sw = true;
    auto res                 = h.view.commitBuses();
    ASSERT_FALSE(res.has_value());
    EXPECT_EQ(res.error(), v2::error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(x.value()->backendKind(), kSw);  // safe default query answer for "no backend"
    EXPECT_EQ(x.value()->controllerId(), -1);
    EXPECT_EQ(h.adapter.allocationCore()->hardwareInUse(), 0u);

    // The next commit (both factories healthy again) recovers normally.
    ASSERT_TRUE(h.view.commitBuses().has_value());
    EXPECT_EQ(x.value()->backendKind(), kHw);
    EXPECT_EQ(x.value()->controllerId(), 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
