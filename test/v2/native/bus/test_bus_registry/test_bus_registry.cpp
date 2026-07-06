// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>

// Reuse the virtual open-drain bus + GPIO injection from the software-I2C
// suite for the M5_Hal.I2C integration test (real software backend on
// injected pins).
#include "../test_software_i2c/i2c_virtual_bus.hpp"

#include <memory>
#include <vector>

// bus::BusRegistry (ADR 034 phase 2) — interns buses by (kind, identity),
// owns them via shared_ptr/weak_ptr, capacity-bounded. The direct tests pin
// the registry logic with a no-I/O fake; the M5_Hal.I2C tests cover the
// real BusView -> facade -> software backend path.

namespace {
namespace v2 = m5::hal::v2;

// Minimal i2c bus for the direct registry tests (no I/O; i2c::IBus is
// instantiable, its virtuals default to NOT_IMPLEMENTED).
class FakeI2cBus : public v2::i2c::IBus {};

// A fake backend that reports a distinct hardware identity and counts its
// release(), to verify swapBackend tears down the old backend and the
// facade's query API tracks the new one.
class FakeHwBackend : public v2::i2c::IBus {
public:
    FakeHwBackend(int8_t controller, int* release_counter) : _controller{controller}, _releases{release_counter}
    {
    }
    v2::types::backend_kind_t backendKind(void) const override
    {
        return v2::types::backend_kind_t::Hardware;
    }
    int8_t controllerId(void) const override
    {
        return _controller;
    }
    uint32_t maxFrequency(void) const override
    {
        return 400000;
    }
    v2::result_t<void> release(void) override
    {
        if (_releases != nullptr) {
            ++(*_releases);
        }
        return {};
    }

private:
    int8_t _controller;
    int* _releases;
};

class FailingReleaseHwBackend : public v2::i2c::IBus {
public:
    FailingReleaseHwBackend(int8_t controller, int* release_counter)
        : _controller{controller}, _releases{release_counter}
    {
    }
    v2::types::backend_kind_t backendKind(void) const override
    {
        return v2::types::backend_kind_t::Hardware;
    }
    int8_t controllerId(void) const override
    {
        return _controller;
    }
    uint32_t maxFrequency(void) const override
    {
        return 400000;
    }
    v2::result_t<void> release(void) override
    {
        if (_releases != nullptr) {
            ++(*_releases);
        }
        return m5::stl::make_unexpected(v2::error::error_t::IO_ERROR);
    }

private:
    int8_t _controller;
    int* _releases;
};

// make-fn: always succeeds with a fresh FakeI2cBus, counting invocations so
// a hit (no make call) is distinguishable from a miss.
struct CountingMaker {
    int* calls;
    v2::result_t<std::shared_ptr<v2::bus::IBus>> operator()() const
    {
        ++(*calls);
        return std::shared_ptr<v2::bus::IBus>{std::make_shared<FakeI2cBus>()};
    }
};

v2::bus::IdentityKey keyOf(v2::types::gpio_number_t scl, v2::types::gpio_number_t sda)
{
    v2::bus::IdentityKey k;
    k.pins[0] = scl;
    k.pins[1] = sda;
    return k;
}

}  // namespace

// ---- bus::BusRegistry direct unit tests (intern / cap / reuse) -----------

TEST(BusRegistry, SamePinsReturnSameInstanceAndMakeOnce)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    auto a    = reg.acquireOrFind(v2::types::bus_kind_t::I2C, keyOf(22, 21), CountingMaker{&calls});
    auto b    = reg.acquireOrFind(v2::types::bus_kind_t::I2C, keyOf(22, 21), CountingMaker{&calls});
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a.value().get(), b.value().get());  // interned -> same instance
    EXPECT_EQ(calls, 1);                          // make ran only on the miss
    EXPECT_EQ(reg.liveCount(), 1u);
}

TEST(BusRegistry, DistinctPinsReturnDistinctInstances)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    auto a    = reg.acquireOrFind(v2::types::bus_kind_t::I2C, keyOf(22, 21), CountingMaker{&calls});
    auto b    = reg.acquireOrFind(v2::types::bus_kind_t::I2C, keyOf(33, 32), CountingMaker{&calls});
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_NE(a.value().get(), b.value().get());
    EXPECT_EQ(calls, 2);
    EXPECT_EQ(reg.liveCount(), 2u);
}

TEST(BusRegistry, FullRegistryReturnsOutOfResource)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    std::vector<std::shared_ptr<v2::bus::IBus>> held;
    for (size_t i = 0; i < v2::bus::BusRegistry::kCapacity; ++i) {
        auto r = reg.acquireOrFind(v2::types::bus_kind_t::I2C, keyOf(static_cast<v2::types::gpio_number_t>(i), 100),
                                   CountingMaker{&calls});
        ASSERT_TRUE(r.has_value());
        held.push_back(r.value());
    }
    EXPECT_EQ(reg.liveCount(), v2::bus::BusRegistry::kCapacity);
    auto overflow = reg.acquireOrFind(v2::types::bus_kind_t::I2C, keyOf(120, 121), CountingMaker{&calls});
    ASSERT_FALSE(overflow.has_value());
    EXPECT_EQ(overflow.error(), v2::error::error_t::OUT_OF_RESOURCE);
}

TEST(BusRegistry, DroppedBusFreesSlotForReuse)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    std::vector<std::shared_ptr<v2::bus::IBus>> held;
    for (size_t i = 0; i < v2::bus::BusRegistry::kCapacity; ++i) {
        auto r = reg.acquireOrFind(v2::types::bus_kind_t::I2C, keyOf(static_cast<v2::types::gpio_number_t>(i), 100),
                                   CountingMaker{&calls});
        ASSERT_TRUE(r.has_value());
        held.push_back(r.value());
    }
    // Full: a new key fails.
    ASSERT_FALSE(reg.acquireOrFind(v2::types::bus_kind_t::I2C, keyOf(120, 121), CountingMaker{&calls}).has_value());
    // Drop one holder -> its weak entry expires -> slot reclaimed.
    held.pop_back();
    EXPECT_EQ(reg.liveCount(), v2::bus::BusRegistry::kCapacity - 1);
    auto reused = reg.acquireOrFind(v2::types::bus_kind_t::I2C, keyOf(120, 121), CountingMaker{&calls});
    EXPECT_TRUE(reused.has_value());
    EXPECT_EQ(reg.liveCount(), v2::bus::BusRegistry::kCapacity);
}

// ---- backend query API defaults (ADR 034 phase 3) ------------------------

TEST(BusBackendQuery, DefaultsAreSafeForUnmigratedBus)
{
    // A bus that has not opted into the phase-3 backend model answers the
    // safe defaults: software / no controller / unknown ceiling / never
    // swapped. This keeps the query API harmless for every kind (and test
    // fake) that inherits it before implementing phase 3.
    FakeI2cBus bus;
    v2::bus::IBus& base = bus;
    EXPECT_EQ(base.backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(base.controllerId(), -1);
    EXPECT_EQ(base.maxFrequency(), 0u);
    EXPECT_EQ(base.backendGeneration(), 0u);
}

// ---- M5_Hal.I2C BusView integration (real software backend) --------------

TEST(I2cBusView, AcquireInternsAndDrivesBackend)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    auto& hal = v2::getM5_Hal();

    // Explicit software config -> BusView picks the bit-bang backend.
    v2::i2c::BusConfig_software cfg;
    cfg.pin_scl = gpio.scl();
    cfg.pin_sda = gpio.sda();

    // First acquire creates the facade (software backend) for these pins.
    auto a = hal.I2C.acquire(cfg);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(a.value());                                 // non-null shared_ptr
    EXPECT_EQ(a.value()->getConfig().pin_scl, gpio.scl());  // facade exposes the wiring

    // Same pins -> SAME interned instance (one physical wiring, one bus).
    auto b = hal.I2C.acquire(cfg);
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a.value().get(), b.value().get());

    // The registry-created facade actually drives the wire: probe the slave.
    EXPECT_TRUE(a.value()->probe(0x42).has_value());   // slave ACKs through the acquired bus
    EXPECT_FALSE(a.value()->probe(0x21).has_value());  // no device -> NACK

    // The facade forwards the query API to its (software) backend, so an
    // explicit software acquire reports software with no controller.
    EXPECT_EQ(a.value()->backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(a.value()->controllerId(), -1);
}

// ---- M5_Hal.I2C logical acquire (pins + intent, ADR 034 phase 3) ---------

TEST(I2cBusViewLogical, LogicalAcquireCreatesSoftwareBusAndInterns)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    auto& hal = v2::getM5_Hal();

    // Logical acquire states pins + intent, not a backend type. Before any
    // commit it lands on the software backend, so the bus works immediately.
    v2::i2c::LogicalBusConfig req{v2::i2c::Scl{gpio.scl()}, v2::i2c::Sda{gpio.sda()}};
    req.intent = v2::i2c::automatic();

    auto a = hal.I2C.acquire(req);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(a.value());
    EXPECT_EQ(a.value()->getConfig().pin_scl, gpio.scl());
    EXPECT_EQ(a.value()->backendKind(), v2::types::backend_kind_t::Software);  // software before commit
    EXPECT_TRUE(a.value()->probe(0x42).has_value());                           // drives the wire on software

    // Identity is the pins alone: a typed software acquire of the SAME wiring
    // shares the interned instance (first backend wins).
    v2::i2c::BusConfig_software typed;
    typed.pin_scl = gpio.scl();
    typed.pin_sda = gpio.sda();
    auto b        = hal.I2C.acquire(typed);
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a.value().get(), b.value().get());  // same wiring -> same bus
}

// ---- i2c::Bus facade hot-swap (ADR 034 phase 3) --------------------------

TEST(I2cFacadeSwap, SwapBackendTracksQueryAndBumpsGeneration)
{
    v2::i2c::Bus facade;
    // Start on a real software backend via the phase-1 typed init. The pins
    // are arbitrary -- no wire I/O is performed in this test.
    v2::i2c::BusConfig_software sw;
    sw.pin_scl = 22;
    sw.pin_sda = 21;
    ASSERT_TRUE(facade.init(sw).has_value());
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(facade.backendGeneration(), 0u);

    int releases_a = 0;
    int releases_b = 0;

    // Swap to a fake hardware backend: the query API tracks the new backend
    // and the generation counter advances.
    ASSERT_TRUE(facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeHwBackend(1, &releases_a)}).has_value());
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Hardware);
    EXPECT_EQ(facade.controllerId(), 1);
    EXPECT_EQ(facade.maxFrequency(), 400000u);
    EXPECT_EQ(facade.backendGeneration(), 1u);
    EXPECT_EQ(releases_a, 0);  // backend A is live, not released yet

    // A second swap tears down backend A and advances the generation again.
    ASSERT_TRUE(facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeHwBackend(0, &releases_b)}).has_value());
    EXPECT_EQ(releases_a, 1);  // old backend released on swap
    EXPECT_EQ(facade.controllerId(), 0);
    EXPECT_EQ(facade.backendGeneration(), 2u);

    // A null backend is rejected without disturbing the live one.
    auto bad = facade.swapBackend(nullptr);
    EXPECT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error(), v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(facade.controllerId(), 0);
    EXPECT_EQ(facade.backendGeneration(), 2u);
}

TEST(I2cFacadeRelease, FailureKeepsBackendForRetry)
{
    v2::i2c::Bus facade;
    int releases = 0;
    ASSERT_TRUE(
        facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FailingReleaseHwBackend(1, &releases)}).has_value());
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Hardware);
    EXPECT_EQ(facade.controllerId(), 1);
    EXPECT_EQ(facade.backendGeneration(), 1u);

    auto first = facade.release();
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), v2::error::error_t::IO_ERROR);
    EXPECT_EQ(releases, 1);
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Hardware);
    EXPECT_EQ(facade.controllerId(), 1);
    EXPECT_EQ(facade.backendGeneration(), 1u);

    auto retry = facade.release();
    ASSERT_FALSE(retry.has_value());
    EXPECT_EQ(retry.error(), v2::error::error_t::IO_ERROR);
    EXPECT_EQ(releases, 2);
}

TEST(I2cFacadeRelease, DefaultReleaseIsNoopSuccess)
{
    v2::i2c::Bus facade;
    ASSERT_TRUE(facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeI2cBus()}).has_value());

    EXPECT_TRUE(facade.release().has_value());
    EXPECT_TRUE(facade.release().has_value());
}

// ---- Accessor co-ownership of a borrowed bus (the canonical borrow path) -
//
// `M5_Hal.<kind>.acquire(cfg)` returns a shared_ptr; constructing an accessor
// from it co-owns the bus. The registry holds only a weak_ptr, so once the
// acquire temporary drops, the accessor's shared_ptr is the SOLE owner. If
// co-own works, the accessor still drives the wire afterwards.
TEST(I2cBusViewCoOwn, AccessorOutlivesAcquireTemporary)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    runner.add(slave.service());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    auto& hal = v2::getM5_Hal();
    v2::i2c::BusConfig_software cfg;
    cfg.pin_scl = gpio.scl();
    cfg.pin_sda = gpio.sda();

    v2::i2c::MasterAccessConfig present;
    present.i2c_addr = 0x42;  // the slave that ACKs
    v2::i2c::MasterAccessConfig absent;
    absent.i2c_addr = 0x21;  // no device -> NACK

    // Co-own straight from the acquire temporary: both the result_t and the
    // shared_ptr it holds are destroyed at the end of this full expression,
    // leaving the accessor's `_owner` as the only strong reference.
    v2::i2c::MasterAccessor dev{hal.I2C.acquire(cfg).value(), present};

    // The bus is alive solely because the accessor co-owns it, and still
    // drives the wire: 0x42 ACKs, a different address NACKs.
    EXPECT_TRUE(dev.probe().has_value());
    v2::i2c::MasterAccessor miss{hal.I2C.acquire(cfg).value(), absent};
    EXPECT_FALSE(miss.probe().has_value());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
