// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

// Reuse the virtual open-drain bus + GPIO injection from the software-I2C
// suite for the M5_Hal.I2C integration test (real software backend on
// injected pins).
#include "../test_software_i2c/i2c_virtual_bus.hpp"

#include <atomic>
#include <memory>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

namespace m5::hal::v2::bus {
struct BusRegistryTestAccess {
    static void seedGeneration(BusRegistry& registry, size_t slot, uint32_t generation)
    {
        BusRegistry::Guard guard{registry._mutex};
        registry._slots[slot].generation = generation;
    }
};
}  // namespace m5::hal::v2::bus

// bus::BusRegistry — interns buses by (kind, identity),
// owns them via shared_ptr/weak_ptr, capacity-bounded. The direct tests pin
// the registry logic with a no-I/O fake; the M5_Hal.I2C tests cover the
// real BusView -> facade -> software backend path.

namespace {
namespace v2 = m5::hal::v2;

// Minimal i2c bus for the direct registry tests (no I/O; i2c::IBus is
// instantiable, its virtuals default to NOT_IMPLEMENTED).
class FakeI2cBus : public v2::i2c::IBus {
public:
    FakeI2cBus() = default;
    FakeI2cBus(v2::types::gpio_number_t scl, v2::types::gpio_number_t sda)
    {
        _config.pin_scl = scl;
        _config.pin_sda = sda;
    }

    ~FakeI2cBus() override
    {
        if (_registry != nullptr && _registry->beginAbandon(_token, _key, this)) {
            _registry->finishAbandon(_token, _key, this, true);
        }
    }

    bool bindRegistryRegistration(v2::bus::BusRegistry& registry, const v2::bus::ResourceKey& key, uint16_t slot,
                                  uint32_t generation) override
    {
        _registry = &registry;
        _key      = key;
        _token    = {slot, 0, generation};
        return _token.valid();
    }
    const v2::bus::ResourceKey* registryResourceKey(void) const override
    {
        return &_key;
    }

    v2::result_t<void> close(void)
    {
        return v2::bus::IBus::close();
    }

private:
    v2::bus::BusRegistry* _registry = nullptr;
    v2::bus::ResourceKey _key;
    v2::bus::RegistryEntryToken _token;
};

class RegistryBoundCloseI2cBus : public FakeI2cBus {
public:
    RegistryBoundCloseI2cBus() : FakeI2cBus{22, 21}
    {
    }

    bool bindRegistryRegistration(v2::bus::BusRegistry& registry, const v2::bus::ResourceKey& key, uint16_t slot,
                                  uint32_t generation) override
    {
        ++bind_calls;
        return FakeI2cBus::bindRegistryRegistration(registry, key, slot, generation);
    }

protected:
    v2::bus::CloseOutcome closeBackend(void) override
    {
        ++close_calls;
        return v2::bus::CloseOutcome::success();
    }

public:
    int bind_calls  = 0;
    int close_calls = 0;
};

struct CloseBarrier {
    std::mutex mutex;
    std::condition_variable cv;
    bool entered = false;
    bool proceed = false;
};

class BlockingCloseI2cBus : public FakeI2cBus {
public:
    explicit BlockingCloseI2cBus(CloseBarrier& barrier) : FakeI2cBus{22, 21}, _barrier{barrier}
    {
    }

protected:
    v2::bus::CloseOutcome closeBackend(void) override
    {
        ++close_calls;
        std::unique_lock<std::mutex> lock{_barrier.mutex};
        _barrier.entered = true;
        _barrier.cv.notify_all();
        _barrier.cv.wait(lock, [&] { return _barrier.proceed; });
        return v2::bus::CloseOutcome::success();
    }

public:
    std::atomic<int> close_calls{0};

private:
    CloseBarrier& _barrier;
};

class ScriptedCloseI2cBus : public FakeI2cBus {
public:
    explicit ScriptedCloseI2cBus(std::vector<v2::bus::CloseOutcome> outcomes)
        : FakeI2cBus{22, 21}, _outcomes{std::move(outcomes)}
    {
    }

    int close_calls = 0;

protected:
    v2::bus::CloseOutcome closeBackend(void) override
    {
        const size_t index = static_cast<size_t>(close_calls++);
        return index < _outcomes.size() ? _outcomes[index] : v2::bus::CloseOutcome::success();
    }

private:
    std::vector<v2::bus::CloseOutcome> _outcomes;
};

class ReleaseFailingCloseI2cBus : public FakeI2cBus {
public:
    bool initializationIsAllowed(void) const
    {
        return initializationAllowed(false);
    }

    int close_calls = 0;

protected:
    v2::bus::CloseOutcome closeBackend(void) override
    {
        ++close_calls;
        return v2::bus::CloseOutcome::success();
    }

    v2::result_t<void> releaseCloseBarrier(void) override
    {
        auto released = FakeI2cBus::releaseCloseBarrier();
        if (!released.has_value()) {
            return released;
        }
        if (_fail_release_once) {
            _fail_release_once = false;
            return m5::stl::make_unexpected(v2::error::error_t::IO_ERROR);
        }
        return {};
    }

private:
    bool _fail_release_once = true;
};

class CountingCloseUartBus : public v2::uart::IBus {
public:
    v2::result_t<void> close(void)
    {
        return v2::bus::IBus::close();
    }

protected:
    v2::bus::CloseOutcome closeBackend(void) override
    {
        ++close_calls;
        return v2::bus::CloseOutcome::success();
    }

public:
    int close_calls = 0;
};

class CountingCloseI2sBus : public v2::i2s::IBus {
public:
    v2::result_t<void> close(void)
    {
        return v2::bus::IBus::close();
    }

protected:
    v2::bus::CloseOutcome closeBackend(void) override
    {
        ++close_calls;
        return v2::bus::CloseOutcome::success();
    }

public:
    int close_calls = 0;
};

struct DestructionBarrier {
    std::mutex mutex;
    std::condition_variable cv;
    bool destructor_entered = false;
    bool allow_begin        = false;
    bool closing_reserved   = false;
    bool allow_finish       = false;
    bool begin_succeeded    = false;
};

class RegisteredFakeI2cBus : public FakeI2cBus {
public:
    explicit RegisteredFakeI2cBus(DestructionBarrier& barrier) : _barrier{barrier}
    {
    }
    ~RegisteredFakeI2cBus() override
    {
        {
            std::unique_lock<std::mutex> lock{_barrier.mutex};
            _barrier.destructor_entered = true;
            _barrier.cv.notify_all();
            _barrier.cv.wait(lock, [&] { return _barrier.allow_begin; });
        }
        const bool begun = _registry != nullptr && _registry->beginAbandon(_token, _key, this);
        {
            std::unique_lock<std::mutex> lock{_barrier.mutex};
            _barrier.begin_succeeded  = begun;
            _barrier.closing_reserved = true;
            _barrier.cv.notify_all();
            _barrier.cv.wait(lock, [&] { return _barrier.allow_finish; });
        }
        if (begun) {
            _registry->finishAbandon(_token, _key, this, true);
        }
    }

    bool bindRegistryRegistration(v2::bus::BusRegistry& registry, const v2::bus::ResourceKey& key, uint16_t slot,
                                  uint32_t generation) override
    {
        _registry = &registry;
        _key      = key;
        _token    = {slot, 0, generation};
        return _token.valid();
    }
    const v2::bus::ResourceKey* registryResourceKey(void) const override
    {
        return &_key;
    }

private:
    DestructionBarrier& _barrier;
    v2::bus::BusRegistry* _registry = nullptr;
    v2::bus::ResourceKey _key;
    v2::bus::RegistryEntryToken _token;
};

// External-lifecycle shape: no registry destructor callback, but an Open
// tombstone remains visible from weak expiration until teardown commits.
class LifecycleOnlyFakeI2cBus : public FakeI2cBus {
public:
    explicit LifecycleOnlyFakeI2cBus(DestructionBarrier& barrier) : FakeI2cBus{22, 21}, _barrier{barrier}
    {
    }
    ~LifecycleOnlyFakeI2cBus() override
    {
        {
            std::unique_lock<std::mutex> lock{_barrier.mutex};
            _barrier.destructor_entered = true;
            _barrier.cv.notify_all();
            _barrier.cv.wait(lock, [&] { return _barrier.allow_finish; });
        }
        v2::bus::BusLifecycle::Close closing{*_lifecycle};
        if (closing) {
            (void)closing.commit();
        }
    }

    bool bindRegistryRegistration(v2::bus::BusRegistry&, const v2::bus::ResourceKey&, uint16_t, uint32_t) override
    {
        return false;
    }
    std::shared_ptr<v2::bus::BusLifecycle> lifecycleHandle(void) const override
    {
        return _lifecycle;
    }

private:
    DestructionBarrier& _barrier;
    std::shared_ptr<v2::bus::BusLifecycle> _lifecycle = std::make_shared<v2::bus::BusLifecycle>();
};

class UnsafeUnmanagedFakeI2cBus : public FakeI2cBus {
public:
    bool bindRegistryRegistration(v2::bus::BusRegistry&, const v2::bus::ResourceKey&, uint16_t, uint32_t) override
    {
        return false;
    }
};

class FakeHalBackend : public v2::bus::IHalBackend {
public:
    v2::bus::BusRegistry& busRegistry(void) override
    {
        return _registry;
    }
    const v2::bus::BusRegistry& busRegistry(void) const override
    {
        return _registry;
    }
    v2::result_t<std::shared_ptr<v2::bus::IBus>> acquireBusPortable(v2::types::bus_kind_t kind,
                                                                    const v2::bus::ResourceKey& id,
                                                                    const v2::bus::IBusConfig& cfg) override
    {
        (void)kind;
        const auto& i2c_cfg = static_cast<const v2::i2c::IBusConfig&>(cfg);
        return busRegistry().acquireOrFind(id, [&]() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
            ++make_calls;
            if (next_bus) {
                return std::move(next_bus);
            }
            return std::shared_ptr<v2::bus::IBus>{std::make_shared<FakeI2cBus>(i2c_cfg.pin_scl, i2c_cfg.pin_sda)};
        });
    }

    v2::result_t<std::shared_ptr<v2::bus::IBus>> acquireBusLogical(v2::types::bus_kind_t kind,
                                                                   const v2::bus::ResourceKey& id,
                                                                   const v2::bus::AllocationRequest& req) override
    {
        (void)kind;
        (void)id;
        (void)req;
        return m5::stl::make_unexpected(v2::error::error_t::NOT_IMPLEMENTED);
    }

    v2::result_t<void> commitBuses(v2::types::bus_kind_t kind, uint32_t timeout_ms) override
    {
        (void)kind;
        (void)timeout_ms;
        return {};
    }

    int make_calls = 0;
    std::shared_ptr<v2::bus::IBus> next_bus;

private:
    v2::bus::BusRegistry _registry;
};

// A fake backend that reports a distinct hardware identity and counts its
// closeBackend(), to verify swapBackend tears down the old backend and the
// facade's query API tracks the new one.
class FakeHwBackend : public v2::i2c::IBus {
public:
    FakeHwBackend(int8_t controller, int* close_counter) : _controller{controller}, _closes{close_counter}
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

protected:
    v2::bus::CloseOutcome closeBackend(void) override
    {
        if (_closes != nullptr) {
            ++(*_closes);
        }
        return v2::bus::CloseOutcome::success();
    }

private:
    int8_t _controller;
    int* _closes;
};

class CoherentCapabilityBackend : public v2::i2c::IBus {
public:
    explicit CoherentCapabilityBackend(bool low_power) : _low_power{low_power}
    {
    }
    v2::bus::BusCapabilities capabilities(void) const override
    {
        return v2::bus::detail::BusCapabilitiesBuilder{}
            .enable(v2::bus::BusFeature::LowPowerBackend, _low_power)
            .setLimit(v2::bus::BusLimit::MaxAtomicTxBytes, _low_power ? 111u : 222u)
            .build();
    }

protected:
    v2::bus::CloseOutcome closeBackend(void) override
    {
        return v2::bus::CloseOutcome::success();
    }

private:
    bool _low_power;
};

class FailingCloseHwBackend : public v2::i2c::IBus {
public:
    FailingCloseHwBackend(int8_t controller, int* close_counter) : _controller{controller}, _closes{close_counter}
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

protected:
    v2::bus::CloseOutcome closeBackend(void) override
    {
        if (_closes != nullptr) {
            ++(*_closes);
        }
        return v2::bus::CloseOutcome::partialOrUnknown(v2::error::error_t::IO_ERROR);
    }

private:
    int8_t _controller;
    int* _closes;
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

v2::bus::ResourceKey keyOf(v2::types::gpio_number_t first, v2::types::gpio_number_t second,
                           v2::types::bus_kind_t kind = v2::types::bus_kind_t::I2C)
{
    return v2::bus::ResourceKey::fromPins(kind, {first, second});
}

}  // namespace

// ---- bus::BusRegistry direct unit tests (intern / cap / reuse) -----------

TEST(BusRegistry, SamePinsReturnSameInstanceAndMakeOnce)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    auto a    = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    auto b    = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a.value().get(), b.value().get());  // interned -> same instance
    EXPECT_EQ(calls, 1);                          // make ran only on the miss
    EXPECT_EQ(reg.liveCount(), 1u);
}

TEST(BusRegistry, SameIdentityRejectsAnIncompatibleNativeBindingBeforeMake)
{
    v2::bus::BusRegistry registry;
    v2::bus::BindingDescriptor borrowed;
    borrowed.provider    = 9;
    borrowed.ownership   = v2::bus::Ownership::Borrowed;
    borrowed.native_kind = v2::bus::NativeBindingKind::Native;
    borrowed.native      = {1, 0, 1};

    int calls     = 0;
    auto validate = [](const std::shared_ptr<v2::bus::IBus>&) -> v2::result_t<void> { return {}; };
    auto first    = registry.acquireOrFind(keyOf(22, 21), borrowed, validate, CountingMaker{&calls});
    ASSERT_TRUE(first.has_value());

    auto same = registry.acquireOrFind(keyOf(22, 21), borrowed, validate, CountingMaker{&calls});
    ASSERT_TRUE(same.has_value());
    EXPECT_EQ(same.value(), first.value());

    auto managed      = borrowed;
    managed.ownership = v2::bus::Ownership::Managed;
    auto conflict     = registry.acquireOrFind(keyOf(22, 21), managed, validate, CountingMaker{&calls});
    ASSERT_FALSE(conflict.has_value());
    EXPECT_EQ(conflict.error(), v2::error::error_t::INVALID_STATE);
    EXPECT_EQ(calls, 1);
}

TEST(BusRegistry, DistinctPinsReturnDistinctInstances)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    auto a    = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    auto b    = reg.acquireOrFind(keyOf(33, 32), CountingMaker{&calls});
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_NE(a.value().get(), b.value().get());
    EXPECT_EQ(calls, 2);
    EXPECT_EQ(reg.liveCount(), 2u);
}

TEST(BusRegistry, ManagedFacadeNaturalTeardownFailureQuarantinesIdentity)
{
    v2::bus::BusRegistry registry;
    int closes = 0;
    auto made  = registry.acquireOrFind(keyOf(22, 21), [&]() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
        auto facade  = std::make_shared<v2::i2c::Bus>();
        auto swapped = facade->swapBackend(std::unique_ptr<v2::i2c::IBus>{new FailingCloseHwBackend(1, &closes)});
        if (!swapped.has_value()) {
            return m5::stl::make_unexpected(swapped.error());
        }
        return std::shared_ptr<v2::bus::IBus>{std::move(facade)};
    });
    ASSERT_TRUE(made.has_value());
    ASSERT_TRUE(made.value());

    made.value().reset();
    EXPECT_EQ(closes, 1);
    EXPECT_EQ(registry.liveCount(), 1u);

    int remake_calls = 0;
    auto reacquired  = registry.acquireOrFind(keyOf(22, 21), CountingMaker{&remake_calls});
    EXPECT_FALSE(reacquired.has_value());
    EXPECT_EQ(reacquired.error(), v2::error::error_t::BUSY);
    EXPECT_EQ(remake_calls, 0);
}

TEST(BusRegistry, UnsetPinsAreOrdinaryIdentityValuesAndFailedMakeIsNotInterned)
{
    v2::bus::BusRegistry reg;
    const auto unset = keyOf(-1, -1, v2::types::bus_kind_t::SPI);
    int failed_calls = 0;
    auto failed      = reg.acquireOrFind(unset, [&]() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
        ++failed_calls;
        return m5::stl::make_unexpected(v2::error::error_t::INVALID_ARGUMENT);
    });
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error(), v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(failed_calls, 1);
    EXPECT_EQ(reg.liveCount(), 0u);

    int success_calls = 0;
    auto a            = reg.acquireOrFind(unset, CountingMaker{&success_calls});
    auto b            = reg.acquireOrFind(unset, CountingMaker{&success_calls});
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a.value().get(), b.value().get());
    EXPECT_EQ(success_calls, 1);
    EXPECT_EQ(reg.liveCount(), 1u);
}

TEST(BusRegistry, FullRegistryReturnsOutOfResource)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    std::vector<std::shared_ptr<v2::bus::IBus>> held;
    for (size_t i = 0; i < v2::bus::BusRegistry::kCapacity; ++i) {
        auto r = reg.acquireOrFind(keyOf(static_cast<v2::types::gpio_number_t>(i), 100), CountingMaker{&calls});
        ASSERT_TRUE(r.has_value());
        held.push_back(r.value());
    }
    EXPECT_EQ(reg.liveCount(), v2::bus::BusRegistry::kCapacity);
    auto overflow = reg.acquireOrFind(keyOf(120, 121), CountingMaker{&calls});
    ASSERT_FALSE(overflow.has_value());
    EXPECT_EQ(overflow.error(), v2::error::error_t::OUT_OF_RESOURCE);
}

TEST(BusRegistry, DroppedBusFreesSlotForReuse)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    std::vector<std::shared_ptr<v2::bus::IBus>> held;
    for (size_t i = 0; i < v2::bus::BusRegistry::kCapacity; ++i) {
        auto r = reg.acquireOrFind(keyOf(static_cast<v2::types::gpio_number_t>(i), 100), CountingMaker{&calls});
        ASSERT_TRUE(r.has_value());
        held.push_back(r.value());
    }
    // Full: a new key fails.
    ASSERT_FALSE(reg.acquireOrFind(keyOf(120, 121), CountingMaker{&calls}).has_value());
    // Drop one holder -> its weak entry expires -> slot reclaimed.
    held.pop_back();
    EXPECT_EQ(reg.liveCount(), v2::bus::BusRegistry::kCapacity - 1);
    auto reused = reg.acquireOrFind(keyOf(120, 121), CountingMaker{&calls});
    EXPECT_TRUE(reused.has_value());
    EXPECT_EQ(reg.liveCount(), v2::bus::BusRegistry::kCapacity);
}

TEST(BusRegistry, ConstructingIdentityIsBusyAndFactoryRunsOutsideRegistryLock)
{
    v2::bus::BusRegistry reg;
    int calls = 0;

    auto outer = reg.acquireOrFind(keyOf(22, 21), [&]() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
        auto same = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
        EXPECT_FALSE(same.has_value());
        EXPECT_EQ(same.error(), v2::error::error_t::BUSY);

        auto different = reg.acquireOrFind(keyOf(18, 19), CountingMaker{&calls});
        EXPECT_TRUE(different.has_value());
        ++calls;
        return std::shared_ptr<v2::bus::IBus>{std::make_shared<FakeI2cBus>(22, 21)};
    });

    ASSERT_TRUE(outer.has_value());
    EXPECT_EQ(calls, 2);
    EXPECT_EQ(reg.liveCount(), 1u);  // the nested different-key temporary expired
}

TEST(BusRegistry, MalformedKeyNeverCallsFactoryOrConsumesCapacity)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    v2::bus::ResourceKey malformed;

    auto rejected = reg.acquireOrFind(malformed, CountingMaker{&calls});
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error(), v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(calls, 0);
    EXPECT_EQ(reg.liveCount(), 0u);

    auto valid = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    ASSERT_TRUE(valid.has_value());
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(reg.liveCount(), 1u);
}

#if defined(__cpp_exceptions)
TEST(BusRegistry, ThrowingFactoryRollsBackConstructingReservation)
{
    v2::bus::BusRegistry reg;
    EXPECT_THROW((void)reg.acquireOrFind(
                     keyOf(22, 21), []() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> { throw std::bad_alloc{}; }),
                 std::bad_alloc);
    EXPECT_EQ(reg.liveCount(), 0u);
    int calls  = 0;
    auto valid = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    EXPECT_TRUE(valid.has_value());
}
#endif

TEST(BusRegistryLifecycle, WeakExpiredGapAndClosingBothRejectReacquire)
{
    v2::bus::BusRegistry reg;
    DestructionBarrier barrier;
    int calls = 0;
    auto make = [&]() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
        ++calls;
        return std::shared_ptr<v2::bus::IBus>{std::make_shared<RegisteredFakeI2cBus>(barrier)};
    };
    auto acquired = reg.acquireOrFind(keyOf(22, 21), make);
    ASSERT_TRUE(acquired.has_value());
    std::shared_ptr<v2::bus::IBus> owner = std::move(acquired.value());

    std::thread destroyer{[owned = std::move(owner)]() mutable { owned.reset(); }};
    {
        std::unique_lock<std::mutex> lock{barrier.mutex};
        barrier.cv.wait(lock, [&] { return barrier.destructor_entered; });
    }

    auto weak_gap = reg.acquireOrFind(keyOf(22, 21), make);
    ASSERT_FALSE(weak_gap.has_value());
    EXPECT_EQ(weak_gap.error(), v2::error::error_t::BUSY);
    {
        std::lock_guard<std::mutex> lock{barrier.mutex};
        barrier.allow_begin = true;
        barrier.cv.notify_all();
    }
    {
        std::unique_lock<std::mutex> lock{barrier.mutex};
        barrier.cv.wait(lock, [&] { return barrier.closing_reserved; });
        EXPECT_TRUE(barrier.begin_succeeded);
    }

    auto closing = reg.acquireOrFind(keyOf(22, 21), make);
    ASSERT_FALSE(closing.has_value());
    EXPECT_EQ(closing.error(), v2::error::error_t::BUSY);
    {
        std::lock_guard<std::mutex> lock{barrier.mutex};
        barrier.allow_finish = true;
        barrier.cv.notify_all();
    }
    destroyer.join();

    auto reacquired = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    ASSERT_TRUE(reacquired.has_value());
    EXPECT_EQ(calls, 2);
}

TEST(BusRegistryLifecycle, ExternalTombstoneBlocksReacquireDuringDestructorWithoutCallback)
{
    v2::bus::BusRegistry reg;
    DestructionBarrier barrier;
    int calls = 0;
    auto make = [&]() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
        ++calls;
        return std::shared_ptr<v2::bus::IBus>{std::make_shared<LifecycleOnlyFakeI2cBus>(barrier)};
    };
    auto acquired = reg.acquireOrFind(keyOf(22, 21), make);
    ASSERT_TRUE(acquired.has_value());
    std::thread destroyer{[owned = std::move(acquired.value())]() mutable { owned.reset(); }};
    {
        std::unique_lock<std::mutex> lock{barrier.mutex};
        barrier.cv.wait(lock, [&] { return barrier.destructor_entered; });
    }

    auto during_destructor = reg.acquireOrFind(keyOf(22, 21), make);
    ASSERT_FALSE(during_destructor.has_value());
    EXPECT_EQ(during_destructor.error(), v2::error::error_t::BUSY);
    EXPECT_EQ(calls, 1);

    {
        std::lock_guard<std::mutex> lock{barrier.mutex};
        barrier.allow_finish = true;
        barrier.cv.notify_all();
    }
    destroyer.join();

    auto reacquired = reg.acquireOrFind(keyOf(22, 21), make);
    ASSERT_TRUE(reacquired.has_value());
    EXPECT_EQ(calls, 2);
}

TEST(BusRegistryLifecycle, RejectsBusWithoutDestructorCallbackOrLifecycleTombstone)
{
    v2::bus::BusRegistry reg;
    auto rejected = reg.acquireOrFind(keyOf(22, 21), []() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
        return std::shared_ptr<v2::bus::IBus>{std::make_shared<UnsafeUnmanagedFakeI2cBus>()};
    });
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error(), v2::error::error_t::INVALID_STATE);
    EXPECT_EQ(reg.liveCount(), 0u);
}

TEST(BusRegistryRelease, ReleasingIdentityRejectsAcquireUntilCancel)
{
    v2::bus::BusRegistry reg;
    int calls = 0;
    auto bus  = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());

    auto ticket = reg.beginRelease(keyOf(22, 21), bus.value());
    ASSERT_TRUE(ticket.has_value()) << "err=" << v2::error::toString(ticket.error());

    auto during_release = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    ASSERT_FALSE(during_release.has_value());
    EXPECT_EQ(during_release.error(), v2::error::error_t::BUSY);
    EXPECT_FALSE(reg.findByIdentity(keyOf(22, 21)));
    EXPECT_EQ(calls, 1);

    auto cancelled = reg.cancelRelease(ticket.value());
    ASSERT_TRUE(cancelled.has_value()) << "err=" << v2::error::toString(cancelled.error());
    auto after_cancel = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    ASSERT_TRUE(after_cancel.has_value()) << "err=" << v2::error::toString(after_cancel.error());
    EXPECT_EQ(after_cancel.value().get(), bus.value().get());
    EXPECT_EQ(calls, 1);
}

TEST(BusRegistryRelease, BackendTeardownCompletesBeforeIdentityCanBeReacquired)
{
    FakeHalBackend backend;
    CloseBarrier barrier;
    int calls = 0;
    auto id   = keyOf(22, 21);
    auto bus  = backend.busRegistry().acquireOrFind(id, [&]() -> v2::result_t<std::shared_ptr<v2::bus::IBus>> {
        ++calls;
        return std::shared_ptr<v2::bus::IBus>{std::make_shared<BlockingCloseI2cBus>(barrier)};
    });
    ASSERT_TRUE(bus.has_value());

    v2::result_t<void> close_result;
    std::thread closer{[&] { close_result = backend.closeBus(v2::types::bus_kind_t::I2C, id, bus.value()); }};
    {
        std::unique_lock<std::mutex> lock{barrier.mutex};
        barrier.cv.wait(lock, [&] { return barrier.entered; });
    }

    auto during_teardown = backend.busRegistry().acquireOrFind(id, CountingMaker{&calls});
    ASSERT_FALSE(during_teardown.has_value());
    EXPECT_EQ(during_teardown.error(), v2::error::error_t::BUSY);
    EXPECT_EQ(calls, 1);

    {
        std::lock_guard<std::mutex> lock{barrier.mutex};
        barrier.proceed = true;
        barrier.cv.notify_all();
    }
    closer.join();
    ASSERT_TRUE(close_result.has_value());

    bus.value().reset();
    auto reacquired = backend.busRegistry().acquireOrFind(id, CountingMaker{&calls});
    ASSERT_TRUE(reacquired.has_value());
    EXPECT_EQ(calls, 2);
}

TEST(BusDirectClose, ConcurrentAndRepeatedCloseNeverRepeatBackendTeardown)
{
    CloseBarrier barrier;
    BlockingCloseI2cBus bus{barrier};
    v2::result_t<void> first;
    std::thread closer{[&] { first = bus.close(); }};
    {
        std::unique_lock<std::mutex> lock{barrier.mutex};
        barrier.cv.wait(lock, [&] { return barrier.entered; });
    }

    auto concurrent = bus.close();
    EXPECT_FALSE(concurrent.has_value());
    if (!concurrent.has_value()) {
        EXPECT_EQ(concurrent.error(), v2::error::error_t::BUSY);
    }
    EXPECT_EQ(bus.close_calls.load(), 1);

    {
        std::lock_guard<std::mutex> lock{barrier.mutex};
        barrier.proceed = true;
        barrier.cv.notify_all();
    }
    closer.join();
    ASSERT_TRUE(first.has_value());

    auto repeated = bus.close();
    ASSERT_FALSE(repeated.has_value());
    EXPECT_EQ(repeated.error(), v2::error::error_t::CLOSED);
    EXPECT_EQ(bus.close_calls.load(), 1);
}

TEST(BusDirectClose, ReleaseFailureQuarantinesAndReportsPartialOutcome)
{
    ReleaseFailingCloseI2cBus bus;

    auto failed = bus.close();
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error(), v2::error::error_t::IO_ERROR);
    EXPECT_EQ(bus.close_calls, 1);
    EXPECT_FALSE(bus.initializationIsAllowed());

    auto retried = bus.close();
    ASSERT_TRUE(retried.has_value()) << "err=" << v2::error::toString(retried.error());
    EXPECT_EQ(bus.close_calls, 2);
}

TEST(BusDirectClose, ActiveAccessReturnsBusyWithoutBackendTeardownThenCanClose)
{
    v2::i2c::Bus bus;
    int closes = 0;
    ASSERT_TRUE(bus.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeHwBackend(1, &closes)}).has_value());

    v2::i2c::MasterAccessConfig access_cfg;
    access_cfg.i2c_addr = 0x42;
    v2::i2c::MasterAccessor accessor{bus, access_cfg};
    auto begun = accessor.beginAccess(0);
    ASSERT_TRUE(begun.has_value()) << "err=" << v2::error::toString(begun.error());

    auto busy = bus.close();
    ASSERT_FALSE(busy.has_value());
    EXPECT_EQ(busy.error(), v2::error::error_t::BUSY);
    EXPECT_EQ(closes, 0);

    auto ended = accessor.endAccess(0);
    ASSERT_TRUE(ended.has_value()) << "err=" << v2::error::toString(ended.error());
    auto closed = bus.close();
    ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    EXPECT_EQ(closes, 1);
}

TEST(BusDirectClose, UartBothChannelAccessesFormCloseBarrier)
{
    CountingCloseUartBus bus;
    v2::uart::AccessConfig cfg;
    v2::uart::TxAccessor tx{bus, cfg};
    v2::uart::RxAccessor rx{bus, cfg};
    auto tx_begun = tx.beginAccess(0);
    ASSERT_TRUE(tx_begun.has_value()) << "err=" << v2::error::toString(tx_begun.error());
    auto rx_begun = rx.beginAccess(0);
    ASSERT_TRUE(rx_begun.has_value()) << "err=" << v2::error::toString(rx_begun.error());

    auto busy = bus.close();
    ASSERT_FALSE(busy.has_value());
    EXPECT_EQ(busy.error(), v2::error::error_t::BUSY);
    EXPECT_EQ(bus.close_calls, 0);

    auto tx_ended = tx.endAccess(0);
    ASSERT_TRUE(tx_ended.has_value()) << "err=" << v2::error::toString(tx_ended.error());
    auto rx_ended = rx.endAccess(0);
    ASSERT_TRUE(rx_ended.has_value()) << "err=" << v2::error::toString(rx_ended.error());
    auto closed = bus.close();
    ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    EXPECT_EQ(bus.close_calls, 1);

    auto after_close = tx.beginAccess(0);
    ASSERT_FALSE(after_close.has_value());
    EXPECT_EQ(after_close.error(), v2::error::error_t::CLOSED);
}

TEST(BusDirectClose, I2sBothChannelAccessesFormCloseBarrier)
{
    CountingCloseI2sBus bus;
    v2::i2s::AccessConfig cfg;
    v2::i2s::TxAccessor tx{bus, cfg};
    v2::i2s::RxAccessor rx{bus, cfg};
    auto tx_begun = tx.beginAccess(0);
    ASSERT_TRUE(tx_begun.has_value()) << "err=" << v2::error::toString(tx_begun.error());
    auto rx_begun = rx.beginAccess(0);
    ASSERT_TRUE(rx_begun.has_value()) << "err=" << v2::error::toString(rx_begun.error());

    auto busy = bus.close();
    ASSERT_FALSE(busy.has_value());
    EXPECT_EQ(busy.error(), v2::error::error_t::BUSY);
    EXPECT_EQ(bus.close_calls, 0);

    auto tx_ended = tx.endAccess(0);
    ASSERT_TRUE(tx_ended.has_value()) << "err=" << v2::error::toString(tx_ended.error());
    auto rx_ended = rx.endAccess(0);
    ASSERT_TRUE(rx_ended.has_value()) << "err=" << v2::error::toString(rx_ended.error());
    auto closed = bus.close();
    ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    EXPECT_EQ(bus.close_calls, 1);

    auto after_close = rx.beginAccess(0);
    ASSERT_FALSE(after_close.has_value());
    EXPECT_EQ(after_close.error(), v2::error::error_t::CLOSED);
}

TEST(BusRegistryRelease, ReusedSlotChangesGenerationAndRejectsStaleTicket)
{
    v2::bus::BusRegistry reg;
    int calls  = 0;
    auto first = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    ASSERT_TRUE(first.has_value());
    auto stale = reg.beginRelease(keyOf(22, 21), first.value());
    ASSERT_TRUE(stale.has_value());
    ASSERT_TRUE(reg.commitRelease(stale.value()).has_value());
    first.value().reset();

    auto second = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    ASSERT_TRUE(second.has_value());
    auto current = reg.beginRelease(keyOf(22, 21), second.value());
    ASSERT_TRUE(current.has_value());
    EXPECT_EQ(stale->entry.slot, current->entry.slot);
    EXPECT_NE(stale->entry.generation, current->entry.generation);

    auto stale_commit = reg.commitRelease(stale.value());
    ASSERT_FALSE(stale_commit.has_value());
    EXPECT_EQ(stale_commit.error(), v2::error::error_t::INVALID_STATE);
    ASSERT_TRUE(reg.cancelRelease(current.value()).has_value());
}

TEST(BusRegistryRelease, MaxGenerationRetiresSlotInsteadOfWrapping)
{
    v2::bus::BusRegistry reg;
    v2::bus::BusRegistryTestAccess::seedGeneration(reg, 0, std::numeric_limits<uint32_t>::max() - 1);
    int calls = 0;
    auto last = reg.acquireOrFind(keyOf(22, 21), CountingMaker{&calls});
    ASSERT_TRUE(last.has_value());
    auto last_ticket = reg.beginRelease(keyOf(22, 21), last.value());
    ASSERT_TRUE(last_ticket.has_value());
    EXPECT_EQ(last_ticket->entry.slot, 0u);
    EXPECT_EQ(last_ticket->entry.generation, std::numeric_limits<uint32_t>::max());
    ASSERT_TRUE(reg.commitRelease(last_ticket.value()).has_value());
    last.value().reset();

    auto next = reg.acquireOrFind(keyOf(18, 19), CountingMaker{&calls});
    ASSERT_TRUE(next.has_value());
    auto next_ticket = reg.beginRelease(keyOf(18, 19), next.value());
    ASSERT_TRUE(next_ticket.has_value());
    EXPECT_EQ(next_ticket->entry.slot, 1u);
    EXPECT_EQ(next_ticket->entry.generation, 1u);
}

TEST(BusViewClose, ConsumesSoleOwnerAndAllowsReacquire)
{
    FakeHalBackend backend;
    v2::i2c::BusView view{&backend};
    v2::i2c::BusConfig cfg;
    cfg.pin_scl = 22;
    cfg.pin_sda = 21;

    auto bus = view.acquire(cfg);
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    EXPECT_EQ(backend.make_calls, 1);
    std::weak_ptr<v2::i2c::IBus> closed_instance = bus.value();

    auto closed = view.close(bus.value());
    ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    EXPECT_FALSE(bus.value());
    EXPECT_TRUE(closed_instance.expired());
    EXPECT_EQ(backend.busRegistry().liveCount(), 0u);

    auto reacquired = view.acquire(cfg);
    ASSERT_TRUE(reacquired.has_value()) << "err=" << v2::error::toString(reacquired.error());
    EXPECT_TRUE(reacquired.value());
    EXPECT_EQ(backend.make_calls, 2);
}

TEST(BusViewClose, RegistryBoundBusRejectsDirectWrapperAndClosesThroughView)
{
    FakeHalBackend backend;
    v2::i2c::BusView view{&backend};
    v2::i2c::BusConfig cfg;
    cfg.pin_scl = 22;
    cfg.pin_sda = 21;

    auto candidate    = std::make_shared<RegistryBoundCloseI2cBus>();
    backend.next_bus  = candidate;
    auto closed_guard = std::weak_ptr<RegistryBoundCloseI2cBus>{candidate};
    candidate.reset();

    auto bus = view.acquire(cfg);
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    auto concrete = std::dynamic_pointer_cast<RegistryBoundCloseI2cBus>(bus.value());
    ASSERT_TRUE(concrete);
    EXPECT_EQ(concrete->bind_calls, 1);

    auto direct = concrete->close();
    ASSERT_FALSE(direct.has_value());
    EXPECT_EQ(direct.error(), v2::error::error_t::INVALID_STATE);
    EXPECT_EQ(concrete->close_calls, 0);

    concrete.reset();
    auto closed = view.close(bus.value());
    ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    EXPECT_FALSE(bus.value());
    EXPECT_TRUE(closed_guard.expired());
}

TEST(BusViewClose, DowncastFacadeCannotBypassRegistryCloseOrdering)
{
    FakeHalBackend backend;
    v2::i2c::BusView view{&backend};
    v2::i2c::BusConfig cfg;
    cfg.pin_scl = 22;
    cfg.pin_sda = 21;

    auto facade    = std::make_shared<v2::i2c::Bus>();
    auto scripted  = std::unique_ptr<ScriptedCloseI2cBus>{new ScriptedCloseI2cBus({v2::bus::CloseOutcome::success()})};
    auto* observed = scripted.get();
    auto installed = facade->swapBackend(std::move(scripted));
    ASSERT_TRUE(installed.has_value()) << "err=" << v2::error::toString(installed.error());
    backend.next_bus = facade;
    facade.reset();

    auto bus = view.acquire(cfg);
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    auto concrete = std::dynamic_pointer_cast<v2::i2c::Bus>(bus.value());
    ASSERT_TRUE(concrete);

    auto direct = concrete->close();
    ASSERT_FALSE(direct.has_value());
    EXPECT_EQ(direct.error(), v2::error::error_t::INVALID_STATE);
    EXPECT_EQ(observed->close_calls, 0);

    concrete.reset();
    auto closed = view.close(bus.value());
    ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    EXPECT_FALSE(bus.value());
}

TEST(BusViewClose, DistinguishesUnconnectedViewFromNullHandle)
{
    v2::i2c::BusView unconnected;
    auto bus           = std::shared_ptr<v2::i2c::IBus>{std::make_shared<FakeI2cBus>(22, 21)};
    auto not_connected = unconnected.close(bus);
    ASSERT_FALSE(not_connected.has_value());
    EXPECT_EQ(not_connected.error(), v2::error::error_t::NOT_CONNECTED);
    EXPECT_TRUE(bus);

    FakeHalBackend backend;
    v2::i2c::BusView connected{&backend};
    std::shared_ptr<v2::i2c::IBus> empty;
    auto invalid = connected.close(empty);
    ASSERT_FALSE(invalid.has_value());
    EXPECT_EQ(invalid.error(), v2::error::error_t::INVALID_ARGUMENT);
}

TEST(BusViewClose, CoOwnerAndAccessorReturnBusyWithoutConsumingCaller)
{
    FakeHalBackend backend;
    v2::i2c::BusView view{&backend};
    v2::i2c::BusConfig cfg;
    cfg.pin_scl = 22;
    cfg.pin_sda = 21;

    auto bus = view.acquire(cfg);
    ASSERT_TRUE(bus.has_value()) << "err=" << v2::error::toString(bus.error());
    auto* original = bus.value().get();
    {
        auto alias = bus.value();
        auto busy  = view.close(bus.value());
        ASSERT_FALSE(busy.has_value());
        EXPECT_EQ(busy.error(), v2::error::error_t::BUSY);
        EXPECT_EQ(bus.value().get(), original);
    }

    {
        v2::i2c::MasterAccessConfig access_cfg;
        access_cfg.i2c_addr = 0x42;
        v2::i2c::MasterAccessor accessor{bus.value(), access_cfg};
        auto busy = view.close(bus.value());
        ASSERT_FALSE(busy.has_value());
        EXPECT_EQ(busy.error(), v2::error::error_t::BUSY);
        EXPECT_EQ(bus.value().get(), original);
    }

    auto closed = view.close(bus.value());
    ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    EXPECT_FALSE(bus.value());
}

TEST(BusViewClose, ForeignBusWithSameIdentityIsInvalidArgument)
{
    FakeHalBackend backend;
    v2::i2c::BusView view{&backend};
    v2::i2c::BusConfig cfg;
    cfg.pin_scl = 22;
    cfg.pin_sda = 21;

    auto registered = view.acquire(cfg);
    ASSERT_TRUE(registered.has_value()) << "err=" << v2::error::toString(registered.error());
    auto foreign = std::shared_ptr<v2::i2c::IBus>{std::make_shared<FakeI2cBus>(22, 21)};

    auto invalid = view.close(foreign);
    ASSERT_FALSE(invalid.has_value());
    EXPECT_EQ(invalid.error(), v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_TRUE(foreign);

    auto same = view.acquire(cfg);
    ASSERT_TRUE(same.has_value()) << "err=" << v2::error::toString(same.error());
    EXPECT_EQ(same.value().get(), registered.value().get());
    EXPECT_EQ(backend.make_calls, 1);
}

TEST(BusViewClose, NoMutationFailureRollsBackLiveAndCanRetry)
{
    FakeHalBackend backend;
    v2::i2c::BusView view{&backend};
    v2::i2c::BusConfig cfg;
    cfg.pin_scl      = 22;
    cfg.pin_sda      = 21;
    auto scripted    = std::make_shared<ScriptedCloseI2cBus>(std::vector<v2::bus::CloseOutcome>{
        v2::bus::CloseOutcome::noMutation(v2::error::error_t::TIMEOUT_ERROR),
        v2::bus::CloseOutcome::success(),
    });
    backend.next_bus = scripted;

    auto bus = view.acquire(cfg);
    ASSERT_TRUE(bus.has_value());
    scripted.reset();

    auto first = view.close(bus.value());
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), v2::error::error_t::TIMEOUT_ERROR);
    ASSERT_TRUE(bus.value());

    auto live = backend.busRegistry().findByIdentity(keyOf(22, 21));
    ASSERT_TRUE(live);
    EXPECT_EQ(live.get(), bus.value().get());
    live.reset();

    auto retried = view.close(bus.value());
    ASSERT_TRUE(retried.has_value());
    EXPECT_FALSE(bus.value());
    EXPECT_EQ(backend.busRegistry().liveCount(), 0u);
}

TEST(BusViewClose, PartialFailureQuarantinesAcquireAndSameHandleCanRetry)
{
    FakeHalBackend backend;
    v2::i2c::BusView view{&backend};
    v2::i2c::BusConfig cfg;
    cfg.pin_scl      = 22;
    cfg.pin_sda      = 21;
    auto scripted    = std::make_shared<ScriptedCloseI2cBus>(std::vector<v2::bus::CloseOutcome>{
        v2::bus::CloseOutcome::partialOrUnknown(v2::error::error_t::IO_ERROR),
        v2::bus::CloseOutcome::success(),
    });
    backend.next_bus = scripted;

    auto bus = view.acquire(cfg);
    ASSERT_TRUE(bus.has_value());
    scripted.reset();

    auto first = view.close(bus.value());
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), v2::error::error_t::IO_ERROR);
    ASSERT_TRUE(bus.value());
    EXPECT_EQ(backend.busRegistry().liveCount(), 1u);

    auto blocked = view.acquire(cfg);
    ASSERT_FALSE(blocked.has_value());
    EXPECT_EQ(blocked.error(), v2::error::error_t::BUSY);

    v2::i2c::MasterAccessConfig access_cfg;
    access_cfg.i2c_addr = 0x42;
    v2::i2c::MasterAccessor accessor{*bus.value(), access_cfg};
    auto operation = accessor.beginAccess(0);
    ASSERT_FALSE(operation.has_value());
    EXPECT_EQ(operation.error(), v2::error::error_t::CLOSED);

    auto retried = view.close(bus.value());
    ASSERT_TRUE(retried.has_value());
    EXPECT_FALSE(bus.value());
    EXPECT_EQ(backend.busRegistry().liveCount(), 0u);
}

TEST(BusViewClose, DroppedQuarantinedFacadeRetriesTeardownAndReclaimsSlot)
{
    FakeHalBackend backend;
    v2::i2c::BusView view{&backend};
    v2::i2c::BusConfig cfg;
    cfg.pin_scl = 22;
    cfg.pin_sda = 21;

    auto facade    = std::make_shared<v2::i2c::Bus>();
    auto scripted  = std::unique_ptr<ScriptedCloseI2cBus>{new ScriptedCloseI2cBus({
        v2::bus::CloseOutcome::partialOrUnknown(v2::error::error_t::IO_ERROR),
        v2::bus::CloseOutcome::success(),
    })};
    auto* observed = scripted.get();
    ASSERT_TRUE(facade->swapBackend(std::move(scripted)).has_value());
    backend.next_bus = facade;

    auto bus = view.acquire(cfg);
    ASSERT_TRUE(bus.has_value());
    facade.reset();

    auto first = view.close(bus.value());
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), v2::error::error_t::IO_ERROR);
    EXPECT_EQ(observed->close_calls, 1);
    EXPECT_EQ(backend.busRegistry().liveCount(), 1u);

    // No explicit retry: the facade destructor retries the still-owned
    // backend. Confirmed success releases the quarantined registry slot.
    bus.value().reset();
    EXPECT_EQ(backend.busRegistry().liveCount(), 0u);

    auto reacquired = view.acquire(cfg);
    ASSERT_TRUE(reacquired.has_value());
    EXPECT_EQ(backend.make_calls, 2);
}

// ---- backend query API defaults ------------------------

TEST(BusBackendQuery, DefaultsAreSafeForUnmigratedBus)
{
    // A bus that has not opted into the backend model answers the
    // safe defaults: software / no controller / unknown ceiling / never
    // swapped. This keeps the query API harmless for every kind (and test
    // fake) that inherits it before implementing.
    FakeI2cBus bus;
    v2::bus::IBus& base = bus;
    EXPECT_EQ(base.backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(base.controllerId(), -1);
    EXPECT_EQ(base.maxFrequency(), 0u);
    EXPECT_EQ(base.backendGeneration(), 0u);
}

TEST(BusCapabilities, IsBoundedOwnedAndRejectsAbsentOrUnknownValues)
{
    static_assert(sizeof(v2::bus::BusCapabilities) == 28);
    static_assert(std::is_trivially_copyable_v<v2::bus::BusCapabilities>);

    const auto caps = v2::bus::detail::BusCapabilitiesBuilder{}
                          .enable(v2::bus::BusFeature::Transmit)
                          .setLimit(v2::bus::BusLimit::MaxAtomicTxBytes, 243)
                          .setGeneration(17)
                          .build();
    EXPECT_TRUE(caps.supports(v2::bus::BusFeature::Transmit));
    EXPECT_FALSE(caps.supports(v2::bus::BusFeature::Receive));
    EXPECT_FALSE(caps.supports(static_cast<v2::bus::BusFeature>(31)));
    EXPECT_FALSE(caps.supports(static_cast<v2::bus::BusFeature>(32)));
    ASSERT_TRUE(caps.limit(v2::bus::BusLimit::MaxAtomicTxBytes).has_value());
    EXPECT_EQ(caps.limit(v2::bus::BusLimit::MaxAtomicTxBytes).value(), 243u);
    ASSERT_FALSE(caps.limit(v2::bus::BusLimit::MaxAtomicRxBytes).has_value());
    EXPECT_EQ(caps.limit(v2::bus::BusLimit::MaxAtomicRxBytes).error(), v2::error::error_t::UNSUPPORTED);
    ASSERT_FALSE(caps.limit(static_cast<v2::bus::BusLimit>(4)).has_value());
    EXPECT_EQ(caps.limit(static_cast<v2::bus::BusLimit>(4)).error(), v2::error::error_t::UNSUPPORTED);
    EXPECT_EQ(caps.generation(), 17u);
}

TEST(BusCapabilities, UnmigratedKindBaseFailsClosedWithoutInventingFeaturesOrLimits)
{
    FakeI2cBus bus;
    const auto caps = bus.capabilities();
    EXPECT_FALSE(caps.supports(v2::bus::BusFeature::MasterTransfer));
    EXPECT_FALSE(caps.supports(v2::bus::BusFeature::Transmit));
    EXPECT_FALSE(caps.supports(v2::bus::BusFeature::Receive));
    EXPECT_FALSE(caps.supports(v2::bus::BusFeature::HardwareBackend));
    EXPECT_FALSE(caps.limit(v2::bus::BusLimit::MaxFrequencyHz).has_value());
    EXPECT_EQ(caps.generation(), 0u);
}

// ---- M5_Hal.I2C BusView integration (real software backend) --------------

TEST(I2cBusView, AcquireInternsAndDrivesBackend)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    const auto added = runner.add(slave.service());
    ASSERT_TRUE(added.has_value()) << "err=" << v2::error::toString(added.error());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    auto& hal = v2::getM5_Hal();

    // Portable config -> the selected local provider picks the bit-bang backend.
    v2::i2c::BusConfig cfg;
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
    // selected software acquire reports software with no controller.
    EXPECT_EQ(a.value()->backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(a.value()->controllerId(), -1);
}

// ---- M5_Hal.I2C logical acquire (pins + intent,) ---------

TEST(I2cBusViewLogical, LogicalAcquireCreatesSoftwareBusAndInterns)
{
    using namespace service_proto;

    VirtualOpenDrainBus lines;
    ServiceRunner runner;
    SlaveEndpoint slave{lines, 0x42};
    const auto added = runner.add(slave.service());
    ASSERT_TRUE(added.has_value()) << "err=" << v2::error::toString(added.error());
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

    // Identity is the pins alone: a portable acquire of the SAME wiring
    // shares the interned instance (first backend wins).
    v2::i2c::BusConfig portable;
    portable.pin_scl = gpio.scl();
    portable.pin_sda = gpio.sda();
    auto b           = hal.I2C.acquire(portable);
    ASSERT_TRUE(b.has_value());
    EXPECT_EQ(a.value().get(), b.value().get());  // same wiring -> same bus
}

// ---- i2c::Bus facade hot-swap --------------------------

TEST(I2cFacadeSwap, SwapBackendTracksQueryAndBumpsGeneration)
{
    v2::i2c::Bus facade;
    // Start on the selected provider via the portable init. The pins
    // are arbitrary -- no wire I/O is performed in this test.
    v2::i2c::BusConfig sw;
    sw.pin_scl = 22;
    sw.pin_sda = 21;
    ASSERT_TRUE(facade.init(sw).has_value());
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(facade.backendGeneration(), 1u);
    const auto software_caps = facade.capabilities();
    EXPECT_TRUE(software_caps.supports(v2::bus::BusFeature::ManagedAllocation));
    EXPECT_TRUE(software_caps.supports(v2::bus::BusFeature::MasterTransfer));
    EXPECT_FALSE(software_caps.supports(v2::bus::BusFeature::HardwareBackend));
    EXPECT_EQ(software_caps.generation(), 1u);

    int closes_a = 0;
    int closes_b = 0;

    // Swap to a fake hardware backend: the query API tracks the new backend
    // and the generation counter advances.
    ASSERT_TRUE(facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeHwBackend(1, &closes_a)}).has_value());
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Hardware);
    EXPECT_EQ(facade.controllerId(), 1);
    EXPECT_EQ(facade.maxFrequency(), 400000u);
    EXPECT_EQ(facade.backendGeneration(), 2u);
    const auto hardware_caps = facade.capabilities();
    EXPECT_TRUE(hardware_caps.supports(v2::bus::BusFeature::ManagedAllocation));
    EXPECT_TRUE(hardware_caps.supports(v2::bus::BusFeature::HardwareBackend));
    ASSERT_TRUE(hardware_caps.limit(v2::bus::BusLimit::MaxFrequencyHz).has_value());
    EXPECT_EQ(hardware_caps.limit(v2::bus::BusLimit::MaxFrequencyHz).value(), 400000u);
    EXPECT_EQ(hardware_caps.generation(), 2u);
    // A snapshot owns its values; a later hot-swap cannot mutate it.
    EXPECT_FALSE(software_caps.supports(v2::bus::BusFeature::HardwareBackend));
    EXPECT_EQ(software_caps.generation(), 1u);
    EXPECT_EQ(closes_a, 0);  // backend A is live, not closed yet

    // A second swap tears down backend A and advances the generation again.
    ASSERT_TRUE(facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeHwBackend(0, &closes_b)}).has_value());
    EXPECT_EQ(closes_a, 1);  // old backend closed on swap
    EXPECT_EQ(facade.controllerId(), 0);
    EXPECT_EQ(facade.backendGeneration(), 3u);

    // A null backend is rejected without disturbing the live one.
    auto bad = facade.swapBackend(nullptr);
    EXPECT_FALSE(bad.has_value());
    EXPECT_EQ(bad.error(), v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_EQ(facade.controllerId(), 0);
    EXPECT_EQ(facade.backendGeneration(), 3u);
}

TEST(I2cFacadeCapabilities, DirectCloseAndReinitPublishDistinctGenerations)
{
    v2::i2c::Bus facade;
    v2::i2c::BusConfig cfg;
    cfg.pin_scl = 22;
    cfg.pin_sda = 21;
    ASSERT_TRUE(
        facade.adoptPortableBackend(std::unique_ptr<v2::i2c::IBus>{new FakeHwBackend(1, nullptr)}, cfg).has_value());
    const auto first = facade.capabilities();
    EXPECT_EQ(first.generation(), 1u);
    EXPECT_TRUE(first.supports(v2::bus::BusFeature::HardwareBackend));
    ASSERT_TRUE(first.limit(v2::bus::BusLimit::MaxFrequencyHz).has_value());
    EXPECT_EQ(first.limit(v2::bus::BusLimit::MaxFrequencyHz).value(), 400000u);

    ASSERT_TRUE(facade.close().has_value());
    const auto closed = facade.capabilities();
    EXPECT_NE(closed.generation(), first.generation());
    EXPECT_FALSE(closed.supports(v2::bus::BusFeature::MasterTransfer));
    EXPECT_FALSE(closed.supports(v2::bus::BusFeature::HardwareBackend));
    EXPECT_FALSE(closed.limit(v2::bus::BusLimit::MaxFrequencyHz).has_value());
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(facade.maxFrequency(), 0u);

    ASSERT_TRUE(facade.init(cfg).has_value());
    const auto reopened = facade.capabilities();
    EXPECT_NE(reopened.generation(), closed.generation());
    EXPECT_FALSE(reopened.supports(v2::bus::BusFeature::HardwareBackend));
    EXPECT_TRUE(reopened.supports(v2::bus::BusFeature::MasterTransfer));
    EXPECT_EQ(first.generation(), 1u);
}

TEST(I2cFacadeCapabilities, ConcurrentQueriesObserveOnlyCommittedSnapshots)
{
    v2::i2c::Bus facade;
    ASSERT_TRUE(
        facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new CoherentCapabilityBackend(false)}, 1000).has_value());
    std::atomic<bool> start{false};
    std::atomic<bool> reader_ready{false};
    std::atomic<bool> done{false};
    std::atomic<bool> failed{false};
    std::atomic<size_t> observations{0};

    std::thread reader{[&] {
        while (!start.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        uint32_t previous_generation = 0;
        while (!done.load(std::memory_order_acquire)) {
            const auto snapshot = facade.capabilities();
            reader_ready.store(true, std::memory_order_release);
            observations.fetch_add(1, std::memory_order_relaxed);
            auto tx_limit = snapshot.limit(v2::bus::BusLimit::MaxAtomicTxBytes);
            if (!tx_limit.has_value() ||
                tx_limit.value() != (snapshot.supports(v2::bus::BusFeature::LowPowerBackend) ? 111u : 222u) ||
                snapshot.generation() < previous_generation) {
                failed.store(true, std::memory_order_release);
                break;
            }
            previous_generation = snapshot.generation();
        }
    }};

    start.store(true, std::memory_order_release);
    while (!reader_ready.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
    for (size_t i = 0; i < 500; ++i) {
        auto swapped =
            facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new CoherentCapabilityBackend((i & 1u) != 0)}, 1000);
        if (!swapped.has_value()) {
            failed.store(true, std::memory_order_release);
            break;
        }
    }
    done.store(true, std::memory_order_release);
    reader.join();
    EXPECT_FALSE(failed.load(std::memory_order_acquire));
    EXPECT_GT(observations.load(std::memory_order_relaxed), 1u);
}

TEST(I2cFacadeSwap, NoMutationCloseFailureKeepsFacadeLiveAndPropagatesExactError)
{
    v2::i2c::Bus facade;
    auto scripted  = std::unique_ptr<ScriptedCloseI2cBus>{new ScriptedCloseI2cBus({
        v2::bus::CloseOutcome::noMutation(v2::error::error_t::TIMEOUT_ERROR),
        v2::bus::CloseOutcome::success(),
    })};
    auto* observed = scripted.get();
    auto installed = facade.swapBackend(std::move(scripted));
    ASSERT_TRUE(installed.has_value()) << "err=" << v2::error::toString(installed.error());

    auto failed = facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeHwBackend(1, nullptr)});
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error(), v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_EQ(observed->close_calls, 1);

    v2::i2c::MasterAccessConfig access_cfg;
    access_cfg.i2c_addr = 0x42;
    v2::i2c::MasterAccessor accessor{facade, access_cfg};
    auto begun = accessor.beginAccess(0);
    ASSERT_TRUE(begun.has_value()) << "err=" << v2::error::toString(begun.error());
    auto ended = accessor.endAccess(0);
    ASSERT_TRUE(ended.has_value()) << "err=" << v2::error::toString(ended.error());

    auto retried = facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeHwBackend(0, nullptr)});
    ASSERT_TRUE(retried.has_value()) << "err=" << v2::error::toString(retried.error());
}

TEST(I2cFacadeSwap, PartialCloseFailureQuarantinesFacadeUntilCloseRetry)
{
    v2::i2c::Bus facade;
    auto scripted  = std::unique_ptr<ScriptedCloseI2cBus>{new ScriptedCloseI2cBus({
        v2::bus::CloseOutcome::partialOrUnknown(v2::error::error_t::IO_ERROR),
        v2::bus::CloseOutcome::success(),
    })};
    auto* observed = scripted.get();
    auto installed = facade.swapBackend(std::move(scripted));
    ASSERT_TRUE(installed.has_value()) << "err=" << v2::error::toString(installed.error());

    auto failed = facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeHwBackend(1, nullptr)});
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error(), v2::error::error_t::IO_ERROR);
    EXPECT_EQ(observed->close_calls, 1);

    v2::i2c::MasterAccessConfig access_cfg;
    access_cfg.i2c_addr = 0x42;
    v2::i2c::MasterAccessor accessor{facade, access_cfg};
    auto rejected = accessor.beginAccess(0);
    ASSERT_FALSE(rejected.has_value());
    EXPECT_EQ(rejected.error(), v2::error::error_t::CLOSED);

    auto retried = facade.close();
    ASSERT_TRUE(retried.has_value()) << "err=" << v2::error::toString(retried.error());
}

TEST(I2cFacadeClose, FailureKeepsBackendForRetry)
{
    v2::i2c::Bus facade;
    int closes = 0;
    ASSERT_TRUE(facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FailingCloseHwBackend(1, &closes)}).has_value());
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Hardware);
    EXPECT_EQ(facade.controllerId(), 1);
    EXPECT_EQ(facade.backendGeneration(), 1u);

    auto first = facade.close();
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), v2::error::error_t::IO_ERROR);
    EXPECT_EQ(closes, 1);
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Hardware);
    EXPECT_EQ(facade.controllerId(), 1);
    EXPECT_EQ(facade.backendGeneration(), 1u);

    auto retry = facade.close();
    ASSERT_FALSE(retry.has_value());
    EXPECT_EQ(retry.error(), v2::error::error_t::IO_ERROR);
    EXPECT_EQ(closes, 2);
}

TEST(I2cFacadeClose, PreservesOwnedBackendNoMutationClassification)
{
    v2::i2c::Bus facade;
    auto scripted  = std::unique_ptr<ScriptedCloseI2cBus>{new ScriptedCloseI2cBus({
        v2::bus::CloseOutcome::noMutation(v2::error::error_t::TIMEOUT_ERROR),
        v2::bus::CloseOutcome::success(),
    })};
    auto* observed = scripted.get();
    auto swapped   = facade.swapBackend(std::move(scripted));
    ASSERT_TRUE(swapped.has_value()) << "err=" << v2::error::toString(swapped.error());

    auto first = facade.close();
    ASSERT_FALSE(first.has_value());
    EXPECT_EQ(first.error(), v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_EQ(observed->close_calls, 1);

    auto retried = facade.close();
    ASSERT_TRUE(retried.has_value()) << "err=" << v2::error::toString(retried.error());
}

TEST(I2cFacadeClose, DefaultCloseIsNoopSuccess)
{
    v2::i2c::Bus facade;
    ASSERT_TRUE(facade.swapBackend(std::unique_ptr<v2::i2c::IBus>{new FakeI2cBus()}).has_value());

    EXPECT_TRUE(facade.close().has_value());
    auto repeated = facade.close();
    ASSERT_FALSE(repeated.has_value());
    EXPECT_EQ(repeated.error(), v2::error::error_t::CLOSED);
}

TEST(I2cFacadeClose, DirectCloseThenInitReopensForAccess)
{
    v2::i2c::Bus bus;
    v2::i2c::BusConfig cfg;
    cfg.pin_scl      = 22;
    cfg.pin_sda      = 21;
    auto initialized = bus.init(cfg);
    ASSERT_TRUE(initialized.has_value()) << "err=" << v2::error::toString(initialized.error());

    auto closed = bus.close();
    ASSERT_TRUE(closed.has_value()) << "err=" << v2::error::toString(closed.error());
    auto reopened = bus.init(cfg);
    ASSERT_TRUE(reopened.has_value()) << "err=" << v2::error::toString(reopened.error());

    v2::i2c::MasterAccessConfig access_cfg;
    access_cfg.i2c_addr = 0x42;
    v2::i2c::MasterAccessor accessor{bus, access_cfg};
    auto begun = accessor.beginAccess(0);
    ASSERT_TRUE(begun.has_value()) << "err=" << v2::error::toString(begun.error());
    auto ended = accessor.endAccess(0);
    ASSERT_TRUE(ended.has_value()) << "err=" << v2::error::toString(ended.error());

    auto reclosed = bus.close();
    ASSERT_TRUE(reclosed.has_value()) << "err=" << v2::error::toString(reclosed.error());
}

// ---- Accessor co-ownership of an acquired bus (canonical registry path) ---
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
    const auto added = runner.add(slave.service());
    ASSERT_TRUE(added.has_value()) << "err=" << v2::error::toString(added.error());
    lines.setRunner(&runner);

    VirtualI2CPort scl_port{lines, VirtualI2CPort::Line::SCL};
    VirtualI2CPort sda_port{lines, VirtualI2CPort::Line::SDA};
    ScopedVirtualI2CGPIO gpio{scl_port, sda_port};

    auto& hal = v2::getM5_Hal();
    v2::i2c::BusConfig cfg;
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
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
