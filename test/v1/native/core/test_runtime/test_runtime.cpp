// SPDX-License-Identifier: MIT
// runtime kind: time / mutex contracts on the native host.
//
// The flat-injected runtime here is the posix variant (CLOCK_MONOTONIC
// + std::timed_mutex), so the time assertions use generous lower
// bounds only (no upper bounds — CI machines stall). Deterministic
// assertions go through the variant-qualified stub fake. The Bus
// integration tests pin the S7 BREAKING semantics: contention WAITS
// and fails with TIMEOUT_ERROR (not BUSY), timeout 0 is a try-lock.
#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

#include <atomic>
#include <chrono>
#include <thread>

namespace {

namespace runtime = ::m5::hal::v1::runtime;
namespace stub_rt = ::m5::variants::frameworks::stub::hal::v1::runtime;

// ---- selection ------------------------------------------------------------

static_assert(M5HAL_V1_SELECTED_VARIANT_RUNTIME == M5HAL_V1_VARIANT_ID_FRAMEWORK_POSIX,
              "scan order: posix provides runtime on a plain host build");

TEST(RuntimeSelection, FlatInjectionIsThePosixVariant)
{
    constexpr bool same =
        std::is_same<runtime::Mutex, ::m5::variants::frameworks::posix::hal::v1::runtime::Mutex>::value;
    EXPECT_TRUE(same);
}

// ---- time (real clock through the posix variant) ---------------------------

TEST(RuntimeTime, DelayMsAdvancesMillis)
{
    const uint32_t m0 = runtime::millis();
    const uint32_t u0 = runtime::micros();
    runtime::delayMs(20);
    EXPECT_GE(runtime::millis() - m0, 15u);
    EXPECT_GE(runtime::micros() - u0, 15000u);
}

TEST(RuntimeTime, DelayUsAdvancesMicros)
{
    const uint32_t u0 = runtime::micros();
    runtime::delayUs(2000);
    EXPECT_GE(runtime::micros() - u0, 1500u);
}

// ---- time (deterministic stub fake) ----------------------------------------

TEST(RuntimeStubTime, FakeClockIsDeterministic)
{
    stub_rt::fakeReset();
    EXPECT_EQ(stub_rt::micros(), 0u);
    EXPECT_EQ(stub_rt::millis(), 0u);

    stub_rt::delayUs(123);
    EXPECT_EQ(stub_rt::micros(), 123u);
    EXPECT_EQ(stub_rt::millis(), 0u);

    stub_rt::delayMs(2);
    EXPECT_EQ(stub_rt::micros(), 2123u);
    EXPECT_EQ(stub_rt::millis(), 2u);

    stub_rt::fakeReset();
    EXPECT_EQ(stub_rt::micros(), 0u);
}

// ---- mutex (posix std::timed_mutex) ----------------------------------------

TEST(RuntimeMutex, TryLockAndRelease)
{
    runtime::Mutex m;
    EXPECT_TRUE(m.lock(0));
    m.unlock();
    EXPECT_TRUE(m.lock(0));
    m.unlock();
}

TEST(RuntimeMutex, NonRecursiveRelockTimesOut)
{
    runtime::Mutex m;
    ASSERT_TRUE(m.lock(0));
    // Re-lock from the holding thread: waits until the timeout, then
    // fails (non-recursive).
    EXPECT_FALSE(m.lock(30));
    m.unlock();
    EXPECT_TRUE(m.lock(0));
    m.unlock();
}

TEST(RuntimeMutex, ContendedLockTimesOutThenSucceedsAfterRelease)
{
    runtime::Mutex m;
    std::atomic<bool> held{false};
    std::atomic<bool> release{false};

    std::thread holder([&] {
        ASSERT_TRUE(m.lock(0));
        held = true;
        while (!release) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        m.unlock();
    });
    while (!held) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Contention: the wait is real (lower bound only), then times out.
    const uint32_t t0 = runtime::millis();
    EXPECT_FALSE(m.lock(50));
    EXPECT_GE(runtime::millis() - t0, 40u);

    // The holder releases; a waiting lock with budget succeeds.
    release = true;
    EXPECT_TRUE(m.lock(2000));
    m.unlock();
    holder.join();
}

// ---- mutex (stub single-task guard) ----------------------------------------

TEST(RuntimeStubMutex, ContentionFailsImmediately)
{
    stub_rt::Mutex m;
    ASSERT_TRUE(m.lock(0));
    // Single-task fake: a timeout cannot resolve without another task,
    // so even a large budget fails immediately (deterministic).
    EXPECT_FALSE(m.lock(10000));
    m.unlock();
    EXPECT_TRUE(m.lock(0));
    m.unlock();
}

// ---- Bus integration (BREAKING: contention -> TIMEOUT_ERROR) ------------

struct TestBusConfig : public m5::hal::v1::bus::IBusConfig {
    TestBusConfig(void) : m5::hal::v1::bus::IBusConfig{m5::hal::v1::types::bus_kind_t::I2C}
    {
    }
};

struct TestBus : public m5::hal::v1::bus::IBus {
    const m5::hal::v1::bus::IBusConfig& getConfig(void) const override
    {
        return _cfg;
    }
    TestBusConfig _cfg;
};

struct TestAccessConfig : public m5::hal::v1::bus::IAccessConfig {
    TestAccessConfig(void) : m5::hal::v1::bus::IAccessConfig{m5::hal::v1::types::bus_kind_t::I2C}
    {
    }
};

struct TestAccessor : public m5::hal::v1::bus::IAccessor {
    explicit TestAccessor(m5::hal::v1::bus::IBus& bus) : IAccessor{bus}
    {
    }
    const m5::hal::v1::bus::IAccessConfig& getConfig(void) const override
    {
        return _cfg;
    }
    TestAccessConfig _cfg;
};

TEST(BusLock, ContendedTryLockReportsTimeoutError)
{
    TestBus bus;
    TestAccessor a1{bus};
    TestAccessor a2{bus};

    ASSERT_TRUE(bus.lock(&a1, 0).has_value());
    auto r = bus.lock(&a2, 0);  // try-lock: fails immediately under contention
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v1::error::error_t::TIMEOUT_ERROR);
    ASSERT_TRUE(bus.unlock(&a1).has_value());
}

TEST(BusLock, ContendedLockWaitsForHolderAcrossTasks)
{
    TestBus bus;
    TestAccessor holder_acc{bus};
    TestAccessor waiter_acc{bus};
    std::atomic<bool> held{false};

    // The holder task takes the bus for ~150 ms; the waiter's budget
    // (2 s) covers it, so the waiter must succeed WITHOUT an error —
    // the S7 "waiting lock" semantics, impossible with the old
    // owner-pointer implementation.
    std::thread holder([&] {
        ASSERT_TRUE(bus.lock(&holder_acc, 0).has_value());
        held = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        ASSERT_TRUE(bus.unlock(&holder_acc).has_value());
    });
    while (!held) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const uint32_t t0 = runtime::millis();
    auto r            = bus.lock(&waiter_acc, 2000);
    ASSERT_TRUE(r.has_value());
    EXPECT_GE(runtime::millis() - t0, 50u);  // it really waited
    ASSERT_TRUE(bus.unlock(&waiter_acc).has_value());
    holder.join();
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
