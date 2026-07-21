// SPDX-License-Identifier: MIT
// runtime kind: time / mutex contracts on the native host.
//
// The flat-injected runtime here is the posix variant (CLOCK_MONOTONIC
// + std::timed_mutex), so the time assertions use generous lower
// bounds only (no upper bounds — CI machines stall). Deterministic
// assertions go through the variant-qualified stub fake. The Bus
// integration tests pin the BREAKING lock semantics: contention WAITS
// and fails with TIMEOUT_ERROR (not BUSY), timeout 0 is a try-lock.
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include <atomic>
#include <chrono>
#include <thread>

namespace {

namespace runtime = ::m5::hal::v2::runtime;
namespace stub_rt = ::m5::variants::frameworks::stub::hal::v2::runtime;

struct TaskGate {
    std::atomic<bool> entered{false};
    std::atomic<bool> release{false};
};

void gatedTask(void* raw)
{
    auto& gate = *static_cast<TaskGate*>(raw);
    gate.entered.store(true, std::memory_order_release);
    while (!gate.release.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
}

void incrementTask(void* raw)
{
    static_cast<std::atomic<unsigned>*>(raw)->fetch_add(1, std::memory_order_relaxed);
}

template <typename Task>
void expectTaskStartContract()
{
    Task task;

    auto invalid = task.start(nullptr, nullptr);
    ASSERT_FALSE(invalid.has_value());
    EXPECT_EQ(invalid.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    EXPECT_FALSE(task.joinable());

    TaskGate gate;
    auto started = task.start(&gatedTask, &gate);
    ASSERT_TRUE(started.has_value());
    while (!gate.entered.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }
    EXPECT_TRUE(task.joinable());

    auto invalid_while_joinable = task.start(nullptr, nullptr);
    ASSERT_FALSE(invalid_while_joinable.has_value());
    EXPECT_EQ(invalid_while_joinable.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    auto duplicate = task.start(&gatedTask, &gate);
    ASSERT_FALSE(duplicate.has_value());
    EXPECT_EQ(duplicate.error(), ::m5::hal::v2::error::error_t::INVALID_STATE);

    gate.release.store(true, std::memory_order_release);
    task.join();
    EXPECT_FALSE(task.joinable());

    std::atomic<unsigned> calls{0};
    auto restarted = task.start(&incrementTask, &calls);
    ASSERT_TRUE(restarted.has_value());
    task.join();
    EXPECT_EQ(calls.load(std::memory_order_relaxed), 1u);

    task.join();  // idempotent no-op
    EXPECT_FALSE(task.joinable());
}

// ---- selection ------------------------------------------------------------

static_assert(M5HAL_V2_SELECTED_VARIANT_RUNTIME == M5HAL_V2_VARIANT_ID_FRAMEWORK_POSIX,
              "scan order: posix provides runtime on a plain host build");

TEST(RuntimeSelection, FlatInjectionIsThePosixVariant)
{
    constexpr bool same =
        std::is_same<runtime::Mutex, ::m5::variants::frameworks::posix::hal::v2::runtime::Mutex>::value;
    EXPECT_TRUE(same);
}

TEST(RuntimeTask, PosixStartReportsStateAndArgumentErrors)
{
    expectTaskStartContract<runtime::Task>();
}

TEST(RuntimeTask, ThreadedStubMatchesTheStartContract)
{
    expectTaskStartContract<stub_rt::Task>();
}

TEST(RuntimeTask, HostThreadErrorClassificationIsDeterministic)
{
    using error_t        = ::m5::hal::v2::error::error_t;
    const auto exhausted = std::make_error_code(std::errc::resource_unavailable_try_again);
    const auto no_memory = std::make_error_code(std::errc::not_enough_memory);
    const auto other     = std::make_error_code(std::errc::permission_denied);

    EXPECT_EQ(::m5::variants::frameworks::detail::mapThreadCreateError(exhausted), error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(::m5::variants::frameworks::detail::mapThreadCreateError(no_memory), error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(::m5::variants::frameworks::detail::mapThreadCreateError(other), error_t::IO_ERROR);
}

TEST(RuntimeTask, ThreadlessStubRejectsLaunchWithoutInventingState)
{
    stub_rt::ThreadlessTask task;
    auto invalid = task.start(nullptr, nullptr);
    ASSERT_FALSE(invalid.has_value());
    EXPECT_EQ(invalid.error(), ::m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    std::atomic<unsigned> calls{0};
    auto unsupported = task.start(&incrementTask, &calls);
    ASSERT_FALSE(unsupported.has_value());
    EXPECT_EQ(unsupported.error(), ::m5::hal::v2::error::error_t::UNSUPPORTED);
    EXPECT_FALSE(task.joinable());
    task.join();
    EXPECT_EQ(calls.load(std::memory_order_relaxed), 0u);
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
    auto locked = m.lock(0);
    ASSERT_TRUE(locked.has_value()) << "err=" << ::m5::hal::v2::error::toString(locked.error());
    auto unlocked = m.unlock();
    ASSERT_TRUE(unlocked.has_value()) << "err=" << ::m5::hal::v2::error::toString(unlocked.error());
    locked = m.lock(0);
    ASSERT_TRUE(locked.has_value()) << "err=" << ::m5::hal::v2::error::toString(locked.error());
    unlocked = m.unlock();
    ASSERT_TRUE(unlocked.has_value()) << "err=" << ::m5::hal::v2::error::toString(unlocked.error());
}

#if defined(M5HAL_TEST_POSIX_MUTEX_FAULTS) && defined(__cpp_exceptions)
TEST(RuntimeMutex, MapsProviderSystemErrorToIoError)
{
    ::m5::variants::frameworks::posix::hal::v2::runtime::detail::failNextMutexLockWithSystemError();
    runtime::Mutex m;
    auto failed = m.lock(0);
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error(), ::m5::hal::v2::error::error_t::IO_ERROR);

    auto retried = m.lock(0);
    ASSERT_TRUE(retried.has_value()) << "err=" << ::m5::hal::v2::error::toString(retried.error());
    auto unlocked = m.unlock();
    ASSERT_TRUE(unlocked.has_value()) << "err=" << ::m5::hal::v2::error::toString(unlocked.error());
}
#endif

TEST(RuntimeMutex, ContendedLockTimesOutThenSucceedsAfterRelease)
{
    runtime::Mutex m;
    std::atomic<bool> held{false};
    std::atomic<bool> release{false};

    std::thread holder([&] {
        auto locked = m.lock(0);
        ASSERT_TRUE(locked.has_value()) << "err=" << ::m5::hal::v2::error::toString(locked.error());
        held = true;
        while (!release) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        auto unlocked = m.unlock();
        ASSERT_TRUE(unlocked.has_value()) << "err=" << ::m5::hal::v2::error::toString(unlocked.error());
    });
    while (!held) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Contention: the wait is real (lower bound only), then times out.
    const uint32_t t0 = runtime::millis();
    auto timed_out    = m.lock(50);
    ASSERT_FALSE(timed_out.has_value());
    EXPECT_EQ(timed_out.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_GE(runtime::millis() - t0, 40u);

    // The holder releases; a waiting lock with budget succeeds.
    release     = true;
    auto locked = m.lock(2000);
    ASSERT_TRUE(locked.has_value()) << "err=" << ::m5::hal::v2::error::toString(locked.error());
    auto unlocked = m.unlock();
    ASSERT_TRUE(unlocked.has_value()) << "err=" << ::m5::hal::v2::error::toString(unlocked.error());
    holder.join();
}

// ---- event (posix latching binary event) ------------------------------------

TEST(RuntimeEvent, NotifyBeforeWaitIsLatched)
{
    runtime::Event e;
    e.notify();
    auto consumed = e.wait(0);
    ASSERT_TRUE(consumed.has_value()) << "err=" << ::m5::hal::v2::error::toString(consumed.error());
    auto empty = e.wait(0);  // and consuming clears the latch
    ASSERT_FALSE(empty.has_value());
    EXPECT_EQ(empty.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
}

TEST(RuntimeEvent, MultipleNotifiesMergeIntoOne)
{
    runtime::Event e;
    e.notify();
    e.notify();
    e.notify();
    auto consumed = e.wait(0);
    ASSERT_TRUE(consumed.has_value()) << "err=" << ::m5::hal::v2::error::toString(consumed.error());
    auto empty = e.wait(0);  // merged: one consume drains them all
    ASSERT_FALSE(empty.has_value());
    EXPECT_EQ(empty.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
}

TEST(RuntimeEvent, WaitTimesOutWithoutNotify)
{
    runtime::Event e;
    const uint32_t t0 = runtime::millis();
    auto timed_out    = e.wait(30);
    ASSERT_FALSE(timed_out.has_value());
    EXPECT_EQ(timed_out.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    // The wait is real (lower bound only — CI machines stall).
    EXPECT_GE(runtime::millis() - t0, 25u);
}

TEST(RuntimeEvent, WaitWakesOnNotifyFromAnotherThread)
{
    runtime::Event e;
    std::thread notifier([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        e.notify();
    });
    auto woken = e.wait(2000);  // wakes through the notify, well inside the budget
    ASSERT_TRUE(woken.has_value()) << "err=" << ::m5::hal::v2::error::toString(woken.error());
    notifier.join();
    auto empty = e.wait(0);  // consumed by the wait above
    ASSERT_FALSE(empty.has_value());
    EXPECT_EQ(empty.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
}

TEST(RuntimeEvent, ForeverWaitWakesOnNotify)
{
    runtime::Event e;
    std::thread notifier([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        e.notify();
    });
    auto woken = e.wait(::m5::hal::v2::types::TIMEOUT_FOREVER);
    ASSERT_TRUE(woken.has_value()) << "err=" << ::m5::hal::v2::error::toString(woken.error());
    notifier.join();
}

#if defined(M5HAL_TEST_POSIX_EVENT_FAULTS) && defined(__cpp_exceptions)
TEST(RuntimeEvent, MapsProviderSystemErrorToIoError)
{
    runtime::Event e;
    ::m5::variants::frameworks::posix::hal::v2::runtime::detail::failNextEventWaitWithSystemError();
    auto failed = e.wait(0);
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error(), ::m5::hal::v2::error::error_t::IO_ERROR);

    e.notify();
    auto recovered = e.wait(0);
    ASSERT_TRUE(recovered.has_value()) << "err=" << ::m5::hal::v2::error::toString(recovered.error());
}
#endif

// ---- event (stub fake) -------------------------------------------------------

TEST(RuntimeStubEvent, WaitNeverBlocksButLatchWorks)
{
    stub_rt::Event e;
    // Single-task fake: with nobody around to notify, blocking could never
    // end — even TIMEOUT_FOREVER fails immediately (documented exception).
    auto finite = e.wait(10000);
    ASSERT_FALSE(finite.has_value());
    EXPECT_EQ(finite.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    auto forever = e.wait(::m5::hal::v2::types::TIMEOUT_FOREVER);
    ASSERT_FALSE(forever.has_value());
    EXPECT_EQ(forever.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    e.notify();
    e.notify();  // merges
    auto consumed = e.wait(0);
    ASSERT_TRUE(consumed.has_value()) << "err=" << ::m5::hal::v2::error::toString(consumed.error());
    auto empty = e.wait(0);
    ASSERT_FALSE(empty.has_value());
    EXPECT_EQ(empty.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
}

// ---- mutex (stub single-task guard) ----------------------------------------

TEST(RuntimeStubMutex, ContentionFailsImmediately)
{
    stub_rt::Mutex m;
    auto locked = m.lock(0);
    ASSERT_TRUE(locked.has_value()) << "err=" << ::m5::hal::v2::error::toString(locked.error());
    // Single-task fake: a timeout cannot resolve without another task,
    // so even a large budget fails immediately (deterministic).
    auto timed_out = m.lock(10000);
    ASSERT_FALSE(timed_out.has_value());
    EXPECT_EQ(timed_out.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    auto forever = m.lock(::m5::hal::v2::types::TIMEOUT_FOREVER);
    ASSERT_FALSE(forever.has_value());
    EXPECT_EQ(forever.error(), ::m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    auto unlocked = m.unlock();
    ASSERT_TRUE(unlocked.has_value()) << "err=" << ::m5::hal::v2::error::toString(unlocked.error());
    locked = m.lock(0);
    ASSERT_TRUE(locked.has_value()) << "err=" << ::m5::hal::v2::error::toString(locked.error());
    unlocked = m.unlock();
    ASSERT_TRUE(unlocked.has_value()) << "err=" << ::m5::hal::v2::error::toString(unlocked.error());
    auto invalid = m.unlock();
    ASSERT_FALSE(invalid.has_value());
    EXPECT_EQ(invalid.error(), ::m5::hal::v2::error::error_t::INVALID_STATE);
}

// ---- Bus integration (BREAKING: contention -> TIMEOUT_ERROR) ------------

struct TestBusConfig : public m5::hal::v2::bus::IBusConfig {
    TestBusConfig(void) : m5::hal::v2::bus::IBusConfig{m5::hal::v2::types::bus_kind_t::I2C}
    {
    }
};

struct TestBus : public m5::hal::v2::bus::IBus {
    const m5::hal::v2::bus::IBusConfig& getConfig(void) const override
    {
        return _cfg;
    }
    m5::hal::v2::result_t<void> acquire(m5::hal::v2::bus::IAccessor& owner, uint32_t timeout_ms)
    {
        return acquireAccessLock(owner, timeout_ms);
    }
    m5::hal::v2::result_t<void> release(m5::hal::v2::bus::IAccessor& owner)
    {
        return releaseAccessLock(owner);
    }
    TestBusConfig _cfg;
};

struct TestAccessConfig : public m5::hal::v2::bus::IAccessConfig {
    TestAccessConfig(void) : m5::hal::v2::bus::IAccessConfig{m5::hal::v2::types::bus_kind_t::I2C}
    {
    }
};

struct TestAccessor : public m5::hal::v2::bus::IAccessor {
    explicit TestAccessor(m5::hal::v2::bus::IBus& bus) : IAccessor{bus}
    {
    }
    const m5::hal::v2::bus::IAccessConfig& getConfig(void) const override
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

    ASSERT_TRUE(bus.acquire(a1, 0).has_value());
    auto r = bus.acquire(a2, 0);  // try-lock: fails immediately under contention
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    ASSERT_TRUE(bus.release(a1).has_value());
}

TEST(BusLock, ContendedLockWaitsForHolderAcrossTasks)
{
    TestBus bus;
    TestAccessor holder_acc{bus};
    TestAccessor waiter_acc{bus};
    std::atomic<bool> held{false};

    // The holder task takes the bus for ~150 ms; the waiter's budget
    // (2 s) covers it, so the waiter must succeed WITHOUT an error —
    // the waiting-lock semantics, impossible with the old
    // owner-pointer implementation.
    std::thread holder([&] {
        ASSERT_TRUE(bus.acquire(holder_acc, 0).has_value());
        held = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        ASSERT_TRUE(bus.release(holder_acc).has_value());
    });
    while (!held) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    const uint32_t t0 = runtime::millis();
    auto r            = bus.acquire(waiter_acc, 2000);
    ASSERT_TRUE(r.has_value());
    EXPECT_GE(runtime::millis() - t0, 50u);  // it really waited
    ASSERT_TRUE(bus.release(waiter_acc).has_value());
    holder.join();
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
