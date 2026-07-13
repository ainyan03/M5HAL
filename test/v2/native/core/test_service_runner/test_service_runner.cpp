// SPDX-License-Identifier: MIT
// Native gtest for service::ServiceRunner (hal/v2/service/service.hpp).
//
// test_basic covers the happy path (registration order) and the tick
// conversion helpers; this suite pins the registry semantics the
// cooperative runner promises its services: duplicate / over-capacity
// registration is rejected, removal compacts while preserving order,
// runOnce aggregates Progress|Done into its return value, and a
// service may remove itself (or a sibling) from inside its own
// serviceImpl() callback without skipping the next service in the pass
// (the bit-bang state machines end themselves exactly this way).

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace {

namespace service = ::m5::hal::v2::service;
using service::IService;
using service::ServiceContext;
using service::ServicePoll;
using service::ServiceResult;
using service::ServiceRunner;

// Appends its id to a shared log on every call and returns a scripted
// result. Optionally removes a service (itself by default) from the
// runner when it is called.
class ScriptedService : public IService {
public:
    ScriptedService(std::vector<int>& log, int id, ServiceResult result) : _log{&log}, _id{id}, _result{result}
    {
    }

    ServicePoll serviceImpl(const ServiceContext& ctx) override
    {
        _log->push_back(_id);
        last_elapsed = ctx.elapsed;
        if (remove_from != nullptr) {
            (void)remove_from->remove(remove_what != nullptr ? *remove_what : *this);
        }
        return ServicePoll{_result, next_due_delta};
    }

    ServiceRunner* remove_from          = nullptr;
    IService* remove_what               = nullptr;  // nullptr = remove self
    service::fast_tick_t last_elapsed   = 0;
    service::fast_tick_t next_due_delta = 0;

private:
    std::vector<int>* _log = nullptr;
    int _id                = 0;
    ServiceResult _result  = ServiceResult::Idle;
};

TEST(ServiceRunner, AddRejectsDuplicatesAndOverCapacity)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService first{log, 1, ServiceResult::Idle};

    EXPECT_TRUE(runner.add(first));
    EXPECT_FALSE(runner.add(first));  // duplicate registration
    EXPECT_EQ(runner.size(), 1u);

    std::vector<std::unique_ptr<ScriptedService>> fill;
    fill.reserve(ServiceRunner::kMaxServices);
    for (size_t i = 1; i < ServiceRunner::kMaxServices; ++i) {
        fill.emplace_back(new ScriptedService(log, static_cast<int>(100 + i), ServiceResult::Idle));
        EXPECT_TRUE(runner.add(*fill.back())) << i;
    }
    EXPECT_EQ(runner.size(), runner.capacity());

    ScriptedService overflow{log, 999, ServiceResult::Idle};
    EXPECT_FALSE(runner.add(overflow));  // table full
    EXPECT_EQ(runner.size(), ServiceRunner::kMaxServices);
}

TEST(ServiceRunner, RemoveCompactsAndPreservesOrder)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService a{log, 1, ServiceResult::Idle};
    ScriptedService b{log, 2, ServiceResult::Idle};
    ScriptedService c{log, 3, ServiceResult::Idle};
    ASSERT_TRUE(runner.add(a));
    ASSERT_TRUE(runner.add(b));
    ASSERT_TRUE(runner.add(c));

    EXPECT_TRUE(runner.remove(b));
    EXPECT_FALSE(runner.remove(b));  // not registered (anymore)
    EXPECT_EQ(runner.size(), 2u);

    (void)runner.runOnce(ServiceContext{0, 0});
    EXPECT_EQ(log, (std::vector<int>{1, 3}));  // order survives the compaction
}

TEST(ServiceRunner, ClearEmptiesTheTableAndAllowsReuse)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService a{log, 1, ServiceResult::Progress};
    ASSERT_TRUE(runner.add(a));

    runner.clear();
    EXPECT_EQ(runner.size(), 0u);
    EXPECT_FALSE(runner.runOnce(ServiceContext{0, 0}));  // nothing to run
    EXPECT_TRUE(log.empty());

    EXPECT_TRUE(runner.add(a));  // re-registration after clear works
    EXPECT_TRUE(runner.runOnce(ServiceContext{0, 0}));
}

TEST(ServiceRunner, RunOnceAggregatesProgressAndDone)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService idle{log, 1, ServiceResult::Idle};
    ScriptedService error{log, 2, ServiceResult::Error};
    ASSERT_TRUE(runner.add(idle));
    ASSERT_TRUE(runner.add(error));
    EXPECT_FALSE(runner.runOnce(ServiceContext{10, 0}));  // Idle/Error: no progress

    ScriptedService progress{log, 3, ServiceResult::Progress};
    ASSERT_TRUE(runner.add(progress));
    EXPECT_TRUE(runner.runOnce(ServiceContext{10, 0}));

    // Per-service elapsed: idle was last polled one 10-tick pass ago;
    // progress was added between the passes, so its first poll reports the
    // advance since its add (the pre-add past is nobody's to vouch for).
    EXPECT_EQ(idle.last_elapsed, 10u);
    EXPECT_EQ(progress.last_elapsed, 10u);
}

TEST(ServiceRunner, SkipsServiceUntilNextDue)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService timed{log, 1, ServiceResult::Idle};
    ScriptedService always{log, 2, ServiceResult::Progress};
    timed.next_due_delta = 100;
    ASSERT_TRUE(runner.add(timed));
    ASSERT_TRUE(runner.add(always));

    // Pass 1 (elapsed 10): both run; timed asks to sleep 100 ticks.
    EXPECT_TRUE(runner.runOnce(ServiceContext{10, 0}));
    EXPECT_EQ(log, (std::vector<int>{1, 2}));

    // Pass 2 (elapsed 89, 99 ticks total since timed's poll): still short.
    log.clear();
    EXPECT_TRUE(runner.runOnce(ServiceContext{89, 0}));
    EXPECT_EQ(log, (std::vector<int>{2}));

    // Pass 3 (elapsed 11): the hint expires. timed's ctx.elapsed covers the
    // WHOLE span since its previous poll, skipped passes included.
    log.clear();
    timed.next_due_delta = 0;
    EXPECT_TRUE(runner.runOnce(ServiceContext{11, 0}));
    EXPECT_EQ(log, (std::vector<int>{1, 2}));
    EXPECT_EQ(timed.last_elapsed, 100u);
    EXPECT_EQ(always.last_elapsed, 11u);
}

TEST(ServiceRunner, ServiceMayRemoveItselfWhileRunning)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService a{log, 1, ServiceResult::Done};
    ScriptedService b{log, 2, ServiceResult::Idle};
    a.remove_from = &runner;
    ASSERT_TRUE(runner.add(a));
    ASSERT_TRUE(runner.add(b));

    // Self-removal compacts the table mid-pass; the cursor compensates,
    // so b (shifted into a's slot) still runs in the SAME pass.
    EXPECT_TRUE(runner.runOnce(ServiceContext{0, 0}));
    EXPECT_EQ(runner.size(), 1u);
    EXPECT_EQ(log, (std::vector<int>{1, 2}));

    log.clear();
    (void)runner.runOnce(ServiceContext{1, 0});
    EXPECT_EQ(log, (std::vector<int>{2}));  // a is gone, b still runs
}

TEST(ServiceRunner, ServiceMayRemoveASiblingWhileRunning)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService a{log, 1, ServiceResult::Idle};
    ScriptedService b{log, 2, ServiceResult::Idle};
    ScriptedService c{log, 3, ServiceResult::Idle};
    a.remove_from = &runner;
    a.remove_what = &b;
    ASSERT_TRUE(runner.add(a));
    ASSERT_TRUE(runner.add(b));
    ASSERT_TRUE(runner.add(c));

    // a removes b before b's turn: b must not run, c must not be skipped.
    (void)runner.runOnce(ServiceContext{0, 0});
    EXPECT_EQ(log, (std::vector<int>{1, 3}));
    EXPECT_EQ(runner.size(), 2u);
}

// The posix runtime now runs auto-run on a real background thread (see
// service.inl's runtime::Task guard), so ServiceRunner's add/remove/
// startAutoRun concurrency contract is exercisable -- and TSan-checkable --
// from a native gtest instead of only on-target. The fixture's TearDown()
// runs even when an ASSERT_* inside the test body returns early, so the
// background task never outlives its test.
class ServiceRunnerAutoRunTest : public ::testing::Test {
protected:
    void TearDown() override
    {
        runner_.stopAutoRun();
        runner_.clear();
    }

    ServiceRunner runner_;
};

// Logs its id under a mutex on every poll (the auto-run task and the test
// thread both touch the log) and always reports Progress so the runner
// keeps re-polling it every pass instead of idling out.
class LoggingService : public IService {
public:
    LoggingService(std::mutex& log_mutex, std::vector<int>& log, int id) : _log_mutex{&log_mutex}, _log{&log}, _id{id}
    {
    }

    ServicePoll serviceImpl(const ServiceContext&) override
    {
        std::lock_guard<std::mutex> lock{*_log_mutex};
        _log->push_back(_id);
        return ServicePoll{ServiceResult::Progress};
    }

private:
    std::mutex* _log_mutex;
    std::vector<int>* _log;
    int _id;
};

TEST_F(ServiceRunnerAutoRunTest, StartAutoRunFromTwoThreadsIsIdempotent)
{
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    bool ok[2] = {false, false};

    auto start_it = [&](int i) {
        ready.fetch_add(1, std::memory_order_acq_rel);
        while (!go.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        ok[i] = runner_.startAutoRun();
    };

    std::thread t0{start_it, 0};
    std::thread t1{start_it, 1};
    while (ready.load(std::memory_order_acquire) < 2) {
        std::this_thread::yield();
    }
    go.store(true, std::memory_order_release);
    t0.join();
    t1.join();

    // Both callers observe success (startAutoRun is idempotent, R2): the
    // loser of the race must see the winner's already-running task and
    // return true, not fail or spawn a second task. If applyAdd/_auto_task
    // were ever written outside _control's protection, TSan would flag the
    // race here.
    EXPECT_TRUE(ok[0]);
    EXPECT_TRUE(ok[1]);
    EXPECT_TRUE(runner_.autoRunActive());
}

TEST_F(ServiceRunnerAutoRunTest, ConcurrentAddRemoveWhileAutoRunning)
{
    std::mutex log_mutex;
    std::vector<int> log;
    LoggingService anchor{log_mutex, log, 0};
    ASSERT_TRUE(runner_.add(anchor));
    ASSERT_TRUE(runner_.startAutoRun());

    // Bounded poll instead of a fixed sleep or an unbounded spin: wait until
    // the auto-run task has actually observed `id` in the log, up to a
    // generous timeout.
    auto wait_polled = [&](int id) {
        for (int attempt = 0; attempt < 2000; ++attempt) {
            {
                std::lock_guard<std::mutex> lock{log_mutex};
                if (std::find(log.begin(), log.end(), id) != log.end()) {
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    };

    constexpr int kRounds = 30;
    std::vector<long> count_at_removal(static_cast<size_t>(kRounds) + 1, -1);
    std::thread churner{[&]() {
        for (int i = 1; i <= kRounds; ++i) {
            LoggingService svc{log_mutex, log, i};
            EXPECT_TRUE(runner_.add(svc));
            EXPECT_TRUE(wait_polled(i)) << "id=" << i << " never observed by the auto-run task";
            EXPECT_TRUE(runner_.remove(svc));
            // remove() is SYNCHRONOUS: once it returns, svc.serviceImpl() must
            // never run again (R3), so it is safe to destroy svc right after
            // this iteration. Snapshot its log count now to check that later.
            std::lock_guard<std::mutex> lock{log_mutex};
            count_at_removal[static_cast<size_t>(i)] = std::count(log.begin(), log.end(), i);
        }
    }};
    churner.join();

    // Stop the auto-run task BEFORE returning from the test body: `anchor`,
    // `log_mutex` and `log` are locals of this function and unwind when it
    // returns, which happens BEFORE the fixture's TearDown() runs. Without
    // this, the background task can still be mid-poll on `anchor` (a
    // dangling reference once the stack unwinds) after the checks below --
    // observed as a "mutex lock failed: Invalid argument" abort from
    // log_mutex's storage having already been reclaimed.
    runner_.stopAutoRun();

    // Verify "returned => never polled again": no id's log count grew past
    // the snapshot taken right after its remove() returned.
    {
        std::lock_guard<std::mutex> lock{log_mutex};
        for (int i = 1; i <= kRounds; ++i) {
            EXPECT_EQ(std::count(log.begin(), log.end(), i), count_at_removal[static_cast<size_t>(i)]) << "id=" << i;
        }
        EXPECT_FALSE(log.empty());  // the anchor kept progressing throughout
    }
}

// ---- wake-event behavior (notified wake-up out of the idle wait) ------------
//
// The auto-run task blocks on a latching wake event while the table is
// empty (instead of the former tick-quantized delayMs(1) sleep) and is
// notified by add()/remove()/stopAutoRun(). These tests measure the
// wake latency directly with a threshold WELL BELOW the 100 ms
// liveness backstop: a missed notification would only wake through the
// backstop, so a loose "eventually polled" check would hide exactly
// the bug this design must not have.

// Watches for the moment its serviceImpl() first runs.
class FirstPollLatchService : public IService {
public:
    ServicePoll serviceImpl(const ServiceContext&) override
    {
        polled.store(true, std::memory_order_release);
        return ServicePoll{ServiceResult::Progress};
    }

    std::atomic<bool> polled{false};
};

TEST_F(ServiceRunnerAutoRunTest, IdleRunnerWakesPromptlyOnAdd)
{
    ASSERT_TRUE(runner_.startAutoRun());  // empty table: the runner idles

    // Repeat the empty->add round trip: each round exercises the
    // latch window (a notify racing the runner's count check / wait
    // entry) at a different phase of the runner's loop.
    for (int round = 0; round < 10; ++round) {
        // Let the runner drain back to its idle wait after the previous
        // round's remove.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        FirstPollLatchService svc;
        const auto t0 = std::chrono::steady_clock::now();
        ASSERT_TRUE(runner_.add(svc));
        while (!svc.polled.load(std::memory_order_acquire)) {
            ASSERT_LT(std::chrono::steady_clock::now() - t0, std::chrono::seconds(5)) << "round " << round;
            std::this_thread::yield();
        }
        const auto elapsed = std::chrono::steady_clock::now() - t0;
        ASSERT_TRUE(runner_.remove(svc));

        // Must beat the 100 ms backstop by a wide margin: a notified
        // wake is scheduler-latency fast, while a missed notification
        // sleeps the full backstop out.
        EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 50) << "round " << round;
    }
}

TEST_F(ServiceRunnerAutoRunTest, StopWhileIdleReturnsPromptly)
{
    ASSERT_TRUE(runner_.startAutoRun());
    // Give the runner time to enter the idle wait, so the stop below
    // exercises the notified wake (not the pre-wait exit check).
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    const auto t0 = std::chrono::steady_clock::now();
    runner_.stopAutoRun();
    const auto elapsed = std::chrono::steady_clock::now() - t0;

    EXPECT_FALSE(runner_.autoRunActive());
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 50);
}

TEST_F(ServiceRunnerAutoRunTest, RestartAfterStopKeepsServicesAndWakesOnAdd)
{
    // Stop while the runner is busy (non-empty table, yield path): the
    // stop notify can land OUTSIDE a wait and stay latched. The restart
    // must shrug that stale latch off (one spurious idle pass at most)
    // and both the surviving service and a fresh add must be polled.
    std::mutex log_mutex;
    std::vector<int> log;
    LoggingService survivor{log_mutex, log, 1};
    ASSERT_TRUE(runner_.add(survivor));
    ASSERT_TRUE(runner_.startAutoRun());

    auto count_of = [&](int id) {
        std::lock_guard<std::mutex> lock{log_mutex};
        return std::count(log.begin(), log.end(), id);
    };
    while (count_of(1) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    runner_.stopAutoRun();
    const auto stopped_count = count_of(1);

    ASSERT_TRUE(runner_.startAutoRun());
    while (count_of(1) <= stopped_count) {  // the survivor keeps being polled
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    LoggingService fresh{log_mutex, log, 2};
    ASSERT_TRUE(runner_.add(fresh));
    while (count_of(2) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // Stop before the locals unwind (see ConcurrentAddRemoveWhileAutoRunning).
    runner_.stopAutoRun();
}

TEST(ServiceRunnerWake, DestructorJoinsAnIdleWaitingRunnerPromptly)
{
    const auto t0 = std::chrono::steady_clock::now();
    {
        ServiceRunner runner;
        ASSERT_TRUE(runner.startAutoRun());
        // Let the task settle into the idle wait before the dtor stops it.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    // 20 ms settle + a notified wake; only a missed notification would
    // push this toward the 100 ms backstop.
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 80);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
