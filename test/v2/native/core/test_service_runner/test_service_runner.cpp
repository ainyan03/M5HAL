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
namespace error   = ::m5::hal::v2::error;
using service::IService;
using service::ServiceContext;
using service::ServicePoll;
using service::ServiceResult;
using service::ServiceRunner;

#define ASSERT_RESULT_OK(expression)                                                    \
    do {                                                                                \
        const auto result_ = (expression);                                              \
        ASSERT_TRUE(result_.has_value()) << "err=" << error::toString(result_.error()); \
    } while (false)

#define EXPECT_RESULT_OK(expression)                                                    \
    do {                                                                                \
        const auto result_ = (expression);                                              \
        EXPECT_TRUE(result_.has_value()) << "err=" << error::toString(result_.error()); \
    } while (false)

#define EXPECT_RESULT_ERROR(expression, expected_error) \
    do {                                                \
        const auto result_ = (expression);              \
        ASSERT_FALSE(result_.has_value());              \
        EXPECT_EQ(result_.error(), (expected_error));   \
    } while (false)

#define ASSERT_RUN_RESULT(expression, expected_progress)                                \
    do {                                                                                \
        const auto result_ = (expression);                                              \
        ASSERT_TRUE(result_.has_value()) << "err=" << error::toString(result_.error()); \
        EXPECT_EQ(result_.value(), (expected_progress));                                \
    } while (false)

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
            const auto removed = remove_from->remove(remove_what != nullptr ? *remove_what : *this);
            remove_succeeded   = removed.has_value();
            if (!removed.has_value()) {
                remove_error = removed.error();
            }
        }
        return ServicePoll{_result, next_due_delta};
    }

    ServiceRunner* remove_from          = nullptr;
    IService* remove_what               = nullptr;  // nullptr = remove self
    bool remove_succeeded               = false;
    error::error_t remove_error         = error::error_t::OK;
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

    ASSERT_RESULT_OK(runner.add(first));
    EXPECT_RESULT_ERROR(runner.add(first), error::error_t::INVALID_STATE);
    EXPECT_EQ(runner.size(), 1u);

    std::vector<std::unique_ptr<ScriptedService>> fill;
    fill.reserve(ServiceRunner::kMaxServices);
    for (size_t i = 1; i < ServiceRunner::kMaxServices; ++i) {
        fill.emplace_back(new ScriptedService(log, static_cast<int>(100 + i), ServiceResult::Idle));
        const auto added = runner.add(*fill.back());
        ASSERT_TRUE(added.has_value()) << "index=" << i << " err=" << error::toString(added.error());
    }
    EXPECT_EQ(runner.size(), runner.capacity());

    ScriptedService overflow{log, 999, ServiceResult::Idle};
    EXPECT_RESULT_ERROR(runner.add(overflow), error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(runner.size(), ServiceRunner::kMaxServices);
}

#if defined(M5HAL_TEST_POSIX_MUTEX_FAULTS) && defined(__cpp_exceptions)
TEST(ServiceRunner, UnlockFailureBreaksControlPlaneAndRejectsReuse)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService first{log, 1, ServiceResult::Idle};
    ScriptedService second{log, 2, ServiceResult::Idle};

    ::m5::variants::frameworks::posix::hal::v2::runtime::detail::failNextMutexUnlockWithIoError();
    auto failed = runner.add(first);
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error(), error::error_t::IO_ERROR);
    EXPECT_EQ(runner.size(), 1u);

    EXPECT_RESULT_ERROR(runner.add(second), error::error_t::INVALID_STATE);
    EXPECT_EQ(runner.size(), 1u);
}

TEST(ServiceRunner, PrimaryCommandErrorWinsWhileUnlockFailureStillBreaksControlPlane)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService first{log, 1, ServiceResult::Idle};
    ScriptedService second{log, 2, ServiceResult::Idle};
    ASSERT_TRUE(runner.add(first).has_value());

    ::m5::variants::frameworks::posix::hal::v2::runtime::detail::failNextMutexUnlockWithIoError();
    EXPECT_RESULT_ERROR(runner.add(first), error::error_t::INVALID_STATE);
    EXPECT_RESULT_ERROR(runner.add(second), error::error_t::INVALID_STATE);
    EXPECT_EQ(runner.size(), 1u);
}
#endif

TEST(ServiceRunner, RemoveCompactsAndPreservesOrder)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService a{log, 1, ServiceResult::Idle};
    ScriptedService b{log, 2, ServiceResult::Idle};
    ScriptedService c{log, 3, ServiceResult::Idle};
    ASSERT_RESULT_OK(runner.add(a));
    ASSERT_RESULT_OK(runner.add(b));
    ASSERT_RESULT_OK(runner.add(c));

    EXPECT_RESULT_OK(runner.remove(b));
    EXPECT_RESULT_OK(runner.remove(b));  // idempotent no-op
    EXPECT_EQ(runner.size(), 2u);

    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{0, 0}), false);
    EXPECT_EQ(log, (std::vector<int>{1, 3}));  // order survives the compaction
}

TEST(ServiceRunner, ClearEmptiesTheTableAndAllowsReuse)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService a{log, 1, ServiceResult::Progress};
    ASSERT_RESULT_OK(runner.add(a));

    EXPECT_RESULT_OK(runner.clear());
    EXPECT_RESULT_OK(runner.clear());  // empty clear is an idempotent no-op
    EXPECT_EQ(runner.size(), 0u);
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{0, 0}), false);  // nothing to run
    EXPECT_TRUE(log.empty());

    ASSERT_RESULT_OK(runner.add(a));  // re-registration after clear works
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{0, 0}), true);
}

TEST(ServiceRunner, RunOnceAggregatesProgressAndDone)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService idle{log, 1, ServiceResult::Idle};
    ScriptedService error_service{log, 2, ServiceResult::Error};
    ASSERT_RESULT_OK(runner.add(idle));
    ASSERT_RESULT_OK(runner.add(error_service));
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{10, 0}), false);  // Idle/Error: no progress

    ScriptedService progress{log, 3, ServiceResult::Progress};
    ASSERT_RESULT_OK(runner.add(progress));
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{10, 0}), true);

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
    ASSERT_RESULT_OK(runner.add(timed));
    ASSERT_RESULT_OK(runner.add(always));

    // Pass 1 (elapsed 10): both run; timed asks to sleep 100 ticks.
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{10, 0}), true);
    EXPECT_EQ(log, (std::vector<int>{1, 2}));

    // Pass 2 (elapsed 89, 99 ticks total since timed's poll): still short.
    log.clear();
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{89, 0}), true);
    EXPECT_EQ(log, (std::vector<int>{2}));

    // Pass 3 (elapsed 11): the hint expires. timed's ctx.elapsed covers the
    // WHOLE span since its previous poll, skipped passes included.
    log.clear();
    timed.next_due_delta = 0;
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{11, 0}), true);
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
    ASSERT_RESULT_OK(runner.add(a));
    ASSERT_RESULT_OK(runner.add(b));

    // Self-removal compacts the table mid-pass; the cursor compensates,
    // so b (shifted into a's slot) still runs in the SAME pass.
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{0, 0}), true);
    EXPECT_TRUE(a.remove_succeeded) << "err=" << error::toString(a.remove_error);
    EXPECT_EQ(runner.size(), 1u);
    EXPECT_EQ(log, (std::vector<int>{1, 2}));

    log.clear();
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{1, 0}), false);
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
    ASSERT_RESULT_OK(runner.add(a));
    ASSERT_RESULT_OK(runner.add(b));
    ASSERT_RESULT_OK(runner.add(c));

    // a removes b before b's turn: b must not run, c must not be skipped.
    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{0, 0}), false);
    EXPECT_TRUE(a.remove_succeeded) << "err=" << error::toString(a.remove_error);
    EXPECT_EQ(log, (std::vector<int>{1, 3}));
    EXPECT_EQ(runner.size(), 2u);
}

class AddFromCallbackService : public IService {
public:
    AddFromCallbackService(ServiceRunner& runner, IService& target, std::vector<int>& log)
        : _runner{&runner}, _target{&target}, _log{&log}
    {
    }

    ServicePoll serviceImpl(const ServiceContext&) override
    {
        _log->push_back(1);
        if (!attempted) {
            attempted        = true;
            const auto added = _runner->add(*_target);
            succeeded        = added.has_value();
            if (!added.has_value()) {
                failure = added.error();
            }
        }
        return ServicePoll{ServiceResult::Idle};
    }

    bool attempted         = false;
    bool succeeded         = false;
    error::error_t failure = error::error_t::OK;

private:
    ServiceRunner* _runner = nullptr;
    IService* _target      = nullptr;
    std::vector<int>* _log = nullptr;
};

TEST(ServiceRunner, ServiceMayAddAnotherServiceWhileRunning)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService added{log, 2, ServiceResult::Progress};
    AddFromCallbackService adding{runner, added, log};
    ASSERT_RESULT_OK(runner.add(adding));

    ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{0, 0}), true);
    EXPECT_TRUE(adding.succeeded) << "err=" << error::toString(adding.failure);
    EXPECT_EQ(log, (std::vector<int>{1, 2}));
    EXPECT_EQ(runner.size(), 2u);
}

enum class CallbackControlAction : uint8_t {
    RunOnce,
    Clear,
    StartAutoRun,
    StopAutoRun,
};

class CallbackControlService : public IService {
public:
    CallbackControlService(ServiceRunner& runner, CallbackControlAction action) : _runner{&runner}, _action{action}
    {
    }

    ServicePoll serviceImpl(const ServiceContext&) override
    {
        called = true;
        if (_action == CallbackControlAction::RunOnce) {
            const auto result = _runner->runOnce(ServiceContext{0, 0});
            succeeded         = result.has_value();
            if (result.has_value()) {
                progress = result.value();
            } else {
                failure = result.error();
            }
        } else if (_action == CallbackControlAction::Clear) {
            const auto result = _runner->clear();
            succeeded         = result.has_value();
            if (!result.has_value()) {
                failure = result.error();
            }
        } else if (_action == CallbackControlAction::StartAutoRun) {
            const auto result = _runner->startAutoRun();
            succeeded         = result.has_value();
            if (!result.has_value()) {
                failure = result.error();
            }
        } else {
            const auto result = _runner->stopAutoRun();
            succeeded         = result.has_value();
            if (!result.has_value()) {
                failure = result.error();
            }
        }
        return ServicePoll{ServiceResult::Idle};
    }

    bool called            = false;
    bool succeeded         = false;
    bool progress          = false;
    error::error_t failure = error::error_t::OK;

private:
    ServiceRunner* _runner = nullptr;
    CallbackControlAction _action;
};

TEST(ServiceRunner, CallbackControlOperationsFailWithoutChangingState)
{
    const auto verify = [](CallbackControlAction action, error::error_t expected_error) {
        ServiceRunner runner;
        CallbackControlService service{runner, action};
        ASSERT_RESULT_OK(runner.add(service));

        ASSERT_RUN_RESULT(runner.runOnce(ServiceContext{0, 0}), false);
        ASSERT_TRUE(service.called);
        EXPECT_FALSE(service.succeeded);
        EXPECT_FALSE(service.progress);
        EXPECT_EQ(service.failure, expected_error);
        EXPECT_EQ(runner.size(), 1u);
        EXPECT_FALSE(runner.autoRunActive());
    };

    verify(CallbackControlAction::RunOnce, error::error_t::BUSY);
    verify(CallbackControlAction::Clear, error::error_t::INVALID_STATE);
    verify(CallbackControlAction::StartAutoRun, error::error_t::INVALID_STATE);
    verify(CallbackControlAction::StopAutoRun, error::error_t::INVALID_STATE);
}

class BlockingService : public IService {
public:
    ServicePoll serviceImpl(const ServiceContext&) override
    {
        entered.store(true, std::memory_order_release);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!release.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::yield();
        }
        return ServicePoll{ServiceResult::Idle};
    }

    std::atomic<bool> entered{false};
    std::atomic<bool> release{false};
};

TEST(ServiceRunner, ConcurrentRunOnceReturnsBusyWithoutPolling)
{
    ServiceRunner runner;
    BlockingService service;
    ASSERT_RESULT_OK(runner.add(service));

    bool owner_succeeded         = false;
    bool owner_progress          = false;
    error::error_t owner_failure = error::error_t::OK;
    std::thread owner{[&]() {
        const auto result = runner.runOnce(ServiceContext{0, 0});
        owner_succeeded   = result.has_value();
        if (result.has_value()) {
            owner_progress = result.value();
        } else {
            owner_failure = result.error();
        }
    }};

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!service.entered.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    if (!service.entered.load(std::memory_order_acquire)) {
        service.release.store(true, std::memory_order_release);
        owner.join();
        FAIL() << "owner runOnce did not enter the service callback";
    }

    const auto contended = runner.runOnce(ServiceContext{0, 0});
    service.release.store(true, std::memory_order_release);
    owner.join();

    ASSERT_FALSE(contended.has_value());
    EXPECT_EQ(contended.error(), error::error_t::BUSY);
    EXPECT_EQ(runner.size(), 1u);
    ASSERT_TRUE(owner_succeeded) << "err=" << error::toString(owner_failure);
    EXPECT_FALSE(owner_progress);
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
        (void)runner_.stopAutoRun();
        (void)runner_.clear();
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

class CountingService : public IService {
public:
    ServicePoll serviceImpl(const ServiceContext&) override
    {
        polls.fetch_add(1, std::memory_order_relaxed);
        return ServicePoll{ServiceResult::Progress};
    }

    std::atomic<uint64_t> polls{0};
};

#if defined(M5HAL_TEST_POSIX_EVENT_FAULTS) && defined(__cpp_exceptions)
TEST_F(ServiceRunnerAutoRunTest, WakeBackendErrorBacksOffAndRunnerRemainsUsable)
{
    CountingService busy;
    ASSERT_RESULT_OK(runner_.add(busy));
    ASSERT_RESULT_OK(runner_.startAutoRun());

    const auto first_poll_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (busy.polls.load(std::memory_order_acquire) == 0 && std::chrono::steady_clock::now() < first_poll_deadline) {
        std::this_thread::yield();
    }
    ASSERT_NE(busy.polls.load(std::memory_order_acquire), 0u);

    ::m5::variants::frameworks::posix::hal::v2::runtime::detail::failNextEventWaitWithSystemError();
    ASSERT_RESULT_OK(runner_.remove(busy));
    const auto fault_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (::m5::variants::frameworks::posix::hal::v2::runtime::detail::eventWaitSystemErrorPending() &&
           std::chrono::steady_clock::now() < fault_deadline) {
        std::this_thread::yield();
    }
    ASSERT_FALSE(::m5::variants::frameworks::posix::hal::v2::runtime::detail::eventWaitSystemErrorPending());
    EXPECT_TRUE(runner_.autoRunActive());

    CountingService after_error;
    ASSERT_RESULT_OK(runner_.add(after_error));
    const auto resumed_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (after_error.polls.load(std::memory_order_acquire) == 0 &&
           std::chrono::steady_clock::now() < resumed_deadline) {
        std::this_thread::yield();
    }
    EXPECT_NE(after_error.polls.load(std::memory_order_acquire), 0u);
    EXPECT_RESULT_OK(runner_.remove(after_error));
}
#endif

TEST_F(ServiceRunnerAutoRunTest, StartAutoRunFromTwoThreadsIsIdempotent)
{
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    bool ok[2]                 = {false, false};
    error::error_t failures[2] = {error::error_t::OK, error::error_t::OK};

    auto start_it = [&](int i) {
        ready.fetch_add(1, std::memory_order_acq_rel);
        while (!go.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        const auto started = runner_.startAutoRun();
        ok[i]              = started.has_value();
        if (!started.has_value()) {
            failures[i] = started.error();
        }
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
    EXPECT_TRUE(ok[0]) << "err=" << error::toString(failures[0]);
    EXPECT_TRUE(ok[1]) << "err=" << error::toString(failures[1]);
    EXPECT_TRUE(runner_.autoRunActive());
    EXPECT_RESULT_OK(runner_.startAutoRun());  // already running is success
}

class AutoStartFromCallbackService final : public IService {
public:
    explicit AutoStartFromCallbackService(ServiceRunner& runner) : _runner{&runner}
    {
    }

    ServicePoll serviceImpl(const ServiceContext&) override
    {
        const auto started = _runner->startAutoRun();
        succeeded          = started.has_value();
        if (!started.has_value()) {
            failure = started.error();
        }
        called.store(true, std::memory_order_release);
        return ServicePoll{ServiceResult::Idle};
    }

    std::atomic<bool> called{false};
    bool succeeded         = false;
    error::error_t failure = error::error_t::OK;

private:
    ServiceRunner* _runner = nullptr;
};

TEST_F(ServiceRunnerAutoRunTest, FirstAutoRunCallbackSeesStartAsIdempotentSuccess)
{
    AutoStartFromCallbackService service{runner_};
    ASSERT_RESULT_OK(runner_.add(service));
    ASSERT_RESULT_OK(runner_.startAutoRun());

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!service.called.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    const bool was_called = service.called.load(std::memory_order_acquire);
    EXPECT_RESULT_OK(runner_.stopAutoRun());

    ASSERT_TRUE(was_called);
    EXPECT_TRUE(service.succeeded) << "err=" << error::toString(service.failure);
}

TEST_F(ServiceRunnerAutoRunTest, ConcurrentAddRemoveWhileAutoRunning)
{
    std::mutex log_mutex;
    std::vector<int> log;
    LoggingService anchor{log_mutex, log, 0};
    ASSERT_RESULT_OK(runner_.add(anchor));
    ASSERT_RESULT_OK(runner_.startAutoRun());

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
            EXPECT_RESULT_OK(runner_.add(svc));
            EXPECT_TRUE(wait_polled(i)) << "id=" << i << " never observed by the auto-run task";
            EXPECT_RESULT_OK(runner_.remove(svc));
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
    EXPECT_RESULT_OK(runner_.stopAutoRun());
    EXPECT_RESULT_OK(runner_.stopAutoRun());  // already stopped is success

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

TEST_F(ServiceRunnerAutoRunTest, ConcurrentDistinctAddsReportOnlyTableCapacity)
{
    ASSERT_RESULT_OK(runner_.startAutoRun());

    constexpr size_t kCallers = ServiceRunner::kMaxServices + 1;
    std::vector<std::unique_ptr<CountingService>> services;
    services.reserve(kCallers);
    for (size_t i = 0; i < kCallers; ++i) {
        services.emplace_back(new CountingService{});
    }

    std::atomic<size_t> ready{0};
    std::atomic<bool> go{false};
    std::vector<uint8_t> succeeded(kCallers, 0);
    std::vector<error::error_t> failures(kCallers, error::error_t::OK);
    std::vector<std::thread> callers;
    callers.reserve(kCallers);
    for (size_t i = 0; i < kCallers; ++i) {
        callers.emplace_back([&, i]() {
            ready.fetch_add(1, std::memory_order_acq_rel);
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            const auto added = runner_.add(*services[i]);
            succeeded[i]     = static_cast<uint8_t>(added.has_value());
            if (!added.has_value()) {
                failures[i] = added.error();
            }
        });
    }
    while (ready.load(std::memory_order_acquire) != kCallers) {
        std::this_thread::yield();
    }
    go.store(true, std::memory_order_release);
    for (auto& caller : callers) {
        caller.join();
    }

    size_t success_count = 0;
    size_t full_count    = 0;
    for (size_t i = 0; i < kCallers; ++i) {
        success_count += succeeded[i];
        if (!succeeded[i]) {
            EXPECT_EQ(failures[i], error::error_t::OUT_OF_RESOURCE) << "caller=" << i;
            full_count += failures[i] == error::error_t::OUT_OF_RESOURCE;
        }
    }
    EXPECT_EQ(success_count, ServiceRunner::kMaxServices);
    EXPECT_EQ(full_count, 1u);
    EXPECT_EQ(runner_.size(), ServiceRunner::kMaxServices);

    EXPECT_RESULT_OK(runner_.stopAutoRun());
    EXPECT_RESULT_OK(runner_.clear());
}

TEST_F(ServiceRunnerAutoRunTest, ConcurrentDistinctAddsAndRemovesAreLinearized)
{
    constexpr size_t kPairs = 6;
    CountingService anchor;
    std::vector<std::unique_ptr<CountingService>> old_services;
    std::vector<std::unique_ptr<CountingService>> replacements;
    old_services.reserve(kPairs);
    replacements.reserve(kPairs);

    ASSERT_RESULT_OK(runner_.add(anchor));
    for (size_t i = 0; i < kPairs; ++i) {
        old_services.emplace_back(new CountingService{});
        replacements.emplace_back(new CountingService{});
        ASSERT_RESULT_OK(runner_.add(*old_services.back()));
    }
    ASSERT_RESULT_OK(runner_.startAutoRun());

    const auto initial_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    for (;;) {
        const bool all_polled = std::all_of(old_services.begin(), old_services.end(), [](const auto& service) {
            return service->polls.load(std::memory_order_relaxed) != 0;
        });
        if (all_polled) {
            break;
        }
        if (std::chrono::steady_clock::now() >= initial_deadline) {
            EXPECT_RESULT_OK(runner_.stopAutoRun());
            EXPECT_RESULT_OK(runner_.clear());
            FAIL() << "initial services were not all polled";
        }
        std::this_thread::yield();
    }

    constexpr size_t kCallers = kPairs * 2;
    std::atomic<size_t> ready{0};
    std::atomic<bool> go{false};
    std::vector<uint8_t> succeeded(kCallers, 0);
    std::vector<error::error_t> failures(kCallers, error::error_t::OK);
    std::vector<std::thread> callers;
    callers.reserve(kCallers);
    for (size_t i = 0; i < kPairs; ++i) {
        callers.emplace_back([&, i]() {
            ready.fetch_add(1, std::memory_order_acq_rel);
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            const auto removed = runner_.remove(*old_services[i]);
            succeeded[i]       = static_cast<uint8_t>(removed.has_value());
            if (!removed.has_value()) {
                failures[i] = removed.error();
            }
        });
        callers.emplace_back([&, i]() {
            const size_t result_index = kPairs + i;
            ready.fetch_add(1, std::memory_order_acq_rel);
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            const auto added        = runner_.add(*replacements[i]);
            succeeded[result_index] = static_cast<uint8_t>(added.has_value());
            if (!added.has_value()) {
                failures[result_index] = added.error();
            }
        });
    }
    while (ready.load(std::memory_order_acquire) != kCallers) {
        std::this_thread::yield();
    }
    go.store(true, std::memory_order_release);
    for (auto& caller : callers) {
        caller.join();
    }
    for (size_t i = 0; i < kCallers; ++i) {
        EXPECT_TRUE(succeeded[i]) << "caller=" << i << " err=" << error::toString(failures[i]);
    }
    EXPECT_EQ(runner_.size(), kPairs + 1);

    std::vector<uint64_t> removed_counts;
    removed_counts.reserve(kPairs);
    for (const auto& service : old_services) {
        removed_counts.push_back(service->polls.load(std::memory_order_relaxed));
    }
    const uint64_t anchor_count     = anchor.polls.load(std::memory_order_relaxed);
    const auto replacement_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    for (;;) {
        const bool all_polled = std::all_of(replacements.begin(), replacements.end(), [](const auto& service) {
            return service->polls.load(std::memory_order_relaxed) != 0;
        });
        const bool later_pass_completed = anchor.polls.load(std::memory_order_relaxed) > anchor_count;
        if (all_polled && later_pass_completed) {
            break;
        }
        if (std::chrono::steady_clock::now() >= replacement_deadline) {
            EXPECT_RESULT_OK(runner_.stopAutoRun());
            EXPECT_RESULT_OK(runner_.clear());
            FAIL() << "replacement services were not all polled in a later pass";
        }
        std::this_thread::yield();
    }
    for (size_t i = 0; i < kPairs; ++i) {
        EXPECT_EQ(old_services[i]->polls.load(std::memory_order_relaxed), removed_counts[i]) << "service=" << i;
    }

    EXPECT_RESULT_OK(runner_.stopAutoRun());
    EXPECT_RESULT_OK(runner_.clear());
}

TEST_F(ServiceRunnerAutoRunTest, ActiveAddAcknowledgesDuplicateAndCapacityExactly)
{
    class IdleService final : public IService {
        ServicePoll serviceImpl(const ServiceContext&) override
        {
            return ServicePoll{ServiceResult::Idle};
        }
    };

    ASSERT_RESULT_OK(runner_.startAutoRun());
    std::vector<std::unique_ptr<IdleService>> services;
    services.reserve(ServiceRunner::kMaxServices + 1);

    services.emplace_back(new IdleService{});
    ASSERT_RESULT_OK(runner_.add(*services.back()));
    EXPECT_RESULT_ERROR(runner_.add(*services.back()), error::error_t::INVALID_STATE);

    for (size_t i = 1; i < ServiceRunner::kMaxServices; ++i) {
        services.emplace_back(new IdleService{});
        const auto added = runner_.add(*services.back());
        ASSERT_TRUE(added.has_value()) << "index=" << i << " err=" << error::toString(added.error());
    }
    EXPECT_EQ(runner_.size(), ServiceRunner::kMaxServices);

    services.emplace_back(new IdleService{});
    EXPECT_RESULT_ERROR(runner_.add(*services.back()), error::error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(runner_.size(), ServiceRunner::kMaxServices);

    // The service objects are local to this test, so stop before they unwind.
    EXPECT_RESULT_OK(runner_.stopAutoRun());
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

TEST_F(ServiceRunnerAutoRunTest, ClearWhileAutoRunningStopsAndEmptiesTheTable)
{
    FirstPollLatchService service;
    ASSERT_RESULT_OK(runner_.startAutoRun());
    ASSERT_RESULT_OK(runner_.add(service));

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!service.polled.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::yield();
    }
    const bool was_polled = service.polled.load(std::memory_order_acquire);

    const auto cleared = runner_.clear();
    if (!cleared.has_value()) {
        (void)runner_.stopAutoRun();
        FAIL() << "err=" << error::toString(cleared.error());
    }
    EXPECT_TRUE(was_polled);
    EXPECT_FALSE(runner_.autoRunActive());
    EXPECT_EQ(runner_.size(), 0u);
    ASSERT_RUN_RESULT(runner_.runOnce(ServiceContext{0, 0}), false);
}

TEST_F(ServiceRunnerAutoRunTest, IdleRunnerWakesPromptlyOnAdd)
{
    ASSERT_RESULT_OK(runner_.startAutoRun());  // empty table: the runner idles
    EXPECT_RESULT_ERROR(runner_.runOnce(ServiceContext{0, 0}), error::error_t::BUSY);

    // Repeat the empty->add round trip: each round exercises the
    // latch window (a notify racing the runner's count check / wait
    // entry) at a different phase of the runner's loop.
    for (int round = 0; round < 10; ++round) {
        // Let the runner drain back to its idle wait after the previous
        // round's remove.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        FirstPollLatchService svc;
        const auto t0 = std::chrono::steady_clock::now();
        ASSERT_RESULT_OK(runner_.add(svc));
        while (!svc.polled.load(std::memory_order_acquire)) {
            ASSERT_LT(std::chrono::steady_clock::now() - t0, std::chrono::seconds(5)) << "round " << round;
            std::this_thread::yield();
        }
        const auto elapsed = std::chrono::steady_clock::now() - t0;
        ASSERT_RESULT_OK(runner_.remove(svc));

        // Must beat the 100 ms backstop by a wide margin: a notified
        // wake is scheduler-latency fast, while a missed notification
        // sleeps the full backstop out.
        EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 50) << "round " << round;
    }
}

TEST_F(ServiceRunnerAutoRunTest, StopWhileIdleReturnsPromptly)
{
    ASSERT_RESULT_OK(runner_.startAutoRun());
    // Give the runner time to enter the idle wait, so the stop below
    // exercises the notified wake (not the pre-wait exit check).
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_RESULT_OK(runner_.stopAutoRun());
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
    ASSERT_RESULT_OK(runner_.add(survivor));
    ASSERT_RESULT_OK(runner_.startAutoRun());

    auto count_of = [&](int id) {
        std::lock_guard<std::mutex> lock{log_mutex};
        return std::count(log.begin(), log.end(), id);
    };
    while (count_of(1) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    ASSERT_RESULT_OK(runner_.stopAutoRun());
    const auto stopped_count = count_of(1);

    ASSERT_RESULT_OK(runner_.startAutoRun());
    while (count_of(1) <= stopped_count) {  // the survivor keeps being polled
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    LoggingService fresh{log_mutex, log, 2};
    ASSERT_RESULT_OK(runner_.add(fresh));
    while (count_of(2) == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // Stop before the locals unwind (see ConcurrentAddRemoveWhileAutoRunning).
    EXPECT_RESULT_OK(runner_.stopAutoRun());
}

TEST(ServiceRunnerWake, DestructorJoinsAnIdleWaitingRunnerPromptly)
{
    const auto t0 = std::chrono::steady_clock::now();
    {
        ServiceRunner runner;
        ASSERT_RESULT_OK(runner.startAutoRun());
        // Let the task settle into the idle wait before the dtor stops it.
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    // 20 ms settle + a notified wake; only a missed notification would
    // push this toward the 100 ms backstop.
    EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 80);
}

}  // namespace

#undef ASSERT_RESULT_OK
#undef EXPECT_RESULT_OK
#undef EXPECT_RESULT_ERROR
#undef ASSERT_RUN_RESULT

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
