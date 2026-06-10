// Native gtest for service::ServiceRunner (hal/v1/service/service.hpp).
//
// test_basic covers the happy path (registration order) and the tick
// conversion helpers; this suite pins the registry semantics the
// cooperative runner promises its services: duplicate / over-capacity
// registration is rejected, removal compacts while preserving order,
// runOnce aggregates Progress|Done into its return value, and a
// service may remove itself from inside its own service() callback
// (the bit-bang state machines end themselves exactly this way).

#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

#include <cstdint>
#include <vector>

namespace {

namespace service = ::m5::hal::v1::service;
using service::IService;
using service::ServiceContext;
using service::ServiceResult;
using service::ServiceRunner;

// Appends its id to a shared log on every call and returns a scripted
// result. Optionally removes itself from the runner when it is called.
class ScriptedService : public IService {
public:
    ScriptedService(std::vector<int>& log, int id, ServiceResult result) : _log{&log}, _id{id}, _result{result}
    {
    }

    ServiceResult service(const ServiceContext& ctx) override
    {
        _log->push_back(_id);
        last_now = ctx.now_nsec;
        if (remove_self_from != nullptr) {
            (void)remove_self_from->remove(*this);
        }
        return _result;
    }

    ServiceRunner* remove_self_from = nullptr;
    service::tick_nsec_t last_now   = 0;

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

    std::vector<ScriptedService> fill;
    fill.reserve(ServiceRunner::kMaxServices);
    for (size_t i = 1; i < ServiceRunner::kMaxServices; ++i) {
        fill.emplace_back(log, static_cast<int>(100 + i), ServiceResult::Idle);
        EXPECT_TRUE(runner.add(fill.back())) << i;
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

    (void)runner.runOnce(ServiceContext{0});
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
    EXPECT_FALSE(runner.runOnce(ServiceContext{0}));  // nothing to run
    EXPECT_TRUE(log.empty());

    EXPECT_TRUE(runner.add(a));  // re-registration after clear works
    EXPECT_TRUE(runner.runOnce(ServiceContext{0}));
}

TEST(ServiceRunner, RunOnceAggregatesProgressAndDone)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService idle{log, 1, ServiceResult::Idle};
    ScriptedService error{log, 2, ServiceResult::Error};
    ASSERT_TRUE(runner.add(idle));
    ASSERT_TRUE(runner.add(error));
    EXPECT_FALSE(runner.runOnce(ServiceContext{10}));  // Idle/Error: no progress

    ScriptedService progress{log, 3, ServiceResult::Progress};
    ASSERT_TRUE(runner.add(progress));
    EXPECT_TRUE(runner.runOnce(ServiceContext{20}));

    // The context tick reaches every service untouched.
    EXPECT_EQ(idle.last_now, 20u);
    EXPECT_EQ(progress.last_now, 20u);
}

TEST(ServiceRunner, ServiceMayRemoveItselfWhileRunning)
{
    std::vector<int> log;
    ServiceRunner runner;
    ScriptedService a{log, 1, ServiceResult::Done};
    ScriptedService b{log, 2, ServiceResult::Idle};
    a.remove_self_from = &runner;
    ASSERT_TRUE(runner.add(a));
    ASSERT_TRUE(runner.add(b));

    // The self-removing pass must not crash and must report progress
    // (a returned Done). In-flight table edits may defer a sibling to
    // the next pass, so b's call count is asserted over two passes.
    EXPECT_TRUE(runner.runOnce(ServiceContext{0}));
    EXPECT_EQ(runner.size(), 1u);

    log.clear();
    (void)runner.runOnce(ServiceContext{1});
    EXPECT_EQ(log, (std::vector<int>{2}));  // a is gone, b still runs
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
