// Smoke test for the native build environment: verifies that
// PlatformIO + googletest are wired up before the rest of the v1 suite.
#include <gtest/gtest.h>
#include <M5HAL_v1.hpp>

namespace {

class CountingService : public m5::hal::v1::service::IService {
public:
    explicit CountingService(m5::hal::v1::service::ServiceResult result) : _result(result)
    {
    }

    m5::hal::v1::service::ServiceResult service(const m5::hal::v1::service::ServiceContext& ctx) override
    {
        ++count;
        last_now_nsec = ctx.now_nsec;
        return _result;
    }

    int count                                       = 0;
    m5::hal::v1::service::tick_nsec_t last_now_nsec = 0;

private:
    m5::hal::v1::service::ServiceResult _result;
};

TEST(NativeEnvironment, GoogleTestRuns)
{
    EXPECT_EQ(1 + 1, 2);
}

TEST(ServiceRunner, RunsRegisteredServicesInOrder)
{
    using m5::hal::v1::service::ServiceResult;

    m5::hal::v1::service::ServiceRunner runner;
    CountingService idle{ServiceResult::Idle};
    CountingService progress{ServiceResult::Progress};

    EXPECT_TRUE(runner.add(idle));
    EXPECT_TRUE(runner.add(progress));
    EXPECT_FALSE(runner.add(idle));
    EXPECT_EQ(runner.size(), size_t{2});

    EXPECT_TRUE(runner.runOnce(1234));
    EXPECT_EQ(idle.count, 1);
    EXPECT_EQ(progress.count, 1);
    EXPECT_EQ(idle.last_now_nsec, 1234u);
    EXPECT_EQ(progress.last_now_nsec, 1234u);

    EXPECT_TRUE(runner.remove(idle));
    EXPECT_FALSE(runner.remove(idle));
    EXPECT_EQ(runner.size(), size_t{1});

    EXPECT_TRUE(runner.runOnce(5678));
    EXPECT_EQ(idle.count, 1);
    EXPECT_EQ(progress.count, 2);
    EXPECT_EQ(progress.last_now_nsec, 5678u);
}

TEST(ServiceTiming, WrapAwareElapsedAndReached)
{
    using namespace m5::hal::v1::service;

    EXPECT_EQ(elapsedNsec(10, 3), 7u);
    EXPECT_EQ(elapsedNsec(3, 0xFFFFFFFEu), 5u);

    EXPECT_TRUE(hasReached(10, 10));
    EXPECT_TRUE(hasReached(11, 10));
    EXPECT_TRUE(hasReached(3, 0xFFFFFFFEu));
    EXPECT_FALSE(hasReached(9, 10));
}

TEST(ServiceTiming, ConvertsFastTickToNsec)
{
    using namespace m5::hal::v1::service;

    EXPECT_NEAR(fastTickToNsec(240, 240000000), 1000u, 1u);
    EXPECT_NEAR(fastTickToNsec(80, 80000000), 1000u, 1u);
    EXPECT_EQ(fastTickToNsec(1, 1000000), 1000u);
    EXPECT_EQ(fastTickToNsec(123, 0), 123u);
}

TEST(ServiceTiming, ConvertsNsecToFastTick)
{
    using namespace m5::hal::v1::service;

    EXPECT_EQ(nsecToFastTickCeil(1000, 240000000), 240u);
    EXPECT_EQ(nsecToFastTickCeil(1000, 80000000), 80u);
    EXPECT_EQ(nsecToFastTickCeil(1, 240000000), 1u);
    EXPECT_EQ(nsecToFastTickCeil(0, 240000000), 0u);
    EXPECT_EQ(nsecToFastTickCeil(123, 0), 123u);
}

TEST(M5HALCore, OwnsGlobalServiceRunner)
{
    using m5::hal::v1::service::ServiceResult;

    auto& runner = m5::hal::v1::M5_Hal.Services;
    runner.clear();

    CountingService service{ServiceResult::Progress};
    EXPECT_TRUE(runner.add(service));
    EXPECT_TRUE(m5::hal::v1::getM5_Hal().Services.runOnce(1000));
    EXPECT_EQ(service.count, 1);

    EXPECT_TRUE(runner.remove(service));
    EXPECT_EQ(runner.size(), size_t{0});
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
