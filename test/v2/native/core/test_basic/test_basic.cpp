// SPDX-License-Identifier: MIT
// Smoke test for the native build environment: verifies that
// PlatformIO + googletest are wired up before the rest of the v2 suite.
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

namespace {

#define EXPECT_SERVICE_OK(expression)                                                                \
    do {                                                                                             \
        const auto result_ = (expression);                                                           \
        EXPECT_TRUE(result_.has_value()) << "err=" << m5::hal::v2::error::toString(result_.error()); \
    } while (false)

#define EXPECT_SERVICE_RUN(expression, expected_progress)                                            \
    do {                                                                                             \
        const auto result_ = (expression);                                                           \
        ASSERT_TRUE(result_.has_value()) << "err=" << m5::hal::v2::error::toString(result_.error()); \
        EXPECT_EQ(result_.value(), (expected_progress));                                             \
    } while (false)

class CountingService : public m5::hal::v2::service::IService {
public:
    explicit CountingService(m5::hal::v2::service::ServiceResult result) : _result(result)
    {
    }

    m5::hal::v2::service::ServicePoll serviceImpl(const m5::hal::v2::service::ServiceContext& ctx) override
    {
        ++count;
        last_elapsed = ctx.elapsed;
        return _result;
    }

    int count                                      = 0;
    m5::hal::v2::service::fast_tick_t last_elapsed = 0;

private:
    m5::hal::v2::service::ServiceResult _result;
};

TEST(ErrorToString, ReturnsTheEnumeratorSpelling)
{
    using m5::hal::v2::error::error_t;
    using m5::hal::v2::error::toString;

    EXPECT_STREQ(toString(error_t::OK), "OK");
    EXPECT_STREQ(toString(error_t::I2C_NO_ACK), "I2C_NO_ACK");
    EXPECT_STREQ(toString(error_t::TIMEOUT_ERROR), "TIMEOUT_ERROR");
    EXPECT_STREQ(toString(error_t::DEVICE_MISMATCH), "DEVICE_MISMATCH");
    EXPECT_STREQ(toString(error_t::WOULD_BLOCK), "WOULD_BLOCK");
}

TEST(ErrorToString, UnknownWireValuesDoNotCrash)
{
    using m5::hal::v2::error::error_t;
    using m5::hal::v2::error::toString;

    // A wire-decoded byte from a newer peer may not be in this build's
    // enum; the name lookup must stay total.
    EXPECT_STREQ(toString(static_cast<error_t>(-100)), "UNKNOWN");
}

TEST(ErrorCodes, DeviceMismatchStaysInsideTheWireRange)
{
    using m5::hal::v2::error::error_t;
    // int8_t wire constraint (spec/design/errors.md).
    EXPECT_EQ(static_cast<int8_t>(error_t::DEVICE_MISMATCH), -19);
}

TEST(ErrorCodes, WouldBlockExtendsTheContiguousWireRange)
{
    using m5::hal::v2::error::error_t;
    EXPECT_EQ(static_cast<int8_t>(error_t::WOULD_BLOCK), -22);
    EXPECT_EQ(m5::hal::v2::remote::mapRemoteError(-22), error_t::WOULD_BLOCK);
    EXPECT_EQ(m5::hal::v2::remote::mapRemoteError(-23), error_t::REMOTE_FAULT);
}

TEST(OperationRuntime, UsesWrapSafeRemainingBudgetAndNonzeroGeneration)
{
    namespace bus   = m5::hal::v2::bus;
    namespace types = m5::hal::v2::types;

    bus::OperationRuntime runtime;
    runtime.generation = UINT32_MAX;
    runtime.begin(UINT32_MAX - 3u, 10u, bus::OperationMode::TxRx);
    EXPECT_EQ(runtime.generation, 1u);
    EXPECT_EQ(bus::remainingTimeout(runtime, 2u), 4u);
    EXPECT_EQ(bus::remainingTimeout(runtime, 7u), 0u);

    runtime.begin(123u, types::TIMEOUT_FOREVER, bus::OperationMode::Slave);
    EXPECT_EQ(bus::remainingTimeout(runtime, 456u), types::TIMEOUT_FOREVER);
    EXPECT_EQ(runtime.mode, bus::OperationMode::Slave);
}

TEST(TransferStatus, ZeroIdIsTheUninitializedSentinel)
{
    m5::hal::v2::bus::TransferStatus status;
    EXPECT_EQ(status.transfer_id, 0u);
    EXPECT_EQ(status.error, m5::hal::v2::error::error_t::OK);
    EXPECT_EQ(status.completion, m5::hal::v2::bus::CompletionLevel::None);
}

TEST(NativeEnvironment, GoogleTestRuns)
{
    EXPECT_EQ(1 + 1, 2);
}

TEST(ErrorCode, ClassifiesExtendedFailures)
{
    namespace error = m5::hal::v2::error;

    EXPECT_FALSE(error::isError(error::error_t::OK));
    EXPECT_FALSE(error::isError(error::error_t::ASYNC_RUNNING));
    EXPECT_TRUE(error::isOk(error::error_t::OK));
    EXPECT_FALSE(error::isOk(error::error_t::ASYNC_RUNNING));

    EXPECT_TRUE(error::isError(error::error_t::IO_ERROR));
    EXPECT_TRUE(error::isError(error::error_t::CLOSED));
    EXPECT_TRUE(error::isError(error::error_t::OUT_OF_RESOURCE));
    EXPECT_TRUE(error::isError(error::error_t::BUFFER_OVERFLOW));
    EXPECT_TRUE(error::isError(error::error_t::BUFFER_UNDERFLOW));
    EXPECT_TRUE(error::isError(error::error_t::END_OF_STREAM));
    EXPECT_TRUE(error::isError(error::error_t::CHECKSUM_ERROR));
    EXPECT_TRUE(error::isError(error::error_t::PROTOCOL_ERROR));
}

TEST(ServiceRunner, RunsRegisteredServicesInOrder)
{
    using m5::hal::v2::service::ServiceResult;

    m5::hal::v2::service::ServiceRunner runner;
    CountingService idle{ServiceResult::Idle};
    CountingService progress{ServiceResult::Progress};

    EXPECT_SERVICE_OK(runner.add(idle));
    EXPECT_SERVICE_OK(runner.add(progress));
    const auto duplicate = runner.add(idle);
    ASSERT_FALSE(duplicate.has_value());
    EXPECT_EQ(duplicate.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    EXPECT_EQ(runner.size(), size_t{2});

    EXPECT_SERVICE_RUN(runner.runOnce(m5::hal::v2::service::ServiceContext{1234, 0}), true);
    EXPECT_EQ(idle.count, 1);
    EXPECT_EQ(progress.count, 1);
    EXPECT_EQ(idle.last_elapsed, 1234u);
    EXPECT_EQ(progress.last_elapsed, 1234u);

    EXPECT_SERVICE_OK(runner.remove(idle));
    EXPECT_SERVICE_OK(runner.remove(idle));
    EXPECT_EQ(runner.size(), size_t{1});

    EXPECT_SERVICE_RUN(runner.runOnce(m5::hal::v2::service::ServiceContext{4444, 0}), true);
    EXPECT_EQ(idle.count, 1);
    EXPECT_EQ(progress.count, 2);
    EXPECT_EQ(progress.last_elapsed, 4444u);
}

TEST(ServiceTiming, WrapAwareElapsedAndReached)
{
    using namespace m5::hal::v2::service;

    EXPECT_EQ(elapsedTicks(10, 3), 7u);
    EXPECT_EQ(elapsedTicks(3, 0xFFFFFFFEu), 5u);

    EXPECT_TRUE(hasReached(10, 10));
    EXPECT_TRUE(hasReached(11, 10));
    EXPECT_TRUE(hasReached(3, 0xFFFFFFFEu));
    EXPECT_FALSE(hasReached(9, 10));
}

TEST(ServiceTiming, ConvertsFastTickToNsec)
{
    using namespace m5::hal::v2::service;

    EXPECT_NEAR(fastTickToNsec(240, 240000000), 1000u, 1u);
    EXPECT_NEAR(fastTickToNsec(80, 80000000), 1000u, 1u);
    EXPECT_EQ(fastTickToNsec(1, 1000000), 1000u);
    EXPECT_EQ(fastTickToNsec(123, 0), 123u);
}

TEST(ServiceTiming, ConvertsNsecToFastTick)
{
    using namespace m5::hal::v2::service;

    EXPECT_EQ(nsecToFastTickCeil(1000, 240000000), 240u);
    EXPECT_EQ(nsecToFastTickCeil(1000, 80000000), 80u);
    EXPECT_EQ(nsecToFastTickCeil(1, 240000000), 1u);
    EXPECT_EQ(nsecToFastTickCeil(0, 240000000), 0u);
    EXPECT_EQ(nsecToFastTickCeil(123, 0), 123u);
}

TEST(M5HALCore, OwnsGlobalServiceRunner)
{
    using m5::hal::v2::service::ServiceResult;

    auto& runner = m5::hal::v2::M5_Hal.Services;
    EXPECT_SERVICE_OK(runner.clear());

    CountingService service{ServiceResult::Progress};
    EXPECT_SERVICE_OK(runner.add(service));
    EXPECT_SERVICE_RUN(m5::hal::v2::getM5_Hal().Services.runOnce(m5::hal::v2::service::ServiceContext{1000, 0}), true);
    EXPECT_EQ(service.count, 1);

    EXPECT_SERVICE_OK(runner.remove(service));
    EXPECT_EQ(runner.size(), size_t{0});
}

}  // namespace

#undef EXPECT_SERVICE_OK
#undef EXPECT_SERVICE_RUN

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
