// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"
#include <M5HAL_v2.hpp>

#include <string>
#include <tuple>
#include <type_traits>

namespace {

namespace native = m5::hal::v2::native;

TEST(NativePolicy, BorrowedIsPointerSizedAndPreservesExactResource)
{
    int resource = 42;
    auto policy  = native::borrowed(resource);

    static_assert(sizeof(policy) == sizeof(void*), "borrowed policy must remain pointer-sized");
    static_assert(native::is_borrowed<decltype(policy)>::value, "borrowed policy trait mismatch");
    static_assert(!native::is_managed<decltype(policy)>::value, "borrowed policy classified as managed");
    EXPECT_EQ(&policy.resource(), &resource);
}

TEST(NativePolicy, ManagedOwnsDecayedArguments)
{
    std::string path = "/dev/example";
    auto policy      = native::managed(path, 7);

    static_assert(native::is_managed<decltype(policy)>::value, "managed policy trait mismatch");
    static_assert(!native::is_borrowed<decltype(policy)>::value, "managed policy classified as borrowed");
    path.clear();
    EXPECT_EQ(std::get<0>(policy.arguments()), "/dev/example");
    EXPECT_EQ(std::get<1>(policy.arguments()), 7);
}

TEST(NativePolicy, AcquireUsesTheSameNameAndRejectsAnUnsupportedSelectedProvider)
{
    m5::hal::v2::i2c::BusConfig config;
    config.pin_scl = 22;
    config.pin_sda = 21;

    auto acquired = m5::hal::v2::getM5_Hal().I2C.acquire(config, native::managed(7));
    ASSERT_FALSE(acquired.has_value());
    EXPECT_EQ(acquired.error(), m5::hal::v2::error::error_t::UNSUPPORTED);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
