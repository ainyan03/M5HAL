// SPDX-License-Identifier: MIT

#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>

namespace {
namespace v2 = m5::hal::v2;

uint32_t constantHash(const char*, size_t)
{
    return 7;
}

TEST(PathInterner, ExactBytesSurviveSameHashAndSameLengthCollision)
{
    v2::bus::FixedPathInterner<2, 16> paths{&constantHash};
    auto first  = paths.intern("/dev/a", 6);
    auto second = paths.intern("/dev/b", 6);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_FALSE(first.value() == second.value());

    char out[16] = {};
    auto copied  = paths.copy(second.value(), out, sizeof(out));
    ASSERT_TRUE(copied.has_value());
    EXPECT_EQ(copied.value(), 6u);
    EXPECT_EQ(std::memcmp(out, "/dev/b", 6), 0);
}

TEST(PathInterner, CapacityReferenceCountingAndStaleGenerationAreExact)
{
    v2::bus::FixedPathInterner<2, 16> paths;
    auto first       = paths.intern("/dev/a", 6);
    auto first_alias = paths.intern("/dev/a", 6);
    auto second      = paths.intern("/dev/b", 6);
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(first_alias.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(first.value(), first_alias.value());
    auto full = paths.intern("/dev/c", 6);
    ASSERT_FALSE(full.has_value());
    EXPECT_EQ(full.error(), v2::error::error_t::OUT_OF_RESOURCE);

    ASSERT_TRUE(paths.release(first.value()).has_value());
    char out[16] = {};
    EXPECT_TRUE(paths.copy(first.value(), out, sizeof(out)).has_value());
    ASSERT_TRUE(paths.release(first_alias.value()).has_value());
    EXPECT_FALSE(paths.copy(first.value(), out, sizeof(out)).has_value());

    auto reused = paths.intern("/dev/c", 6);
    ASSERT_TRUE(reused.has_value());
    EXPECT_EQ(reused->slot, first->slot);
    EXPECT_NE(reused->generation, first->generation);
    EXPECT_FALSE(paths.release(first.value()).has_value());
}

TEST(NativeInterner, AddressReuseProducesNewGenerationAndRejectsStaleToken)
{
    v2::bus::FixedNativeInterner<uintptr_t, 1> natives;
    auto old = natives.intern(0x1234u);
    ASSERT_TRUE(old.has_value());
    ASSERT_TRUE(natives.release(old.value()).has_value());

    auto current = natives.intern(0x1234u);
    ASSERT_TRUE(current.has_value());
    EXPECT_EQ(current->slot, old->slot);
    EXPECT_NE(current->generation, old->generation);
    EXPECT_FALSE(natives.resolve(old.value()).has_value());
    ASSERT_TRUE(natives.resolve(current.value()).has_value());
    EXPECT_EQ(natives.resolve(current.value()).value(), 0x1234u);
}

TEST(PathInterner, MaximumGenerationRetiresSlotPermanently)
{
    constexpr uint32_t almost_max = std::numeric_limits<uint32_t>::max() - 1u;
    v2::bus::FixedPathInterner<1, 16, almost_max> paths;

    auto last = paths.intern("/dev/a", 6);
    ASSERT_TRUE(last.has_value());
    EXPECT_EQ(last->generation, std::numeric_limits<uint32_t>::max());
    ASSERT_TRUE(paths.release(last.value()).has_value());

    auto retired = paths.intern("/dev/b", 6);
    ASSERT_FALSE(retired.has_value());
    EXPECT_EQ(retired.error(), v2::error::error_t::OUT_OF_RESOURCE);
}

TEST(NativeInterner, MaximumGenerationRetiresSlotPermanently)
{
    constexpr uint32_t almost_max = std::numeric_limits<uint32_t>::max() - 1u;
    v2::bus::FixedNativeInterner<uintptr_t, 1, almost_max> natives;

    auto last = natives.intern(0x1234u);
    ASSERT_TRUE(last.has_value());
    EXPECT_EQ(last->generation, std::numeric_limits<uint32_t>::max());
    ASSERT_TRUE(natives.release(last.value()).has_value());

    auto retired = natives.intern(0x5678u);
    ASSERT_FALSE(retired.has_value());
    EXPECT_EQ(retired.error(), v2::error::error_t::OUT_OF_RESOURCE);
}

TEST(ResourceDomain, PathStorageIsOptionalAndCoOwnedOnlyWhenInjected)
{
    v2::ResourceDomain pathless;
    EXPECT_EQ(pathless.pathInterner(), nullptr);

    auto storage                               = std::make_shared<v2::bus::FixedPathInterner<2, 32>>();
    std::weak_ptr<v2::bus::IPathInterner> weak = storage;
    v2::ResourceDomain with_paths{storage};
    storage.reset();
    EXPECT_FALSE(weak.expired());
    EXPECT_EQ(with_paths.pathInterner(), weak.lock().get());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
