// SPDX-License-Identifier: MIT

#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <cstdint>
#include <type_traits>

namespace {
namespace v2 = m5::hal::v2;

static_assert(sizeof(v2::bus::NativeIdentity) == 32u);
static_assert(sizeof(v2::bus::BindingDescriptor) == 32u);
static_assert(std::is_trivially_copyable<v2::bus::NativeIdentity>::value);
static_assert(std::is_trivially_copyable<v2::bus::BindingDescriptor>::value);

v2::bus::NativeIdentity identity(uint64_t value)
{
    auto made = v2::bus::NativeIdentity::make(v2::bus::NativeIdentityKind::ObjectAddress, {value});
    EXPECT_TRUE(made.has_value());
    return made.has_value() ? made.value() : v2::bus::NativeIdentity{};
}

TEST(NativeIdentity, TypeValueCountAndEveryValueAreExact)
{
    auto object  = v2::bus::NativeIdentity::make(v2::bus::NativeIdentityKind::ObjectAddress, {1, 2, 3});
    auto device  = v2::bus::NativeIdentity::make(v2::bus::NativeIdentityKind::PosixDevice, {1, 2, 3});
    auto shorter = v2::bus::NativeIdentity::make(v2::bus::NativeIdentityKind::ObjectAddress, {1, 2});
    auto changed = v2::bus::NativeIdentity::make(v2::bus::NativeIdentityKind::ObjectAddress, {1, 2, 4});
    ASSERT_TRUE(object.has_value());
    ASSERT_TRUE(device.has_value());
    ASSERT_TRUE(shorter.has_value());
    ASSERT_TRUE(changed.has_value());
    EXPECT_TRUE(object->isValid());
    EXPECT_FALSE(object.value() == device.value());
    EXPECT_FALSE(object.value() == shorter.value());
    EXPECT_FALSE(object.value() == changed.value());

    EXPECT_FALSE(v2::bus::NativeIdentity::make(v2::bus::NativeIdentityKind::None, {1}).has_value());
    EXPECT_FALSE(v2::bus::NativeIdentity::make(v2::bus::NativeIdentityKind::ObjectAddress, {}).has_value());
    EXPECT_FALSE(v2::bus::NativeIdentity::make(v2::bus::NativeIdentityKind::ObjectAddress, {1, 2, 3, 4}).has_value());
}

TEST(BindingDescriptor, EveryFieldIncludingTokenReservedBitsParticipatesInEquality)
{
    v2::bus::BindingDescriptor base;
    base.provider         = 7;
    base.ownership        = v2::bus::Ownership::Borrowed;
    base.native_kind      = v2::bus::NativeBindingKind::NativeAndPath;
    base.native           = {1, 0, 2};
    base.path             = {3, 0, 4};
    base.config_primary   = 5;
    base.config_secondary = 6;
    base.native_options   = 7;

    EXPECT_TRUE(base == base);
#define EXPECT_FIELD_DIFFERENT(field, value) \
    do {                                     \
        auto other  = base;                  \
        other.field = value;                 \
        EXPECT_FALSE(base == other);         \
    } while (false)
    EXPECT_FIELD_DIFFERENT(provider, 8);
    EXPECT_FIELD_DIFFERENT(ownership, v2::bus::Ownership::Managed);
    EXPECT_FIELD_DIFFERENT(native_kind, v2::bus::NativeBindingKind::Native);
    EXPECT_FIELD_DIFFERENT(native, (v2::bus::NativeToken{1, 1, 2}));
    EXPECT_FIELD_DIFFERENT(path, (v2::bus::PathToken{3, 1, 4}));
    EXPECT_FIELD_DIFFERENT(config_primary, 8);
    EXPECT_FIELD_DIFFERENT(config_secondary, 8);
    EXPECT_FIELD_DIFFERENT(native_options, 8);
#undef EXPECT_FIELD_DIFFERENT
}

TEST(NativeInterner, ReferenceCountsAndAbaGenerationAreExact)
{
    v2::bus::FixedNativeInterner<v2::bus::NativeIdentity, 1> interner;
    auto first = interner.intern(identity(0x1234));
    auto alias = interner.intern(identity(0x1234));
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(alias.has_value());
    EXPECT_EQ(first.value(), alias.value());

    ASSERT_TRUE(interner.release(first.value()).has_value());
    EXPECT_TRUE(interner.resolve(alias.value()).has_value());
    ASSERT_TRUE(interner.release(alias.value()).has_value());
    EXPECT_FALSE(interner.resolve(first.value()).has_value());

    auto reused = interner.intern(identity(0x1234));
    ASSERT_TRUE(reused.has_value());
    EXPECT_EQ(reused->slot, first->slot);
    EXPECT_NE(reused->generation, first->generation);
    EXPECT_FALSE(interner.release(first.value()).has_value());
}

TEST(ResourceDomain, NativeIdentityStorageIsOwnedAndSharedByTheDomain)
{
    v2::ResourceDomain domain;
    v2::ResourceDomain alias = domain;
    v2::ResourceDomain other;
    EXPECT_EQ(&domain.nativeInterner(), &alias.nativeInterner());
    EXPECT_NE(&domain.nativeInterner(), &other.nativeInterner());

    auto token = domain.nativeInterner().intern(identity(0x4321));
    ASSERT_TRUE(token.has_value());
    auto resolved = alias.nativeInterner().resolve(token.value());
    ASSERT_TRUE(resolved.has_value());
    EXPECT_TRUE(resolved.value() == identity(0x4321));
    EXPECT_FALSE(other.nativeInterner().resolve(token.value()).has_value());
    EXPECT_TRUE(alias.nativeInterner().release(token.value()).has_value());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
