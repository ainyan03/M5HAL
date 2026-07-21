// SPDX-License-Identifier: MIT

#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <cstdint>
#include <type_traits>

namespace {
namespace v2 = m5::hal::v2;

TEST(ResourceKey, FixedSizeAndCheckedPins)
{
    static_assert(std::is_trivially_copyable<v2::bus::ResourceKey>::value, "ResourceKey must remain value-like");
    static_assert(sizeof(v2::bus::ResourceKey) == 32, "ResourceKey production ABI must remain target-independent");
    static_assert(alignof(v2::bus::ResourceKey) <= alignof(uint32_t),
                  "ResourceKey must not force over-aligned embedded allocation");

    const v2::types::gpio_number_t pins[] = {22, 21};
    auto valid                            = v2::bus::ResourceKey::makePins(v2::types::bus_kind_t::I2C, pins, 2);
    ASSERT_TRUE(valid.has_value());
    EXPECT_EQ(valid->kind, v2::types::bus_kind_t::I2C);
    EXPECT_EQ(valid->tag, v2::bus::ResourceTag::Pins);
    EXPECT_EQ(valid->value_count, 2u);

    EXPECT_FALSE(v2::bus::ResourceKey::makePins(v2::types::bus_kind_t::Unknown, pins, 2).has_value());
    EXPECT_FALSE(v2::bus::ResourceKey::makePins(v2::types::bus_kind_t::I2C, nullptr, 2).has_value());
    EXPECT_FALSE(v2::bus::ResourceKey::makePins(v2::types::bus_kind_t::I2C, pins, 0).has_value());
    const v2::types::gpio_number_t too_many[] = {0, 1, 2, 3, 4, 5, 6};
    EXPECT_FALSE(v2::bus::ResourceKey::makePins(v2::types::bus_kind_t::I2C, too_many, 7).has_value());
}

TEST(ResourceKey, MalformedValuesAreNeverEqualOrValid)
{
    v2::bus::ResourceKey empty;
    EXPECT_FALSE(empty.isValid());
    EXPECT_FALSE(empty == empty);

    auto invalid_tag = v2::bus::ResourceKey::fromPins(v2::types::bus_kind_t::I2C, {22, 21});
    invalid_tag.tag  = static_cast<v2::bus::ResourceTag>(0xFF);
    EXPECT_FALSE(invalid_tag.isValid());
    EXPECT_FALSE(invalid_tag == invalid_tag);

    auto invalid_reserved     = v2::bus::ResourceKey::fromPins(v2::types::bus_kind_t::I2C, {22, 21});
    invalid_reserved.reserved = 1;
    EXPECT_FALSE(invalid_reserved.isValid());
}

TEST(ResourceKey, PinsCompareEveryExactField)
{
    const auto base       = v2::bus::ResourceKey::fromPins(v2::types::bus_kind_t::I2C, {22, 21});
    const auto same       = v2::bus::ResourceKey::fromPins(v2::types::bus_kind_t::I2C, {22, 21});
    const auto kind       = v2::bus::ResourceKey::fromPins(v2::types::bus_kind_t::SPI, {22, 21});
    const auto order      = v2::bus::ResourceKey::fromPins(v2::types::bus_kind_t::I2C, {21, 22});
    const auto count      = v2::bus::ResourceKey::fromPins(v2::types::bus_kind_t::I2C, {22});
    const auto trailing   = v2::bus::ResourceKey::fromPins(v2::types::bus_kind_t::I2C, {22, 21, -1});
    auto malformed        = base;
    malformed.value_count = 0;

    EXPECT_EQ(base, same);
    EXPECT_FALSE(base == kind);
    EXPECT_FALSE(base == order);
    EXPECT_FALSE(base == count);
    EXPECT_FALSE(base == trailing);
    EXPECT_FALSE(base == malformed);
}

TEST(ResourceKey, EveryPortableBusKindProjectsItsExactPhysicalPins)
{
    v2::i2c::IBusConfig i2c;
    i2c.pin_scl  = 22;
    i2c.pin_sda  = 21;
    auto i2c_key = v2::i2c::BusTraits::identityFromConfig(i2c);

    v2::spi::IBusConfig spi;
    spi.pin_clk  = 18;
    spi.pin_mosi = 23;
    spi.pin_miso = 19;
    auto spi_key = v2::spi::BusTraits::identityFromConfig(spi);

    v2::uart::IBusConfig uart;
    uart.pin_tx   = 17;
    uart.pin_rx   = 16;
    auto uart_key = v2::uart::BusTraits::identityFromConfig(uart);

    v2::i2s::IBusConfig i2s;
    i2s.pin_bclk = 26;
    i2s.pin_ws   = 25;
    i2s.pin_dout = 32;
    i2s.pin_din  = 33;
    auto i2s_key = v2::i2s::BusTraits::identityFromConfig(i2s);

    v2::pdm::IBusConfig pdm;
    pdm.pin_clk  = 0;
    pdm.pin_din  = 34;
    auto pdm_key = v2::pdm::BusTraits::identityFromConfig(pdm);

    const v2::bus::ResourceKey* keys[]  = {&i2c_key, &spi_key, &uart_key, &i2s_key, &pdm_key};
    const v2::types::bus_kind_t kinds[] = {v2::types::bus_kind_t::I2C, v2::types::bus_kind_t::SPI,
                                           v2::types::bus_kind_t::UART, v2::types::bus_kind_t::I2S,
                                           v2::types::bus_kind_t::PDM};
    const uint8_t counts[]              = {2, 3, 2, 4, 2};
    for (size_t i = 0; i < 5; ++i) {
        EXPECT_TRUE(keys[i]->isValid());
        EXPECT_EQ(keys[i]->kind, kinds[i]);
        EXPECT_EQ(keys[i]->tag, v2::bus::ResourceTag::Pins);
        EXPECT_EQ(keys[i]->value_count, counts[i]);
    }

    auto changed    = pdm;
    changed.pin_din = 35;
    EXPECT_FALSE(pdm_key == v2::pdm::BusTraits::identityFromConfig(changed));
}

TEST(ResourceKey, NativeAndPathTokensAreTypedAndGenerationExact)
{
    const v2::bus::SlotGeneration first{1, 0, 7};
    const v2::bus::SlotGeneration next{1, 0, 8};
    auto native = v2::bus::ResourceKey::makeToken(v2::types::bus_kind_t::UART, v2::bus::ResourceTag::Native, first, 3);
    auto native_same =
        v2::bus::ResourceKey::makeToken(v2::types::bus_kind_t::UART, v2::bus::ResourceTag::Native, first, 3);
    auto generation =
        v2::bus::ResourceKey::makeToken(v2::types::bus_kind_t::UART, v2::bus::ResourceTag::Native, next, 3);
    auto type = v2::bus::ResourceKey::makeToken(v2::types::bus_kind_t::UART, v2::bus::ResourceTag::Native, first, 4);
    auto path = v2::bus::ResourceKey::makeToken(v2::types::bus_kind_t::UART, v2::bus::ResourceTag::Path, first, 3);

    ASSERT_TRUE(native.has_value());
    ASSERT_TRUE(native_same.has_value());
    ASSERT_TRUE(generation.has_value());
    ASSERT_TRUE(type.has_value());
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ(native.value(), native_same.value());
    EXPECT_FALSE(native.value() == generation.value());
    EXPECT_FALSE(native.value() == type.value());
    EXPECT_FALSE(native.value() == path.value());

    EXPECT_FALSE(
        v2::bus::ResourceKey::makeToken(v2::types::bus_kind_t::UART, v2::bus::ResourceTag::Pins, first, 3).has_value());
    EXPECT_FALSE(
        v2::bus::ResourceKey::makeToken(v2::types::bus_kind_t::UART, v2::bus::ResourceTag::Native, {}, 3).has_value());
    EXPECT_FALSE(v2::bus::ResourceKey::makeToken(v2::types::bus_kind_t::UART, v2::bus::ResourceTag::Native,
                                                 v2::bus::SlotGeneration{1, 1, 7}, 3)
                     .has_value());
}

TEST(ResourceKey, RemoteSessionAndTargetAreExact)
{
    const v2::types::gpio_number_t target[]       = {3, 4};
    const v2::types::gpio_number_t other_target[] = {3, 5};
    auto base  = v2::bus::ResourceKey::makeRemote(v2::types::bus_kind_t::I2C, 11, target, 2, 1);
    auto same  = v2::bus::ResourceKey::makeRemote(v2::types::bus_kind_t::I2C, 11, target, 2, 1);
    auto sess  = v2::bus::ResourceKey::makeRemote(v2::types::bus_kind_t::I2C, 12, target, 2, 1);
    auto peer  = v2::bus::ResourceKey::makeRemote(v2::types::bus_kind_t::I2C, 11, other_target, 2, 1);
    auto tkind = v2::bus::ResourceKey::makeRemote(v2::types::bus_kind_t::I2C, 11, target, 2, 2);

    ASSERT_TRUE(base.has_value());
    ASSERT_TRUE(same.has_value());
    ASSERT_TRUE(sess.has_value());
    ASSERT_TRUE(peer.has_value());
    ASSERT_TRUE(tkind.has_value());
    EXPECT_EQ(base.value(), same.value());
    EXPECT_FALSE(base.value() == sess.value());
    EXPECT_FALSE(base.value() == peer.value());
    EXPECT_FALSE(base.value() == tkind.value());
    EXPECT_FALSE(v2::bus::ResourceKey::makeRemote(v2::types::bus_kind_t::I2C, 0, target, 2, 1).has_value());
}

TEST(ResourceKey, RemoteConnectionOwnerGenerationsAreNonzeroAndMonotonic)
{
    v2::remote::RemoteSessionHandle first;
    v2::remote::RemoteSessionHandle second;
    ASSERT_NE(first.generation(), 0u);
    ASSERT_NE(second.generation(), 0u);
    EXPECT_LT(first.generation(), second.generation());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
