// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/remote/server_bus_pool.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace {

using namespace m5::hal::v2;

#define EXPECT_OK_RESULT(expr)                                                                    \
    do {                                                                                          \
        auto m5hal_result = (expr);                                                               \
        ASSERT_TRUE(m5hal_result.has_value()) << "err=" << error::toString(m5hal_result.error()); \
    } while (false)

void expectError(const result_t<void>& r, error::error_t expected)
{
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), expected) << "err=" << error::toString(r.error());
}

template <size_t N>
data::ConstDataSpan spanOf(const std::array<uint8_t, N>& bytes)
{
    return data::ConstDataSpan{bytes.data(), bytes.size()};
}

void putI16LE(std::array<uint8_t, 4>& out, size_t offset, int16_t value)
{
    const auto v    = static_cast<uint16_t>(value);
    out[offset]     = static_cast<uint8_t>(v & 0xFFu);
    out[offset + 1] = static_cast<uint8_t>(v >> 8);
}

std::array<uint8_t, 4> i2cConfig(int16_t scl, int16_t sda)
{
    std::array<uint8_t, 4> out{};
    putI16LE(out, 0, scl);
    putI16LE(out, 2, sda);
    return out;
}

std::array<uint8_t, 5> i2cConfigWithExtraByte(int16_t scl, int16_t sda, uint8_t extra)
{
    std::array<uint8_t, 5> out{};
    const auto base = i2cConfig(scl, sda);
    out[0]          = base[0];
    out[1]          = base[1];
    out[2]          = base[2];
    out[3]          = base[3];
    out[4]          = extra;
    return out;
}

size_t usedI2CSlots(const remote::ServerPhysicalBusPool& phys)
{
    size_t count = 0;
    for (size_t i = 0; i < remote::kServerBusPoolSlots; ++i) {
        if (phys.i2c[i].used) {
            ++count;
        }
    }
    return count;
}

const remote::PhysI2CSlot* firstUsedI2CSlot(const remote::ServerPhysicalBusPool& phys)
{
    for (size_t i = 0; i < remote::kServerBusPoolSlots; ++i) {
        if (phys.i2c[i].used) {
            return &phys.i2c[i];
        }
    }
    return nullptr;
}

const remote::I2CSlot* findI2CBinding(const remote::ServerBusPool& pool, uint8_t bus_id)
{
    for (size_t i = 0; i < remote::kServerBusPoolSlots; ++i) {
        if (pool.i2c[i].used && pool.i2c[i].bus_id == bus_id) {
            return &pool.i2c[i];
        }
    }
    return nullptr;
}

struct ServerCtx {
    explicit ServerCtx(remote::ServerPhysicalBusPool& phys_pool) : server{data::DataSpan{scratch, sizeof(scratch)}}
    {
        pool.server = &server;
        pool.phys   = &phys_pool;
    }

    uint8_t scratch[remote::kMaxScriptSize];
    remote::Server server;
    remote::ServerBusPool pool;
};

result_t<void> createI2C(ServerCtx& ctx, uint8_t bus_id, data::ConstDataSpan pin_config)
{
    return remote::ServerBusPool::handler(&ctx.pool, true, types::bus_kind_t::I2C, bus_id, pin_config);
}

result_t<void> releaseI2C(ServerCtx& ctx, uint8_t bus_id)
{
    return remote::ServerBusPool::handler(&ctx.pool, false, types::bus_kind_t::I2C, bus_id, {nullptr, 0});
}

class ServerBusPoolPhysicalSharing : public ::testing::Test {
protected:
    void SetUp() override
    {
        EXPECT_OK_RESULT(M5_Hal.init());
    }
};

TEST_F(ServerBusPoolPhysicalSharing, SamePinConfigSharesOnePhysicalSlot)
{
    remote::ServerPhysicalBusPool phys;
    ServerCtx a{phys};
    ServerCtx b{phys};
    const auto cfg = i2cConfig(22, 21);

    EXPECT_OK_RESULT(createI2C(a, 1, spanOf(cfg)));
    EXPECT_OK_RESULT(createI2C(b, 1, spanOf(cfg)));

    EXPECT_EQ(usedI2CSlots(phys), 1u);
    ASSERT_NE(firstUsedI2CSlot(phys), nullptr);
    EXPECT_EQ(firstUsedI2CSlot(phys)->refcount, 2u);
    ASSERT_NE(findI2CBinding(a.pool, 1), nullptr);
    ASSERT_NE(findI2CBinding(b.pool, 1), nullptr);

    EXPECT_OK_RESULT(releaseI2C(a, 1));
    EXPECT_OK_RESULT(releaseI2C(b, 1));
}

TEST_F(ServerBusPoolPhysicalSharing, SamePinsDifferentConfigBytesAreRejected)
{
    remote::ServerPhysicalBusPool phys;
    ServerCtx a{phys};
    ServerCtx b{phys};
    const auto cfg_a = i2cConfig(22, 21);
    const auto cfg_b = i2cConfigWithExtraByte(22, 21, 0xA5);

    EXPECT_OK_RESULT(createI2C(a, 2, spanOf(cfg_a)));
    expectError(createI2C(b, 2, spanOf(cfg_b)), error::error_t::INVALID_STATE);

    EXPECT_EQ(usedI2CSlots(phys), 1u);
    ASSERT_NE(firstUsedI2CSlot(phys), nullptr);
    EXPECT_EQ(firstUsedI2CSlot(phys)->refcount, 1u);
    EXPECT_EQ(findI2CBinding(b.pool, 2), nullptr);

    EXPECT_OK_RESULT(releaseI2C(a, 2));
}

TEST_F(ServerBusPoolPhysicalSharing, ReleaseKeepsOtherBindingAndFreesWhenLastReferenceDrops)
{
    remote::ServerPhysicalBusPool phys;
    ServerCtx a{phys};
    ServerCtx b{phys};
    const auto cfg = i2cConfig(22, 21);

    EXPECT_OK_RESULT(createI2C(a, 3, spanOf(cfg)));
    EXPECT_OK_RESULT(createI2C(b, 3, spanOf(cfg)));

    EXPECT_OK_RESULT(releaseI2C(a, 3));
    EXPECT_EQ(usedI2CSlots(phys), 1u);
    ASSERT_NE(firstUsedI2CSlot(phys), nullptr);
    EXPECT_EQ(firstUsedI2CSlot(phys)->refcount, 1u);
    ASSERT_NE(findI2CBinding(b.pool, 3), nullptr);
    EXPECT_NE(findI2CBinding(b.pool, 3)->acc, nullptr);

    EXPECT_OK_RESULT(releaseI2C(b, 3));
    EXPECT_EQ(usedI2CSlots(phys), 0u);

    const auto next_cfg = i2cConfig(20, 19);
    EXPECT_OK_RESULT(createI2C(a, 0, spanOf(next_cfg)));
    EXPECT_EQ(usedI2CSlots(phys), 1u);
    ASSERT_NE(firstUsedI2CSlot(phys), nullptr);
    EXPECT_EQ(firstUsedI2CSlot(phys)->refcount, 1u);
    EXPECT_OK_RESULT(releaseI2C(a, 0));
}

TEST_F(ServerBusPoolPhysicalSharing, ReleaseAllOnlyDropsThisConnectionReference)
{
    remote::ServerPhysicalBusPool phys;
    ServerCtx a{phys};
    ServerCtx b{phys};
    const auto cfg = i2cConfig(22, 21);

    EXPECT_OK_RESULT(createI2C(a, 0, spanOf(cfg)));
    EXPECT_OK_RESULT(createI2C(b, 0, spanOf(cfg)));

    a.pool.releaseAll();
    EXPECT_EQ(usedI2CSlots(phys), 1u);
    ASSERT_NE(firstUsedI2CSlot(phys), nullptr);
    EXPECT_EQ(firstUsedI2CSlot(phys)->refcount, 1u);
    EXPECT_EQ(findI2CBinding(a.pool, 0), nullptr);
    ASSERT_NE(findI2CBinding(b.pool, 0), nullptr);
    EXPECT_NE(findI2CBinding(b.pool, 0)->acc, nullptr);

    b.pool.releaseAll();
    EXPECT_EQ(usedI2CSlots(phys), 0u);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
