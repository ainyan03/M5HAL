// SPDX-License-Identifier: MIT
#include <M5HAL_v1.hpp>
#include <gtest/gtest.h>

#include <vector>

namespace {

// -------------------------------------------------------------------------
// Stub I2S bus that records all calls for inspection.
// -------------------------------------------------------------------------
class StubIBus : public m5::hal::v1::i2s::IBus {
public:
    // Typed init: the fake adds no fields, so it takes the
    // abstract kind config.
    m5::hal::v1::result_t<void> init(const m5::hal::v1::i2s::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    m5::hal::v1::result_t<void> release(void) override
    {
        tx_recorded.clear();
        return {};
    }

    m5::hal::v1::result_t<size_t> write(m5::hal::v1::bus::IAccessor* owner, const m5::hal::v1::i2s::AccessConfig& cfg,
                                        m5::hal::v1::data::Source* tx, size_t len) override
    {
        last_owner  = owner;
        last_cfg    = cfg;
        size_t done = 0;
        while (tx != nullptr && !tx->eof() && done < len) {
            auto span = tx->peek(len - done);
            if (!span.has_value()) {
                return m5::stl::make_unexpected(span.error());
            }
            if (span.value().size == 0) {
                break;
            }
            tx_recorded.insert(tx_recorded.end(), span.value().data, span.value().data + span.value().size);
            auto advanced = tx->advance(span.value().size);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
            done += span.value().size;
        }
        return done;
    }

    m5::hal::v1::result_t<size_t> writableBytes(m5::hal::v1::bus::IAccessor* owner,
                                                const m5::hal::v1::i2s::AccessConfig& cfg) override
    {
        last_owner = owner;
        last_cfg   = cfg;
        return stub_writable;
    }

    std::vector<uint8_t> tx_recorded;
    m5::hal::v1::bus::IAccessor* last_owner = nullptr;
    m5::hal::v1::i2s::AccessConfig last_cfg;
    size_t stub_writable = 1024;
};

}  // namespace

// -------------------------------------------------------------------------
// IBusConfig defaults
// -------------------------------------------------------------------------
TEST(IBusConfig, DefaultCtorSetsI2SKind)
{
    m5::hal::v1::i2s::IBusConfig cfg;
    EXPECT_EQ(cfg.getBusKind(), m5::hal::v1::types::bus_kind_t::I2S);
    EXPECT_EQ(cfg.pin_bclk, -1);
    EXPECT_EQ(cfg.pin_ws, -1);
    EXPECT_EQ(cfg.pin_dout, -1);
    EXPECT_EQ(cfg.tx_buffer_size, 8192u);
}

// -------------------------------------------------------------------------
// IBus base default returns NOT_IMPLEMENTED
// -------------------------------------------------------------------------
TEST(IBus, BaseDefaultWriteReturnsNotImplemented)
{
    m5::hal::v1::i2s::IBus bus;
    m5::hal::v1::i2s::AccessConfig cfg;
    auto result = bus.write(nullptr, cfg, nullptr, 0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v1::error::error_t::NOT_IMPLEMENTED);
}

TEST(IBus, BaseDefaultWritableBytesReturnsNotImplemented)
{
    m5::hal::v1::i2s::IBus bus;
    m5::hal::v1::i2s::AccessConfig cfg;
    auto result = bus.writableBytes(nullptr, cfg);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v1::error::error_t::NOT_IMPLEMENTED);
}

// -------------------------------------------------------------------------
// TxAccessor forwards writes to bus
// -------------------------------------------------------------------------
TEST(TxAccessor, WriteConstDataSpanForwardsTobus)
{
    StubIBus bus;
    m5::hal::v1::i2s::AccessConfig cfg;
    cfg.sample_rate_hz = 44100;
    m5::hal::v1::i2s::TxAccessor acc{bus, cfg};

    const uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    auto result             = acc.write(m5::hal::v1::data::ConstDataSpan{payload, sizeof(payload)});
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(payload));
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    EXPECT_EQ(bus.tx_recorded[0], 0x01u);
    EXPECT_EQ(bus.tx_recorded[3], 0x04u);
    EXPECT_EQ(bus.last_owner, &acc);
    EXPECT_EQ(bus.last_cfg.sample_rate_hz, 44100u);
}

TEST(TxAccessor, WriteRawPtrForwardsTobus)
{
    StubIBus bus;
    m5::hal::v1::i2s::AccessConfig cfg;
    m5::hal::v1::i2s::TxAccessor acc{bus, cfg};

    const uint8_t payload[] = {0xAA, 0xBB};
    auto result             = acc.write(payload, sizeof(payload));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(payload));
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    EXPECT_EQ(bus.tx_recorded[0], 0xAAu);
    EXPECT_EQ(bus.tx_recorded[1], 0xBBu);
}

TEST(TxAccessor, WriteSourceForwardsTobus)
{
    StubIBus bus;
    m5::hal::v1::i2s::AccessConfig cfg;
    m5::hal::v1::i2s::TxAccessor acc{bus, cfg};

    const uint8_t payload[] = {0x10, 0x20, 0x30};
    m5::hal::v1::data::MemorySource src{m5::hal::v1::data::ConstDataSpan{payload, sizeof(payload)}};
    auto result = acc.write(src, sizeof(payload));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(payload));
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    EXPECT_EQ(bus.tx_recorded[1], 0x20u);
}

// All three write overloads must produce the same byte sequence.
TEST(TxAccessor, WriteOverloadsAreEquivalent)
{
    const uint8_t payload[] = {0xDE, 0xAD};

    // via ConstDataSpan
    {
        StubIBus bus;
        m5::hal::v1::i2s::TxAccessor acc{bus, {}};
        acc.write(m5::hal::v1::data::ConstDataSpan{payload, sizeof(payload)});
        ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
        EXPECT_EQ(bus.tx_recorded[0], 0xDEu);
    }
    // via raw pointer
    {
        StubIBus bus;
        m5::hal::v1::i2s::TxAccessor acc{bus, {}};
        acc.write(payload, sizeof(payload));
        ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
        EXPECT_EQ(bus.tx_recorded[0], 0xDEu);
    }
    // via Source
    {
        StubIBus bus;
        m5::hal::v1::i2s::TxAccessor acc{bus, {}};
        m5::hal::v1::data::MemorySource src{m5::hal::v1::data::ConstDataSpan{payload, sizeof(payload)}};
        acc.write(src, sizeof(payload));
        ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
        EXPECT_EQ(bus.tx_recorded[0], 0xDEu);
    }
}

// -------------------------------------------------------------------------
// writableBytes forwarding
// -------------------------------------------------------------------------
TEST(TxAccessor, WritableBytesForwardsTobus)
{
    StubIBus bus;
    bus.stub_writable = 2048;
    m5::hal::v1::i2s::TxAccessor acc{bus, {}};

    auto result = acc.writableBytes();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 2048u);
    EXPECT_EQ(bus.last_owner, &acc);
}

// -------------------------------------------------------------------------
// setConfig rejects inside access, applies outside
// -------------------------------------------------------------------------
TEST(TxAccessor, SetConfigRejectsInsideAccess)
{
    StubIBus bus;
    m5::hal::v1::i2s::AccessConfig cfg;
    m5::hal::v1::i2s::TxAccessor acc{bus, cfg};

    // Acquire the lock explicitly so inAccess() is true.
    ASSERT_TRUE(acc.beginAccess(0).has_value());
    cfg.sample_rate_hz = 48000;
    auto result        = acc.setConfig(cfg);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    ASSERT_TRUE(acc.endAccess().has_value());
}

TEST(TxAccessor, SetConfigAppliedOutsideAccess)
{
    StubIBus bus;
    m5::hal::v1::i2s::AccessConfig cfg;
    cfg.sample_rate_hz = 44100;
    m5::hal::v1::i2s::TxAccessor acc{bus, cfg};

    m5::hal::v1::i2s::AccessConfig new_cfg;
    new_cfg.sample_rate_hz = 48000;
    ASSERT_TRUE(acc.setConfig(new_cfg).has_value());

    // write so the cfg is forwarded to the stub
    const uint8_t dummy[] = {0};
    acc.write(dummy, sizeof(dummy));
    EXPECT_EQ(bus.last_cfg.sample_rate_hz, 48000u);
}

// -------------------------------------------------------------------------
// Lock ownership: accessor passes itself as owner to the bus
// -------------------------------------------------------------------------
TEST(TxAccessor, WritePassesAccessorAsOwner)
{
    StubIBus bus;
    m5::hal::v1::i2s::TxAccessor acc{bus, {}};

    const uint8_t d[] = {0xFF};
    acc.write(d, sizeof(d));
    EXPECT_EQ(bus.last_owner, &acc);
}

// -------------------------------------------------------------------------
// Bus contention: second accessor while first holds access
// -------------------------------------------------------------------------
TEST(IBus, SecondAccessorTimesOutWhileFirstHoldsLock)
{
    StubIBus bus;
    m5::hal::v1::i2s::TxAccessor acc1{bus, {}};
    m5::hal::v1::i2s::TxAccessor acc2{bus, {}};

    ASSERT_TRUE(acc1.beginAccess(0).has_value());

    auto result = acc2.beginAccess(0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v1::error::error_t::TIMEOUT_ERROR);

    ASSERT_TRUE(acc1.endAccess().has_value());
}

// -------------------------------------------------------------------------
// getBus returns the same bus object
// -------------------------------------------------------------------------
TEST(TxAccessor, GetIBusReturnsSameBus)
{
    StubIBus bus;
    m5::hal::v1::i2s::TxAccessor acc{bus, {}};
    EXPECT_EQ(&acc.getBus(), &bus);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
