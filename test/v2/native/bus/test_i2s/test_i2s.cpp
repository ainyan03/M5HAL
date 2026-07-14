// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <m5_hal/variants/frameworks/espidf/hal/i2s/i2s.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <cstring>
#include <vector>

namespace {

#define ASSERT_RESULT_OK(expr)                                                                           \
    do {                                                                                                 \
        auto result_ok = (expr);                                                                         \
        ASSERT_TRUE(result_ok.has_value()) << "err=" << m5::hal::v2::error::toString(result_ok.error()); \
    } while (false)

TEST(I2sEspIdfPcmPrimitive, CollapsesHwV2PairsToLeftSlot)
{
    const uint8_t physical[] = {0x10, 0x11, 0x20, 0x21, 0x30, 0x31, 0x40, 0x41};
    uint8_t logical[4]       = {};

    const size_t written = m5::hal::v2::i2s::detail_espidf_i2s::collapseStereo16RxPairsToMonoLeft(
        logical, physical, 2, /*hw_version_1=*/false);

    EXPECT_EQ(written, sizeof(logical));
    const uint8_t expected[] = {0x10, 0x11, 0x30, 0x31};
    EXPECT_EQ(0, std::memcmp(logical, expected, sizeof(expected)));
}

TEST(I2sEspIdfPcmPrimitive, CollapsesTransposedHwV1PairsToPhysicalLeftSlot)
{
    // HW v1 DMA order is R,L within each 32-bit word.
    const uint8_t physical[] = {0x20, 0x21, 0x10, 0x11, 0x40, 0x41, 0x30, 0x31};
    uint8_t logical[4]       = {};

    const size_t written = m5::hal::v2::i2s::detail_espidf_i2s::collapseStereo16RxPairsToMonoLeft(
        logical, physical, 2, /*hw_version_1=*/true);

    EXPECT_EQ(written, sizeof(logical));
    const uint8_t expected[] = {0x10, 0x11, 0x30, 0x31};
    EXPECT_EQ(0, std::memcmp(logical, expected, sizeof(expected)));
}

TEST(I2sEspIdfPcmPrimitive, ZeroFramesLeaveDestinationUntouched)
{
    const uint8_t physical[] = {0x10, 0x11, 0x20, 0x21};
    uint8_t logical[]        = {0xA5, 0x5A};

    const size_t written = m5::hal::v2::i2s::detail_espidf_i2s::collapseStereo16RxPairsToMonoLeft(
        logical, physical, 0, /*hw_version_1=*/false);

    EXPECT_EQ(written, 0u);
    EXPECT_EQ(logical[0], 0xA5);
    EXPECT_EQ(logical[1], 0x5A);
}

// -------------------------------------------------------------------------
// Stub I2S bus that records all calls for inspection.
// -------------------------------------------------------------------------
class StubIBus : public m5::hal::v2::i2s::IBus {
public:
    // Typed init: the fake adds no fields, so it takes the
    // abstract kind config.
    m5::hal::v2::result_t<void> init(const m5::hal::v2::i2s::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    m5::hal::v2::result_t<void> release(void) override
    {
        tx_recorded.clear();
        rx_queue.clear();
        return {};
    }

    m5::hal::v2::result_t<size_t> write(m5::hal::v2::bus::IAccessor* owner, const m5::hal::v2::i2s::AccessConfig& cfg,
                                        m5::hal::v2::data::Source* tx, size_t len) override
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
        call_order.push_back('w');
        return done;
    }

    m5::hal::v2::result_t<size_t> writableBytes(m5::hal::v2::bus::IAccessor* owner,
                                                const m5::hal::v2::i2s::AccessConfig& cfg) override
    {
        last_owner = owner;
        last_cfg   = cfg;
        return stub_writable;
    }

    m5::hal::v2::result_t<size_t> read(m5::hal::v2::bus::IAccessor* owner, const m5::hal::v2::i2s::AccessConfig& cfg,
                                       m5::hal::v2::data::Sink* rx, size_t len) override
    {
        last_owner  = owner;
        last_cfg    = cfg;
        size_t done = 0;
        while (rx != nullptr && !rx->closed() && done < len && !rx_queue.empty()) {
            auto span = rx->reserve(len - done);
            if (!span.has_value()) {
                return m5::stl::make_unexpected(span.error());
            }
            if (span.value().size == 0) {
                break;
            }
            size_t count = span.value().size;
            if (count > rx_queue.size()) {
                count = rx_queue.size();
            }
            for (size_t i = 0; i < count; ++i) {
                span.value().data[i] = rx_queue[i];
            }
            rx_queue.erase(rx_queue.begin(),
                           rx_queue.begin() + static_cast<std::vector<uint8_t>::difference_type>(count));
            auto committed = rx->commit(count);
            if (!committed.has_value()) {
                return m5::stl::make_unexpected(committed.error());
            }
            done += count;
        }
        call_order.push_back('r');
        return done;
    }

    m5::hal::v2::result_t<size_t> readableBytes(m5::hal::v2::bus::IAccessor* owner,
                                                const m5::hal::v2::i2s::AccessConfig& cfg) override
    {
        last_owner = owner;
        last_cfg   = cfg;
        return rx_queue.size();
    }

    std::vector<uint8_t> tx_recorded;
    std::vector<uint8_t> rx_queue;
    std::vector<char> call_order;
    m5::hal::v2::bus::IAccessor* last_owner = nullptr;
    m5::hal::v2::i2s::AccessConfig last_cfg;
    size_t stub_writable = 1024;
};

}  // namespace

// -------------------------------------------------------------------------
// IBusConfig defaults
// -------------------------------------------------------------------------
TEST(IBusConfig, DefaultCtorSetsI2SKind)
{
    m5::hal::v2::i2s::IBusConfig cfg;
    EXPECT_EQ(cfg.getBusKind(), m5::hal::v2::types::bus_kind_t::I2S);
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
    m5::hal::v2::i2s::IBus bus;
    m5::hal::v2::i2s::AccessConfig cfg;
    auto result = bus.write(nullptr, cfg, nullptr, 0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v2::error::error_t::NOT_IMPLEMENTED);
}

TEST(IBus, BaseDefaultWritableBytesReturnsNotImplemented)
{
    m5::hal::v2::i2s::IBus bus;
    m5::hal::v2::i2s::AccessConfig cfg;
    auto result = bus.writableBytes(nullptr, cfg);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v2::error::error_t::NOT_IMPLEMENTED);
}

// -------------------------------------------------------------------------
// TxAccessor forwards writes to bus
// -------------------------------------------------------------------------
TEST(TxAccessor, WriteConstDataSpanForwardsTobus)
{
    StubIBus bus;
    m5::hal::v2::i2s::AccessConfig cfg;
    cfg.sample_rate_hz = 44100;
    m5::hal::v2::i2s::TxAccessor acc{bus, cfg};

    const uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    auto result             = acc.write(m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)});
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
    m5::hal::v2::i2s::AccessConfig cfg;
    m5::hal::v2::i2s::TxAccessor acc{bus, cfg};

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
    m5::hal::v2::i2s::AccessConfig cfg;
    m5::hal::v2::i2s::TxAccessor acc{bus, cfg};

    const uint8_t payload[] = {0x10, 0x20, 0x30};
    m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};
    auto result = acc.write(src, sizeof(payload));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(payload));
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    EXPECT_EQ(bus.tx_recorded[1], 0x20u);
}

TEST(TxAccessor, TransactionTotalsAccumulateWrites)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};

    ASSERT_RESULT_OK(tx.beginTransaction());
    const uint8_t a[] = {0x01, 0x02};
    const uint8_t b[] = {0x03, 0x04, 0x05};
    auto wa           = tx.write(a, sizeof(a));
    ASSERT_TRUE(wa.has_value()) << "err=" << m5::hal::v2::error::toString(wa.error());
    auto wb = tx.write(b, sizeof(b));
    ASSERT_TRUE(wb.has_value()) << "err=" << m5::hal::v2::error::toString(wb.error());

    auto ended = tx.endTransaction();
    ASSERT_TRUE(ended.has_value()) << "err=" << m5::hal::v2::error::toString(ended.error());
    EXPECT_EQ(ended->tx, sizeof(a) + sizeof(b));
    EXPECT_EQ(ended->rx, 0u);
}

TEST(TxAccessor, NestedTransactionReturnsCurrentTotalsAndKeepsAccessUntilOuterEnd)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};
    m5::hal::v2::i2s::TxAccessor other{bus, {}};

    ASSERT_RESULT_OK(tx.beginTransaction());
    ASSERT_RESULT_OK(tx.beginTransaction());
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    auto written            = tx.write(payload, sizeof(payload));
    ASSERT_TRUE(written.has_value()) << "err=" << m5::hal::v2::error::toString(written.error());

    auto inner = tx.endTransaction();
    ASSERT_TRUE(inner.has_value()) << "err=" << m5::hal::v2::error::toString(inner.error());
    EXPECT_EQ(inner->tx, sizeof(payload));
    EXPECT_TRUE(tx.inTransaction());
    EXPECT_TRUE(tx.inAccess());

    auto blocked = other.beginTransaction(0);
    ASSERT_FALSE(blocked.has_value());
    EXPECT_EQ(blocked.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);

    auto outer = tx.endTransaction();
    ASSERT_TRUE(outer.has_value()) << "err=" << m5::hal::v2::error::toString(outer.error());
    EXPECT_EQ(outer->tx, sizeof(payload));
    EXPECT_FALSE(tx.inTransaction());
    EXPECT_FALSE(tx.inAccess());

    ASSERT_RESULT_OK(other.beginTransaction(0));
    auto other_end = other.endTransaction();
    ASSERT_TRUE(other_end.has_value()) << "err=" << m5::hal::v2::error::toString(other_end.error());
}

TEST(TxAccessor, TransactionExcludesOtherAccessorOnSameChannel)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};
    m5::hal::v2::i2s::TxAccessor other{bus, {}};

    ASSERT_RESULT_OK(tx.beginTransaction());
    auto blocked = other.beginTransaction(0);
    ASSERT_FALSE(blocked.has_value());
    EXPECT_EQ(blocked.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    ASSERT_RESULT_OK(tx.endTransaction());

    ASSERT_RESULT_OK(other.beginTransaction(0));
    ASSERT_RESULT_OK(other.endTransaction());
}

TEST(TxAccessor, StandaloneWriteSugarStillWorks)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};

    const uint8_t payload[] = {0xAA, 0xBB};
    auto result             = tx.write(payload, sizeof(payload));
    ASSERT_TRUE(result.has_value()) << "err=" << m5::hal::v2::error::toString(result.error());
    EXPECT_EQ(result.value(), sizeof(payload));
    EXPECT_FALSE(tx.inTransaction());
    EXPECT_FALSE(tx.inAccess());
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
}

TEST(TxAccessor, EndTransactionOutsideTransactionIsInvalidState)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};

    auto ended = tx.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), m5::hal::v2::error::error_t::INVALID_STATE);
}

TEST(RxAccessor, TransactionTotalsAccumulateReads)
{
    StubIBus bus;
    bus.rx_queue = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4};
    m5::hal::v2::i2s::RxAccessor rx{bus, {}};

    ASSERT_RESULT_OK(rx.beginTransaction());
    uint8_t first[2]  = {};
    uint8_t second[3] = {};
    auto ra           = rx.read(first, sizeof(first));
    ASSERT_TRUE(ra.has_value()) << "err=" << m5::hal::v2::error::toString(ra.error());
    auto rb = rx.read(second, sizeof(second));
    ASSERT_TRUE(rb.has_value()) << "err=" << m5::hal::v2::error::toString(rb.error());

    auto ended = rx.endTransaction();
    ASSERT_TRUE(ended.has_value()) << "err=" << m5::hal::v2::error::toString(ended.error());
    EXPECT_EQ(ended->tx, 0u);
    EXPECT_EQ(ended->rx, sizeof(first) + sizeof(second));
    EXPECT_EQ(first[0], 0xA0);
    EXPECT_EQ(second[2], 0xA4);
}

TEST(RxAccessor, StandaloneReadSugarStillWorks)
{
    StubIBus bus;
    bus.rx_queue = {0x55, 0x66};
    m5::hal::v2::i2s::RxAccessor rx{bus, {}};

    uint8_t dst[2] = {};
    auto result    = rx.read(dst, sizeof(dst));
    ASSERT_TRUE(result.has_value()) << "err=" << m5::hal::v2::error::toString(result.error());
    EXPECT_EQ(result.value(), sizeof(dst));
    EXPECT_EQ(dst[0], 0x55);
    EXPECT_EQ(dst[1], 0x66);
    EXPECT_FALSE(rx.inTransaction());
    EXPECT_FALSE(rx.inAccess());
}

TEST(RxAccessor, EndTransactionOutsideTransactionIsInvalidState)
{
    StubIBus bus;
    m5::hal::v2::i2s::RxAccessor rx{bus, {}};

    auto ended = rx.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), m5::hal::v2::error::error_t::INVALID_STATE);
}

TEST(IBus, DefaultTransferComposesWriteThenRead)
{
    StubIBus bus;
    bus.rx_queue = {0x41, 0x42, 0x43, 0x44};
    m5::hal::v2::i2s::AccessConfig cfg;
    cfg.sample_rate_hz = 22050;
    m5::hal::v2::i2s::Accessor dev{bus, cfg};

    const uint8_t tx[] = {0x90, 0x91};
    uint8_t rx[4]      = {};
    auto result =
        dev.transfer(m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)}, m5::hal::v2::data::DataSpan{rx, sizeof(rx)});

    ASSERT_TRUE(result.has_value()) << "err=" << m5::hal::v2::error::toString(result.error());
    EXPECT_EQ(result->tx, sizeof(tx));
    EXPECT_EQ(result->rx, sizeof(rx));
    ASSERT_EQ(bus.call_order.size(), 2u);
    EXPECT_EQ(bus.call_order[0], 'w');
    EXPECT_EQ(bus.call_order[1], 'r');
    EXPECT_EQ(bus.tx_recorded, std::vector<uint8_t>(tx, tx + sizeof(tx)));
    EXPECT_EQ(rx[0], 0x41);
    EXPECT_EQ(rx[3], 0x44);
    EXPECT_EQ(bus.last_cfg.sample_rate_hz, 22050u);
}

// All three write overloads must produce the same byte sequence.
TEST(TxAccessor, WriteOverloadsAreEquivalent)
{
    const uint8_t payload[] = {0xDE, 0xAD};

    // via ConstDataSpan
    {
        StubIBus bus;
        m5::hal::v2::i2s::TxAccessor acc{bus, {}};
        acc.write(m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)});
        ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
        EXPECT_EQ(bus.tx_recorded[0], 0xDEu);
    }
    // via raw pointer
    {
        StubIBus bus;
        m5::hal::v2::i2s::TxAccessor acc{bus, {}};
        acc.write(payload, sizeof(payload));
        ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
        EXPECT_EQ(bus.tx_recorded[0], 0xDEu);
    }
    // via Source
    {
        StubIBus bus;
        m5::hal::v2::i2s::TxAccessor acc{bus, {}};
        m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};
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
    m5::hal::v2::i2s::TxAccessor acc{bus, {}};

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
    m5::hal::v2::i2s::AccessConfig cfg;
    m5::hal::v2::i2s::TxAccessor acc{bus, cfg};

    // Acquire the lock explicitly so inAccess() is true.
    ASSERT_TRUE(acc.beginAccess(0).has_value());
    cfg.sample_rate_hz = 48000;
    auto result        = acc.setConfig(cfg);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    ASSERT_TRUE(acc.endAccess().has_value());
}

TEST(TxAccessor, SetConfigAppliedOutsideAccess)
{
    StubIBus bus;
    m5::hal::v2::i2s::AccessConfig cfg;
    cfg.sample_rate_hz = 44100;
    m5::hal::v2::i2s::TxAccessor acc{bus, cfg};

    m5::hal::v2::i2s::AccessConfig new_cfg;
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
    m5::hal::v2::i2s::TxAccessor acc{bus, {}};

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
    m5::hal::v2::i2s::TxAccessor acc1{bus, {}};
    m5::hal::v2::i2s::TxAccessor acc2{bus, {}};

    ASSERT_TRUE(acc1.beginAccess(0).has_value());

    auto result = acc2.beginAccess(0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);

    ASSERT_TRUE(acc1.endAccess().has_value());
}

// -------------------------------------------------------------------------
// getBus returns the same bus object
// -------------------------------------------------------------------------
TEST(TxAccessor, GetIBusReturnsSameBus)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor acc{bus, {}};
    EXPECT_EQ(&acc.getBus(), &bus);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
