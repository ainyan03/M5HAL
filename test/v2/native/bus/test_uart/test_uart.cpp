// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <algorithm>
#include <cstring>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

#define ASSERT_RESULT_OK(expr)                                                                           \
    do {                                                                                                 \
        auto result_ok = (expr);                                                                         \
        ASSERT_TRUE(result_ok.has_value()) << "err=" << m5::hal::v2::error::toString(result_ok.error()); \
    } while (false)

class RecordingIBus : public m5::hal::v2::uart::IBus {
public:
    // Typed init: the fake adds no fields, so it takes the
    // abstract kind config.
    m5::hal::v2::result_t<void> init(const m5::hal::v2::uart::IBusConfig& config)
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

    m5::hal::v2::result_t<size_t> write(m5::hal::v2::bus::IAccessor* owner, const m5::hal::v2::uart::AccessConfig& cfg,
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

    m5::hal::v2::result_t<size_t> read(m5::hal::v2::bus::IAccessor* owner, const m5::hal::v2::uart::AccessConfig& cfg,
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
                                                const m5::hal::v2::uart::AccessConfig& cfg) override
    {
        last_owner = owner;
        last_cfg   = cfg;
        return rx_queue.size();
    }

    std::vector<uint8_t> tx_recorded;
    std::vector<uint8_t> rx_queue;
    std::vector<char> call_order;
    m5::hal::v2::bus::IAccessor* last_owner = nullptr;
    m5::hal::v2::uart::AccessConfig last_cfg;
};

class ScriptedStreamingBus : public m5::hal::v2::uart::Bus_streaming {
public:
    static m5::hal::v2::result_t<size_t> finishWrite(m5::hal::v2::result_t<size_t> accepted,
                                                     m5::hal::v2::error::error_t completion_error)
    {
        return completeWrite(std::move(accepted), completion_error);
    }

    bool fail_first                             = false;
    size_t first_count                          = 3;
    m5::hal::v2::error::error_t following_error = m5::hal::v2::error::error_t::TIMEOUT_ERROR;
    size_t write_calls                          = 0;

protected:
    m5::hal::v2::result_t<size_t> rawWrite(const uint8_t*, size_t len, uint32_t) override
    {
        ++write_calls;
        if (fail_first || write_calls > 1) {
            return m5::stl::make_unexpected(following_error);
        }
        return std::min(first_count, len);
    }

    m5::hal::v2::result_t<size_t> rawRead(uint8_t*, size_t, uint32_t) override
    {
        return static_cast<size_t>(0);
    }

    m5::hal::v2::result_t<size_t> rawReadableBytes() override
    {
        return static_cast<size_t>(0);
    }
};

}  // namespace

TEST(BusStreamingWrite, ErrorBeforeProgressPropagatesAndKeepsSource)
{
    ScriptedStreamingBus bus;
    bus.fail_first          = true;
    const uint8_t payload[] = {1, 2, 3, 4, 5, 6};
    m5::hal::v2::data::MemorySource src{payload, sizeof(payload)};

    auto result = bus.write(nullptr, {}, &src, sizeof(payload));
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    auto remaining = src.peek(sizeof(payload));
    ASSERT_TRUE(remaining.has_value());
    EXPECT_EQ(remaining.value().size, sizeof(payload));
}

TEST(BusStreamingWrite, ErrorAfterProgressReturnsAcceptedPrefix)
{
    ScriptedStreamingBus bus;
    const uint8_t payload[] = {1, 2, 3, 4, 5, 6};
    m5::hal::v2::data::MemorySource src{payload, sizeof(payload)};

    auto result = bus.write(nullptr, {}, &src, sizeof(payload));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 3u);
    EXPECT_EQ(bus.write_calls, 2u);
    auto remaining = src.peek(sizeof(payload));
    ASSERT_TRUE(remaining.has_value());
    ASSERT_EQ(remaining.value().size, 3u);
    EXPECT_EQ(remaining.value().data[0], 4u);
}

TEST(BusStreamingWrite, CompletionErrorUsesAcceptedPrefixAsRetryBoundary)
{
    auto partial = ScriptedStreamingBus::finishWrite(m5::hal::v2::result_t<size_t>{3u},
                                                     m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    ASSERT_TRUE(partial.has_value());
    EXPECT_EQ(partial.value(), 3u);

    auto none = ScriptedStreamingBus::finishWrite(m5::hal::v2::result_t<size_t>{0u},
                                                  m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    ASSERT_FALSE(none.has_value());
    EXPECT_EQ(none.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
}

TEST(IBusConfig, DefaultCtorSetsUARTKind)
{
    m5::hal::v2::uart::IBusConfig cfg;
    EXPECT_EQ(cfg.getBusKind(), m5::hal::v2::types::bus_kind_t::UART);
    EXPECT_EQ(cfg.pin_tx, -1);
    EXPECT_EQ(cfg.pin_rx, -1);
    EXPECT_EQ(cfg.rx_buffer_size, 256u);
    EXPECT_EQ(cfg.tx_buffer_size, 0u);
}

// Tag-pin constructors. The construction contract is compile-time;
// the load-bearing parts are pinned with static_asserts.
static_assert(std::is_constructible<m5::hal::v2::uart::IBusConfig, m5::hal::v2::uart::Tx, m5::hal::v2::uart::Rx>::value,
              "tag ctor (Tx, Rx)");
static_assert(std::is_constructible<m5::hal::v2::uart::IBusConfig, m5::hal::v2::uart::Rx, m5::hal::v2::uart::Tx>::value,
              "tag ctor (Rx, Tx)");
static_assert(!std::is_constructible<m5::hal::v2::uart::IBusConfig, int, int>::value, "no untagged positional ctor");
static_assert(!std::is_convertible<m5::hal::v2::types::gpio_number_t, m5::hal::v2::uart::Tx>::value,
              "tags take no implicit integer");

TEST(IBusConfig, TagCtorEitherOrderLandsOnTheRightField)
{
    constexpr m5::hal::v2::uart::IBusConfig a{m5::hal::v2::uart::Tx{17}, m5::hal::v2::uart::Rx{16}};
    constexpr m5::hal::v2::uart::IBusConfig b{m5::hal::v2::uart::Rx{16}, m5::hal::v2::uart::Tx{17}};
    EXPECT_EQ(a.pin_tx, 17);
    EXPECT_EQ(a.pin_rx, 16);
    EXPECT_EQ(b.pin_tx, 17);
    EXPECT_EQ(b.pin_rx, 16);
}

TEST(IBusConfig, TagCtorKeepsTheKindAndTheOtherDefaults)
{
    constexpr m5::hal::v2::uart::IBusConfig cfg{m5::hal::v2::uart::Tx{17}, m5::hal::v2::uart::Rx{16}};
    EXPECT_EQ(cfg.getBusKind(), m5::hal::v2::types::bus_kind_t::UART);
    EXPECT_EQ(cfg.pin_rts, -1);
    EXPECT_EQ(cfg.pin_cts, -1);
    EXPECT_EQ(cfg.rx_buffer_size, 256u);
    EXPECT_EQ(cfg.tx_buffer_size, 0u);
}

TEST(Accessor, WriteConsumesSource)
{
    RecordingIBus bus;
    m5::hal::v2::uart::AccessConfig cfg;
    cfg.baud_rate = 921600;
    m5::hal::v2::uart::Accessor dev{bus, cfg};

    const uint8_t payload[] = {0x11, 0x22, 0x33};
    auto result             = dev.write(payload, sizeof(payload));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(payload));
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    EXPECT_EQ(bus.tx_recorded[0], 0x11);
    EXPECT_EQ(bus.tx_recorded[2], 0x33);
    EXPECT_EQ(bus.last_owner, &dev.tx());
    EXPECT_EQ(bus.last_cfg.baud_rate, 921600u);
}

TEST(TxAccessor, TransactionTotalsAccumulateWrites)
{
    RecordingIBus bus;
    m5::hal::v2::uart::TxAccessor tx{bus, {}};

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
    RecordingIBus bus;
    m5::hal::v2::uart::TxAccessor tx{bus, {}};
    m5::hal::v2::uart::TxAccessor other{bus, {}};

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
    RecordingIBus bus;
    m5::hal::v2::uart::TxAccessor tx{bus, {}};
    m5::hal::v2::uart::TxAccessor other{bus, {}};

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
    RecordingIBus bus;
    m5::hal::v2::uart::TxAccessor tx{bus, {}};

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
    RecordingIBus bus;
    m5::hal::v2::uart::TxAccessor tx{bus, {}};

    auto ended = tx.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), m5::hal::v2::error::error_t::INVALID_STATE);
}

TEST(RxAccessor, ReadUntilIncludesTheDelimiter)
{
    RecordingIBus bus;
    bus.rx_queue = {'O', 'K', '\r', '\n', 'X'};  // 'X' belongs to the next line
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::RxAccessor rx{bus, cfg};

    uint8_t line[16] = {};
    auto r           = rx.readUntil('\n', line, sizeof(line));
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r.value(), 4u);
    EXPECT_EQ(line[3], '\n');  // delimiter included: complete-line test works
    EXPECT_EQ(std::memcmp(line, "OK\r\n", 4), 0);
    EXPECT_EQ(bus.rx_queue.size(), 1u);  // the next line's byte was not consumed
}

TEST(RxAccessor, TransactionTotalsAccumulateReads)
{
    RecordingIBus bus;
    bus.rx_queue = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4};
    m5::hal::v2::uart::RxAccessor rx{bus, {}};

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
    RecordingIBus bus;
    bus.rx_queue = {0x55, 0x66};
    m5::hal::v2::uart::RxAccessor rx{bus, {}};

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
    RecordingIBus bus;
    m5::hal::v2::uart::RxAccessor rx{bus, {}};

    auto ended = rx.endTransaction();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), m5::hal::v2::error::error_t::INVALID_STATE);
}

TEST(RxAccessor, ReadUntilReturnsThePartialLineOnTimeout)
{
    RecordingIBus bus;
    bus.rx_queue = {'$', 'G', 'P'};  // no delimiter arrives
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::RxAccessor rx{bus, cfg};

    uint8_t line[16] = {};
    auto r           = rx.readUntil('\n', line, sizeof(line));
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r.value(), 3u);
    EXPECT_NE(line[2], '\n');  // partial: the last byte is not the delimiter
}

TEST(RxAccessor, ReadUntilStopsAtTheBufferBound)
{
    RecordingIBus bus;
    bus.rx_queue = {'1', '2', '3', '4', '\n'};
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::RxAccessor rx{bus, cfg};

    uint8_t line[3] = {};
    auto r          = rx.readUntil('\n', line, sizeof(line));
    ASSERT_TRUE(r.has_value());
    ASSERT_EQ(r.value(), 3u);
    EXPECT_NE(line[2], '\n');  // full buffer, still no delimiter = partial
}

TEST(Accessor, ReadUntilForwardsToTheRxChannel)
{
    RecordingIBus bus;
    bus.rx_queue = {'A', 'T', '\n'};
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::Accessor dev{bus, cfg};

    uint8_t line[8] = {};
    auto r          = dev.readUntil('\n', line, sizeof(line));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 3u);
    EXPECT_EQ(bus.last_owner, &dev.rx());
}

TEST(Accessor, ReadFillsSink)
{
    RecordingIBus bus;
    bus.rx_queue = {0xA0, 0xA1, 0xA2};
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::Accessor dev{bus, cfg};

    uint8_t dst[2] = {};
    auto result    = dev.read(dst, sizeof(dst));
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), sizeof(dst));
    EXPECT_EQ(dst[0], 0xA0);
    EXPECT_EQ(dst[1], 0xA1);
    ASSERT_EQ(bus.rx_queue.size(), 1u);
    EXPECT_EQ(bus.rx_queue[0], 0xA2);
}

TEST(IBus, DefaultTransferComposesWriteThenRead)
{
    RecordingIBus bus;
    bus.rx_queue = {0xB0, 0xB1, 0xB2};
    m5::hal::v2::uart::AccessConfig cfg;
    cfg.baud_rate = 460800;
    m5::hal::v2::uart::Accessor dev{bus, cfg};

    const uint8_t tx[] = {0x10, 0x11, 0x12, 0x13};
    uint8_t rx[3]      = {};
    auto result =
        dev.transfer(m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)}, m5::hal::v2::data::DataSpan{rx, sizeof(rx)});

    ASSERT_TRUE(result.has_value()) << "err=" << m5::hal::v2::error::toString(result.error());
    EXPECT_EQ(result->tx, sizeof(tx));
    EXPECT_EQ(result->rx, sizeof(rx));
    ASSERT_EQ(bus.call_order.size(), 2u);
    EXPECT_EQ(bus.call_order[0], 'w');
    EXPECT_EQ(bus.call_order[1], 'r');
    EXPECT_EQ(bus.tx_recorded, std::vector<uint8_t>(tx, tx + sizeof(tx)));
    EXPECT_EQ(rx[0], 0xB0);
    EXPECT_EQ(rx[2], 0xB2);
    EXPECT_EQ(bus.last_cfg.baud_rate, 460800u);
}

TEST(Accessor, ReadableBytesUsesBus)
{
    RecordingIBus bus;
    bus.rx_queue = {1, 2, 3, 4};
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::Accessor dev{bus, cfg};

    auto result = dev.readableBytes();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result.value(), 4u);
}

TEST(Accessor, SetConfigRejectsInsideAccess)
{
    RecordingIBus bus;
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::Accessor dev{bus, cfg};

    auto locked = dev.beginAccess(0);
    ASSERT_TRUE(locked.has_value());
    cfg.baud_rate = 57600;
    auto result   = dev.setConfig(cfg);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    ASSERT_TRUE(dev.endAccess().has_value());
}

TEST(UARTTxRxAccessor, SplitAccessorsUseSeparateChannels)
{
    RecordingIBus bus;
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::TxAccessor tx{bus, cfg};
    m5::hal::v2::uart::RxAccessor rx{bus, cfg};

    ASSERT_TRUE(tx.beginAccess(0).has_value());
    ASSERT_TRUE(rx.beginAccess(0).has_value());
    EXPECT_TRUE(tx.inAccess());
    EXPECT_TRUE(rx.inAccess());

    ASSERT_TRUE(rx.endAccess().has_value());
    ASSERT_TRUE(tx.endAccess().has_value());
}

TEST(IBus, SameChannelContentionTimesOut)
{
    RecordingIBus bus;
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::TxAccessor tx1{bus, cfg};
    m5::hal::v2::uart::TxAccessor tx2{bus, cfg};
    m5::hal::v2::uart::RxAccessor rx{bus, cfg};

    ASSERT_TRUE(tx1.beginAccess(0).has_value());
    auto tx2_result = tx2.beginAccess(0);
    ASSERT_FALSE(tx2_result.has_value());
    EXPECT_EQ(tx2_result.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);

    auto rx_result = rx.beginAccess(0);
    ASSERT_TRUE(rx_result.has_value());
    ASSERT_TRUE(rx.endAccess().has_value());
    ASSERT_TRUE(tx1.endAccess().has_value());
}

TEST(Accessor, FacadeSessionLocksBothChannels)
{
    RecordingIBus bus;
    m5::hal::v2::uart::AccessConfig cfg;
    m5::hal::v2::uart::Accessor dev{bus, cfg};
    m5::hal::v2::uart::TxAccessor other_tx{bus, cfg};
    m5::hal::v2::uart::RxAccessor other_rx{bus, cfg};

    ASSERT_TRUE(dev.beginAccess(0).has_value());
    EXPECT_TRUE(dev.inAccess());

    auto tx_result = other_tx.beginAccess(0);
    ASSERT_FALSE(tx_result.has_value());
    EXPECT_EQ(tx_result.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);

    auto rx_result = other_rx.beginAccess(0);
    ASSERT_FALSE(rx_result.has_value());
    EXPECT_EQ(rx_result.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);

    ASSERT_TRUE(dev.endAccess().has_value());
    EXPECT_FALSE(dev.inAccess());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
