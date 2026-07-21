// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <thread>
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
    // The fake uses the same portable kind config as production providers.
    m5::hal::v2::result_t<void> init(const m5::hal::v2::uart::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    m5::hal::v2::result_t<void> lockFor(m5::hal::v2::bus::IAccessor& owner, m5::hal::v2::uart::Channel channel)
    {
        return lockChannel(owner, channel, 0);
    }

    m5::hal::v2::result_t<void> unlockFor(m5::hal::v2::bus::IAccessor& owner, m5::hal::v2::uart::Channel channel)
    {
        return unlockChannel(owner, channel);
    }

protected:
    m5::hal::v2::bus::CloseOutcome closeBackend(void) override
    {
        tx_recorded.clear();
        rx_queue.clear();
        return m5::hal::v2::bus::CloseOutcome::success();
    }

public:
    m5::hal::v2::result_t<void> beginOperationBackend(
        m5::hal::v2::bus::OperationContext<m5::hal::v2::uart::AccessConfig>& context) override
    {
        const bool tx = context.runtime.mode == m5::hal::v2::bus::OperationMode::Tx;
        lifecycle_order.push_back(tx ? 'T' : 'R');
        if ((tx && fail_begin_tx) || (!tx && fail_begin_rx)) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::BUSY);
        }
        tx ? ++active_tx : ++active_rx;
        return {};
    }

    m5::hal::v2::result_t<void> endOperationBackend(
        m5::hal::v2::bus::OperationContext<m5::hal::v2::uart::AccessConfig>& context) override
    {
        const bool tx = context.runtime.mode == m5::hal::v2::bus::OperationMode::Tx;
        lifecycle_order.push_back(tx ? 't' : 'r');
        tx ? --active_tx : --active_rx;
        return {};
    }

    m5::hal::v2::result_t<size_t> writeBackend(
        m5::hal::v2::bus::OperationContext<m5::hal::v2::uart::AccessConfig>& context, m5::hal::v2::data::Source* tx,
        size_t len) override
    {
        last_owner  = operationOwner(context);
        last_cfg    = context.config;
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

    m5::hal::v2::result_t<size_t> readBackend(
        m5::hal::v2::bus::OperationContext<m5::hal::v2::uart::AccessConfig>& context, m5::hal::v2::data::Sink* rx,
        size_t len) override
    {
        last_owner  = operationOwner(context);
        last_cfg    = context.config;
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

    m5::hal::v2::result_t<size_t> readableBytesBackend(
        m5::hal::v2::bus::OperationContext<m5::hal::v2::uart::AccessConfig>& context) override
    {
        last_owner = operationOwner(context);
        last_cfg   = context.config;
        ++readable_calls;
        return rx_queue.size();
    }

    std::vector<uint8_t> tx_recorded;
    std::vector<uint8_t> rx_queue;
    std::vector<char> call_order;
    std::vector<char> lifecycle_order;
    m5::hal::v2::bus::IAccessor* last_owner = nullptr;
    m5::hal::v2::uart::AccessConfig last_cfg;
    int active_tx         = 0;
    int active_rx         = 0;
    size_t readable_calls = 0;
    bool fail_begin_tx    = false;
    bool fail_begin_rx    = false;
};

class InspectableUartAccessor : public m5::hal::v2::bus::IAccessor {
public:
    InspectableUartAccessor(RecordingIBus& bus, const m5::hal::v2::uart::AccessConfig& config = {})
        : m5::hal::v2::bus::IAccessor{bus}, _typed_bus{bus}, _context{makeOperationContext(config)}
    {
    }

    const m5::hal::v2::uart::AccessConfig& getConfig() const override
    {
        return _context.config;
    }

    m5::hal::v2::bus::OperationContext<m5::hal::v2::uart::AccessConfig>& context()
    {
        return _context;
    }

    m5::hal::v2::result_t<void> begin(m5::hal::v2::bus::OperationMode mode)
    {
        _channel = mode == m5::hal::v2::bus::OperationMode::Tx ? m5::hal::v2::uart::Channel::Tx
                                                               : m5::hal::v2::uart::Channel::Rx;
        return _beginOperationAccess(
            _context, 0, mode, [&](uint32_t) { return _typed_bus.lockFor(*this, _channel); },
            [&] { return _typed_bus.unlockFor(*this, _channel); },
            [&](auto& context) { return _typed_bus.beginOperation(context); });
    }

    m5::hal::v2::result_t<void> end()
    {
        return _endOperationAccess(
            _context, 0, [&](auto& context) { return _typed_bus.endOperation(context); },
            [&] { return _typed_bus.unlockFor(*this, _channel); });
    }

private:
    RecordingIBus& _typed_bus;
    m5::hal::v2::bus::OperationContext<m5::hal::v2::uart::AccessConfig> _context;
    m5::hal::v2::uart::Channel _channel = m5::hal::v2::uart::Channel::Tx;
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

TEST(UARTCheckedFacade, RejectsInactiveEndedWrongBusAndWrongAccessorContexts)
{
    namespace bus   = m5::hal::v2::bus;
    namespace error = m5::hal::v2::error;
    namespace uart  = m5::hal::v2::uart;

    RecordingIBus first_bus;
    RecordingIBus second_bus;
    InspectableUartAccessor tx{first_bus};
    InspectableUartAccessor rx{first_bus};
    InspectableUartAccessor other{first_bus};

    auto inactive_write = first_bus.write(tx.context(), nullptr, 0);
    ASSERT_FALSE(inactive_write.has_value());
    EXPECT_EQ(inactive_write.error(), error::error_t::INVALID_STATE);
    auto inactive_read = first_bus.read(rx.context(), nullptr, 0);
    ASSERT_FALSE(inactive_read.has_value());
    EXPECT_EQ(inactive_read.error(), error::error_t::INVALID_STATE);

    tx.context().runtime.begin(0, 0, bus::OperationMode::Tx);
    ASSERT_RESULT_OK(first_bus.lockFor(other, uart::Channel::Tx));
    auto wrong_accessor = first_bus.beginOperation(tx.context());
    ASSERT_FALSE(wrong_accessor.has_value());
    EXPECT_EQ(wrong_accessor.error(), error::error_t::INVALID_STATE);
    ASSERT_RESULT_OK(first_bus.unlockFor(other, uart::Channel::Tx));

    ASSERT_RESULT_OK(tx.begin(bus::OperationMode::Tx));
    ASSERT_RESULT_OK(rx.begin(bus::OperationMode::Rx));
    auto wrong_bus_write = second_bus.write(tx.context(), nullptr, 0);
    ASSERT_FALSE(wrong_bus_write.has_value());
    EXPECT_EQ(wrong_bus_write.error(), error::error_t::INVALID_STATE);
    auto wrong_bus_transfer = second_bus.transfer(tx.context(), rx.context(), nullptr, 0, nullptr, 0);
    ASSERT_FALSE(wrong_bus_transfer.has_value());
    EXPECT_EQ(wrong_bus_transfer.error(), error::error_t::INVALID_STATE);
    ASSERT_RESULT_OK(rx.end());
    ASSERT_RESULT_OK(tx.end());

    auto ended_readable = first_bus.readableBytes(rx.context());
    ASSERT_FALSE(ended_readable.has_value());
    EXPECT_EQ(ended_readable.error(), error::error_t::INVALID_STATE);
    EXPECT_TRUE(first_bus.call_order.empty());
    EXPECT_EQ(first_bus.readable_calls, 0u);
}

TEST(UARTCheckedFacade, RejectsCorruptRuntimeAndRecoversBothSlotsAndLocks)
{
    namespace bus   = m5::hal::v2::bus;
    namespace error = m5::hal::v2::error;

    RecordingIBus uart_bus;
    InspectableUartAccessor tx{uart_bus};
    InspectableUartAccessor rx{uart_bus};

    ASSERT_RESULT_OK(tx.begin(bus::OperationMode::Tx));
    const auto registered_generation = tx.context().runtime.generation;
    ++tx.context().runtime.generation;
    auto stale = uart_bus.write(tx.context(), nullptr, 0);
    ASSERT_FALSE(stale.has_value());
    EXPECT_EQ(stale.error(), error::error_t::INVALID_STATE);
    auto stale_end = tx.end();
    ASSERT_FALSE(stale_end.has_value());
    EXPECT_EQ(stale_end.error(), error::error_t::INVALID_STATE);
    EXPECT_FALSE(tx.inAccess());

    ASSERT_RESULT_OK(tx.begin(bus::OperationMode::Tx));
    EXPECT_GT(tx.context().runtime.generation, registered_generation);
    ASSERT_RESULT_OK(rx.begin(bus::OperationMode::Rx));
    rx.context().runtime.mode = bus::OperationMode::Tx;
    auto wrong_mode           = uart_bus.readableBytes(rx.context());
    ASSERT_FALSE(wrong_mode.has_value());
    EXPECT_EQ(wrong_mode.error(), error::error_t::INVALID_STATE);
    auto wrong_mode_end = rx.end();
    ASSERT_FALSE(wrong_mode_end.has_value());
    EXPECT_EQ(wrong_mode_end.error(), error::error_t::INVALID_STATE);
    EXPECT_FALSE(rx.inAccess());
    ASSERT_RESULT_OK(tx.end());

    ASSERT_RESULT_OK(tx.begin(bus::OperationMode::Tx));
    ASSERT_RESULT_OK(rx.begin(bus::OperationMode::Rx));
    auto recovered = uart_bus.transfer(tx.context(), rx.context(), nullptr, 0, nullptr, 0);
    ASSERT_TRUE(recovered.has_value()) << "err=" << error::toString(recovered.error());
    EXPECT_EQ(recovered->tx, 0u);
    EXPECT_EQ(recovered->rx, 0u);
    ASSERT_RESULT_OK(rx.end());
    ASSERT_RESULT_OK(tx.end());
}

TEST(BusStreamingWrite, ErrorBeforeProgressPropagatesAndKeepsSource)
{
    ScriptedStreamingBus bus;
    m5::hal::v2::uart::TxAccessor accessor{bus, {}};
    bus.fail_first          = true;
    const uint8_t payload[] = {1, 2, 3, 4, 5, 6};
    m5::hal::v2::data::MemorySource src{payload, sizeof(payload)};

    auto result = accessor.write(src, sizeof(payload));
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    auto remaining = src.peek(sizeof(payload));
    ASSERT_TRUE(remaining.has_value());
    EXPECT_EQ(remaining.value().size, sizeof(payload));
}

TEST(BusStreamingWrite, ErrorAfterProgressReturnsAcceptedPrefix)
{
    ScriptedStreamingBus bus;
    m5::hal::v2::uart::TxAccessor accessor{bus, {}};
    const uint8_t payload[] = {1, 2, 3, 4, 5, 6};
    m5::hal::v2::data::MemorySource src{payload, sizeof(payload)};

    auto result = accessor.write(src, sizeof(payload));
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

TEST(TxAccessor, ExplicitAccessIsNonNestedAndWritesBorrowIt)
{
    RecordingIBus bus;
    m5::hal::v2::uart::TxAccessor tx{bus, {}};

    ASSERT_RESULT_OK(tx.beginAccess());
    auto nested = tx.beginAccess();
    ASSERT_FALSE(nested.has_value());
    EXPECT_EQ(nested.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    const uint8_t a[] = {0x01, 0x02};
    const uint8_t b[] = {0x03, 0x04, 0x05};
    ASSERT_RESULT_OK(tx.write(a, sizeof(a)));
    ASSERT_RESULT_OK(tx.write(b, sizeof(b)));
    EXPECT_EQ(bus.lifecycle_order, std::vector<char>({'T'}));
    ASSERT_RESULT_OK(tx.endAccess());
    EXPECT_EQ(bus.lifecycle_order, std::vector<char>({'T', 't'}));
    EXPECT_EQ(bus.tx_recorded.size(), sizeof(a) + sizeof(b));
}

TEST(TxAccessor, StandaloneWriteSugarStillWorks)
{
    RecordingIBus bus;
    m5::hal::v2::uart::TxAccessor tx{bus, {}};

    const uint8_t payload[] = {0xAA, 0xBB};
    auto result             = tx.write(payload, sizeof(payload));
    ASSERT_TRUE(result.has_value()) << "err=" << m5::hal::v2::error::toString(result.error());
    EXPECT_EQ(result.value(), sizeof(payload));
    EXPECT_FALSE(tx.inAccess());
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
    EXPECT_EQ(bus.lifecycle_order, std::vector<char>({'T', 't'}));
    auto status = tx.getLastTransferStatus();
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->totals.tx, sizeof(payload));
    EXPECT_EQ(status->completion, m5::hal::v2::bus::CompletionLevel::Complete);
}

TEST(TxAccessor, BeginFailureDoesNotReplacePreviousTransferStatus)
{
    RecordingIBus bus;
    m5::hal::v2::uart::TxAccessor tx{bus, {}};
    const uint8_t payload[] = {0xAA};
    ASSERT_TRUE(tx.write(payload, sizeof(payload)).has_value());
    auto before = tx.getLastTransferStatus();
    ASSERT_TRUE(before.has_value());

    bus.fail_begin_tx = true;
    auto failed       = tx.write(payload, sizeof(payload));
    ASSERT_FALSE(failed.has_value());
    EXPECT_EQ(failed.error(), m5::hal::v2::error::error_t::BUSY);
    auto after = tx.getLastTransferStatus();
    ASSERT_TRUE(after.has_value());
    EXPECT_EQ(after->transfer_id, before->transfer_id);
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

TEST(RxAccessor, ExplicitAccessBorrowsAndLastStatusIsPerRead)
{
    RecordingIBus bus;
    bus.rx_queue = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4};
    m5::hal::v2::uart::RxAccessor rx{bus, {}};

    ASSERT_RESULT_OK(rx.beginAccess());
    uint8_t first[2]  = {};
    uint8_t second[3] = {};
    auto ra           = rx.read(first, sizeof(first));
    ASSERT_TRUE(ra.has_value()) << "err=" << m5::hal::v2::error::toString(ra.error());
    auto rb = rx.read(second, sizeof(second));
    ASSERT_TRUE(rb.has_value()) << "err=" << m5::hal::v2::error::toString(rb.error());

    ASSERT_RESULT_OK(rx.endAccess());
    auto status = rx.getLastTransferStatus();
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->totals.rx, sizeof(second));
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
    EXPECT_FALSE(rx.inAccess());
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
    auto status = dev.getLastTransferStatus();
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->totals.tx, sizeof(tx));
    EXPECT_EQ(status->totals.rx, sizeof(rx));
    EXPECT_EQ(status->completion, m5::hal::v2::bus::CompletionLevel::Complete);
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
    EXPECT_EQ(bus.lifecycle_order, std::vector<char>({'T', 'R'}));

    auto nested = dev.beginAccess(0);
    ASSERT_FALSE(nested.has_value());
    EXPECT_EQ(nested.error(), m5::hal::v2::error::error_t::INVALID_STATE);

    auto tx_result = other_tx.beginAccess(0);
    ASSERT_FALSE(tx_result.has_value());
    EXPECT_EQ(tx_result.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);

    auto rx_result = other_rx.beginAccess(0);
    ASSERT_FALSE(rx_result.has_value());
    EXPECT_EQ(rx_result.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);

    ASSERT_TRUE(dev.endAccess().has_value());
    EXPECT_FALSE(dev.inAccess());
    EXPECT_EQ(bus.lifecycle_order, std::vector<char>({'T', 'R', 'r', 't'}));
}

TEST(IBus, ConcurrentIndependentChannelEndDoesNotCrossReadSlots)
{
    for (size_t iteration = 0; iteration < 64; ++iteration) {
        m5::hal::v2::uart::IBus bus;
        m5::hal::v2::uart::TxAccessor tx{bus, {}};
        m5::hal::v2::uart::RxAccessor rx{bus, {}};
        ASSERT_RESULT_OK(tx.beginAccess(0));
        ASSERT_RESULT_OK(rx.beginAccess(0));

        std::atomic<unsigned> ready{0};
        std::atomic<bool> go{false};
        bool tx_ok = false;
        bool rx_ok = false;
        std::thread tx_end([&] {
            ready.fetch_add(1, std::memory_order_release);
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            tx_ok = tx.endAccess().has_value();
        });
        std::thread rx_end([&] {
            ready.fetch_add(1, std::memory_order_release);
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            rx_ok = rx.endAccess().has_value();
        });
        while (ready.load(std::memory_order_acquire) != 2) {
            std::this_thread::yield();
        }
        go.store(true, std::memory_order_release);
        tx_end.join();
        rx_end.join();
        EXPECT_TRUE(tx_ok);
        EXPECT_TRUE(rx_ok);
    }
}

TEST(Accessor, FacadeBeginRollsBackTxWhenRxBeginFails)
{
    RecordingIBus bus;
    bus.fail_begin_rx = true;
    m5::hal::v2::uart::Accessor dev{bus, {}};

    auto begun = dev.beginAccess(0);
    ASSERT_FALSE(begun.has_value());
    EXPECT_EQ(begun.error(), m5::hal::v2::error::error_t::BUSY);
    EXPECT_FALSE(dev.inAccess());
    EXPECT_EQ(bus.active_tx, 0);
    EXPECT_EQ(bus.active_rx, 0);
    EXPECT_EQ(bus.lifecycle_order, std::vector<char>({'T', 'R', 't'}));
}

TEST(Accessor, ChildAccessIsVisibleAndBlocksFacadeMutation)
{
    RecordingIBus bus;
    m5::hal::v2::uart::Accessor dev{bus, {}};
    ASSERT_RESULT_OK(dev.tx().beginAccess(0));
    EXPECT_TRUE(dev.inAccess());

    auto begun = dev.beginAccess(0);
    ASSERT_FALSE(begun.has_value());
    EXPECT_EQ(begun.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    m5::hal::v2::uart::AccessConfig cfg;
    cfg.baud_rate   = 57600;
    auto configured = dev.setConfig(cfg);
    ASSERT_FALSE(configured.has_value());
    EXPECT_EQ(configured.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    ASSERT_RESULT_OK(dev.tx().endAccess());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
