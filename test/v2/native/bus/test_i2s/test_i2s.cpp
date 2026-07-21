// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <m5_hal/variants/frameworks/espidf/hal/i2s/i2s.hpp>
#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <atomic>
#include <cstring>
#include <thread>
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
    // The fake uses the same portable kind config as production providers.
    m5::hal::v2::result_t<void> init(const m5::hal::v2::i2s::IBusConfig& config)
    {
        _config = config;
        return {};
    }

    m5::hal::v2::result_t<void> lockFor(m5::hal::v2::bus::IAccessor& owner, m5::hal::v2::i2s::Channel channel)
    {
        return lockChannel(owner, channel, 0);
    }

    m5::hal::v2::result_t<void> unlockFor(m5::hal::v2::bus::IAccessor& owner, m5::hal::v2::i2s::Channel channel)
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
        m5::hal::v2::bus::OperationContext<m5::hal::v2::i2s::AccessConfig>& context) override
    {
        last_owner = operationOwner(context);
        ++begin_count;
        lifecycle_order.push_back(context.runtime.mode == m5::hal::v2::bus::OperationMode::Tx ? 'T' : 'R');
        if (fail_rx_begin && context.runtime.mode == m5::hal::v2::bus::OperationMode::Rx) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::IO_ERROR);
        }
        return {};
    }

    m5::hal::v2::result_t<void> endOperationBackend(
        m5::hal::v2::bus::OperationContext<m5::hal::v2::i2s::AccessConfig>& context) override
    {
        last_owner = operationOwner(context);
        ++end_count;
        lifecycle_order.push_back(context.runtime.mode == m5::hal::v2::bus::OperationMode::Tx ? 't' : 'r');
        return {};
    }

    m5::hal::v2::result_t<size_t> writeBackend(
        m5::hal::v2::bus::OperationContext<m5::hal::v2::i2s::AccessConfig>& context, m5::hal::v2::data::Source* tx,
        size_t len) override
    {
        last_owner = operationOwner(context);
        last_cfg   = context.config;
        if (fail_write) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::IO_ERROR);
        }
        if (len > max_write) {
            len = max_write;
        }
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

    m5::hal::v2::result_t<size_t> writableBytesBackend(
        m5::hal::v2::bus::OperationContext<m5::hal::v2::i2s::AccessConfig>& context) override
    {
        last_owner = operationOwner(context);
        last_cfg   = context.config;
        ++writable_calls;
        return stub_writable;
    }

    m5::hal::v2::result_t<size_t> readBackend(
        m5::hal::v2::bus::OperationContext<m5::hal::v2::i2s::AccessConfig>& context, m5::hal::v2::data::Sink* rx,
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
        m5::hal::v2::bus::OperationContext<m5::hal::v2::i2s::AccessConfig>& context) override
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
    m5::hal::v2::i2s::AccessConfig last_cfg;
    size_t stub_writable  = 1024;
    size_t begin_count    = 0;
    size_t end_count      = 0;
    size_t writable_calls = 0;
    size_t readable_calls = 0;
    bool fail_rx_begin    = false;
    bool fail_write       = false;
    size_t max_write      = static_cast<size_t>(-1);
};

class InspectableI2sAccessor : public m5::hal::v2::bus::IAccessor {
public:
    InspectableI2sAccessor(StubIBus& bus, const m5::hal::v2::i2s::AccessConfig& config = {})
        : m5::hal::v2::bus::IAccessor{bus}, _typed_bus{bus}, _context{makeOperationContext(config)}
    {
    }

    const m5::hal::v2::i2s::AccessConfig& getConfig() const override
    {
        return _context.config;
    }

    m5::hal::v2::bus::OperationContext<m5::hal::v2::i2s::AccessConfig>& context()
    {
        return _context;
    }

    m5::hal::v2::result_t<void> begin(m5::hal::v2::bus::OperationMode mode)
    {
        _channel =
            mode == m5::hal::v2::bus::OperationMode::Tx ? m5::hal::v2::i2s::Channel::Tx : m5::hal::v2::i2s::Channel::Rx;
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
    StubIBus& _typed_bus;
    m5::hal::v2::bus::OperationContext<m5::hal::v2::i2s::AccessConfig> _context;
    m5::hal::v2::i2s::Channel _channel = m5::hal::v2::i2s::Channel::Tx;
};

}  // namespace

TEST(I2SCheckedFacade, RejectsInactiveEndedWrongBusAndWrongAccessorContexts)
{
    namespace bus   = m5::hal::v2::bus;
    namespace error = m5::hal::v2::error;
    namespace i2s   = m5::hal::v2::i2s;

    StubIBus first_bus;
    StubIBus second_bus;
    InspectableI2sAccessor tx{first_bus};
    InspectableI2sAccessor rx{first_bus};
    InspectableI2sAccessor other{first_bus};

    auto inactive_write = first_bus.write(tx.context(), nullptr, 0);
    ASSERT_FALSE(inactive_write.has_value());
    EXPECT_EQ(inactive_write.error(), error::error_t::INVALID_STATE);
    auto inactive_writable = first_bus.writableBytes(tx.context());
    ASSERT_FALSE(inactive_writable.has_value());
    EXPECT_EQ(inactive_writable.error(), error::error_t::INVALID_STATE);
    auto inactive_read = first_bus.read(rx.context(), nullptr, 0);
    ASSERT_FALSE(inactive_read.has_value());
    EXPECT_EQ(inactive_read.error(), error::error_t::INVALID_STATE);

    tx.context().runtime.begin(0, 0, bus::OperationMode::Tx);
    ASSERT_RESULT_OK(first_bus.lockFor(other, i2s::Channel::Tx));
    auto wrong_accessor = first_bus.beginOperation(tx.context());
    ASSERT_FALSE(wrong_accessor.has_value());
    EXPECT_EQ(wrong_accessor.error(), error::error_t::INVALID_STATE);
    ASSERT_RESULT_OK(first_bus.unlockFor(other, i2s::Channel::Tx));

    ASSERT_RESULT_OK(tx.begin(bus::OperationMode::Tx));
    ASSERT_RESULT_OK(rx.begin(bus::OperationMode::Rx));
    auto wrong_bus_readable = second_bus.readableBytes(rx.context());
    ASSERT_FALSE(wrong_bus_readable.has_value());
    EXPECT_EQ(wrong_bus_readable.error(), error::error_t::INVALID_STATE);
    auto wrong_bus_transfer = second_bus.transfer(tx.context(), rx.context(), nullptr, 0, nullptr, 0);
    ASSERT_FALSE(wrong_bus_transfer.has_value());
    EXPECT_EQ(wrong_bus_transfer.error(), error::error_t::INVALID_STATE);
    ASSERT_RESULT_OK(rx.end());
    ASSERT_RESULT_OK(tx.end());

    auto ended_read = first_bus.read(rx.context(), nullptr, 0);
    ASSERT_FALSE(ended_read.has_value());
    EXPECT_EQ(ended_read.error(), error::error_t::INVALID_STATE);
    EXPECT_TRUE(first_bus.call_order.empty());
    EXPECT_EQ(first_bus.writable_calls, 0u);
    EXPECT_EQ(first_bus.readable_calls, 0u);
}

TEST(I2SCheckedFacade, RejectsCorruptRuntimeAndRecoversBothSlotsAndLocks)
{
    namespace bus   = m5::hal::v2::bus;
    namespace error = m5::hal::v2::error;

    StubIBus i2s_bus;
    InspectableI2sAccessor tx{i2s_bus};
    InspectableI2sAccessor rx{i2s_bus};

    ASSERT_RESULT_OK(tx.begin(bus::OperationMode::Tx));
    const auto registered_generation = tx.context().runtime.generation;
    ++tx.context().runtime.generation;
    auto stale = i2s_bus.writableBytes(tx.context());
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
    auto wrong_mode           = i2s_bus.readableBytes(rx.context());
    ASSERT_FALSE(wrong_mode.has_value());
    EXPECT_EQ(wrong_mode.error(), error::error_t::INVALID_STATE);
    auto wrong_mode_end = rx.end();
    ASSERT_FALSE(wrong_mode_end.has_value());
    EXPECT_EQ(wrong_mode_end.error(), error::error_t::INVALID_STATE);
    EXPECT_FALSE(rx.inAccess());
    ASSERT_RESULT_OK(tx.end());

    ASSERT_RESULT_OK(tx.begin(bus::OperationMode::Tx));
    ASSERT_RESULT_OK(rx.begin(bus::OperationMode::Rx));
    auto recovered = i2s_bus.transfer(tx.context(), rx.context(), nullptr, 0, nullptr, 0);
    ASSERT_TRUE(recovered.has_value()) << "err=" << error::toString(recovered.error());
    EXPECT_EQ(recovered->tx, 0u);
    EXPECT_EQ(recovered->rx, 0u);
    ASSERT_RESULT_OK(rx.end());
    ASSERT_RESULT_OK(tx.end());
}

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
// IBus base defaults fail closed as UNSUPPORTED.
// -------------------------------------------------------------------------
TEST(IBus, BaseDefaultWriteReturnsUnsupported)
{
    m5::hal::v2::i2s::IBus bus;
    m5::hal::v2::i2s::TxAccessor accessor{bus, {}};
    m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{}};
    auto result = accessor.write(src, 0);
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v2::error::error_t::UNSUPPORTED);
}

TEST(IBus, BaseDefaultWritableBytesReturnsUnsupported)
{
    m5::hal::v2::i2s::IBus bus;
    m5::hal::v2::i2s::TxAccessor accessor{bus, {}};
    auto result = accessor.writableBytes();
    ASSERT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), m5::hal::v2::error::error_t::UNSUPPORTED);
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

TEST(TxAccessor, ActiveAccessBorrowsLifecycleAndTracksEachWrite)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};

    ASSERT_RESULT_OK(tx.beginAccess());
    const uint8_t a[] = {0x01, 0x02};
    const uint8_t b[] = {0x03, 0x04, 0x05};
    auto wa           = tx.write(a, sizeof(a));
    ASSERT_TRUE(wa.has_value()) << "err=" << m5::hal::v2::error::toString(wa.error());
    auto wb = tx.write(b, sizeof(b));
    ASSERT_TRUE(wb.has_value()) << "err=" << m5::hal::v2::error::toString(wb.error());

    ASSERT_RESULT_OK(tx.endAccess());
    EXPECT_EQ(bus.begin_count, 1u);
    EXPECT_EQ(bus.end_count, 1u);
    auto status = tx.getLastTransferStatus();
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->transfer_id, 2u);
    EXPECT_EQ(status->totals.tx, sizeof(b));
    EXPECT_EQ(status->completion, m5::hal::v2::bus::CompletionLevel::Complete);
}

TEST(TxAccessor, AccessIsNonNestingAndExcludesSameDirection)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};
    m5::hal::v2::i2s::TxAccessor other{bus, {}};

    ASSERT_RESULT_OK(tx.beginAccess());
    auto nested = tx.beginAccess();
    ASSERT_FALSE(nested.has_value());
    EXPECT_EQ(nested.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    EXPECT_TRUE(tx.inAccess());

    auto blocked = other.beginAccess(0);
    ASSERT_FALSE(blocked.has_value());
    EXPECT_EQ(blocked.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);

    ASSERT_RESULT_OK(tx.endAccess());
    EXPECT_FALSE(tx.inAccess());

    ASSERT_RESULT_OK(other.beginAccess(0));
    ASSERT_RESULT_OK(other.endAccess());
}

TEST(I2sAccessor, CombinedAccessOrdersChildrenAndRollsBackRxFailure)
{
    StubIBus bus;
    m5::hal::v2::i2s::Accessor both{bus, {}};
    ASSERT_RESULT_OK(both.beginAccess());
    ASSERT_RESULT_OK(both.endAccess());
    EXPECT_EQ(bus.lifecycle_order, (std::vector<char>{'T', 'R', 'r', 't'}));

    StubIBus failing_bus;
    failing_bus.fail_rx_begin = true;
    m5::hal::v2::i2s::Accessor failing{failing_bus, {}};
    auto begun = failing.beginAccess();
    ASSERT_FALSE(begun.has_value());
    EXPECT_EQ(begun.error(), m5::hal::v2::error::error_t::IO_ERROR);
    EXPECT_FALSE(failing.inAccess());
    EXPECT_EQ(failing_bus.lifecycle_order, (std::vector<char>{'T', 'R', 't'}));
}

TEST(TxAccessor, StandaloneWriteSugarStillWorks)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};

    const uint8_t payload[] = {0xAA, 0xBB};
    auto result             = tx.write(payload, sizeof(payload));
    ASSERT_TRUE(result.has_value()) << "err=" << m5::hal::v2::error::toString(result.error());
    EXPECT_EQ(result.value(), sizeof(payload));
    EXPECT_FALSE(tx.inAccess());
    EXPECT_EQ(bus.begin_count, 1u);
    EXPECT_EQ(bus.end_count, 1u);
    ASSERT_EQ(bus.tx_recorded.size(), sizeof(payload));
}

TEST(TxAccessor, LastTransferStatusDistinguishesPartialAndAborted)
{
    StubIBus bus;
    bus.max_write = 1;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};
    const uint8_t payload[] = {0xAA, 0xBB};
    auto partial            = tx.write(payload, sizeof(payload));
    ASSERT_TRUE(partial.has_value());
    auto partial_status = tx.getLastTransferStatus();
    ASSERT_TRUE(partial_status.has_value());
    EXPECT_EQ(partial_status->completion, m5::hal::v2::bus::CompletionLevel::Partial);
    EXPECT_EQ(partial_status->totals.tx, 1u);

    bus.fail_write = true;
    auto failed    = tx.write(payload, sizeof(payload));
    ASSERT_FALSE(failed.has_value());
    auto failed_status = tx.getLastTransferStatus();
    ASSERT_TRUE(failed_status.has_value());
    EXPECT_EQ(failed_status->transfer_id, partial_status->transfer_id + 1);
    EXPECT_EQ(failed_status->completion, m5::hal::v2::bus::CompletionLevel::Aborted);
    EXPECT_EQ(failed_status->error, m5::hal::v2::error::error_t::IO_ERROR);
}

TEST(I2sAccessor, TxAndRxAccessCanBeActiveIndependently)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};
    m5::hal::v2::i2s::RxAccessor rx{bus, {}};
    ASSERT_RESULT_OK(tx.beginAccess(0));
    ASSERT_RESULT_OK(rx.beginAccess(0));
    EXPECT_TRUE(tx.inAccess());
    EXPECT_TRUE(rx.inAccess());
    ASSERT_RESULT_OK(rx.endAccess());
    ASSERT_RESULT_OK(tx.endAccess());
}

TEST(I2sAccessor, ConcurrentIndependentChannelEndDoesNotCrossReadSlots)
{
    for (size_t iteration = 0; iteration < 64; ++iteration) {
        m5::hal::v2::i2s::IBus bus;
        m5::hal::v2::i2s::TxAccessor tx{bus, {}};
        m5::hal::v2::i2s::RxAccessor rx{bus, {}};
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

TEST(TxAccessor, EndAccessOutsideAccessIsInvalidState)
{
    StubIBus bus;
    m5::hal::v2::i2s::TxAccessor tx{bus, {}};

    auto ended = tx.endAccess();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), m5::hal::v2::error::error_t::INVALID_STATE);
}

TEST(RxAccessor, ActiveAccessTracksLastReadOnly)
{
    StubIBus bus;
    bus.rx_queue = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4};
    m5::hal::v2::i2s::RxAccessor rx{bus, {}};

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
    EXPECT_EQ(status->transfer_id, 2u);
    EXPECT_EQ(status->totals.rx, sizeof(second));
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
    EXPECT_FALSE(rx.inAccess());
}

TEST(RxAccessor, EndAccessOutsideAccessIsInvalidState)
{
    StubIBus bus;
    m5::hal::v2::i2s::RxAccessor rx{bus, {}};

    auto ended = rx.endAccess();
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), m5::hal::v2::error::error_t::INVALID_STATE);
}

TEST(Accessor, WriteAndReadSugarsPublishCombinedLastStatus)
{
    StubIBus bus;
    bus.rx_queue = {0x31, 0x32};
    m5::hal::v2::i2s::Accessor accessor{bus, {}};

    const uint8_t tx[] = {0x11, 0x12, 0x13};
    ASSERT_RESULT_OK(accessor.write(tx, sizeof(tx)));
    auto tx_status = accessor.getLastTransferStatus();
    ASSERT_TRUE(tx_status.has_value());
    EXPECT_EQ(tx_status->totals.tx, sizeof(tx));
    EXPECT_EQ(tx_status->totals.rx, 0u);

    uint8_t rx[2] = {};
    ASSERT_RESULT_OK(accessor.read(rx, sizeof(rx)));
    auto rx_status = accessor.getLastTransferStatus();
    ASSERT_TRUE(rx_status.has_value());
    EXPECT_GT(rx_status->transfer_id, tx_status->transfer_id);
    EXPECT_EQ(rx_status->totals.tx, 0u);
    EXPECT_EQ(rx_status->totals.rx, sizeof(rx));
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
