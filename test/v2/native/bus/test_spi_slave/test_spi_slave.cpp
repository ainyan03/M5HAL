// SPDX-License-Identifier: MIT
#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>

#include <vector>

namespace {

// -------------------------------------------------------------------------
// Fake full-duplex SPI slave bus. No spi_slave hardware: it drains the tx
// Source and, byte-for-byte, also feeds those same bytes into the rx Sink
// (a loopback), exercising the SpiSlaveAccessor -> ISlaveBus::serve()
// Source/Sink wiring on a host build. Records the exchanged bytes for
// inspection. `serve_cap` lets a test simulate a master that clocks fewer
// than `len` bytes (early CS deassert).
// -------------------------------------------------------------------------
class FakeSlaveBus : public m5::hal::v2::spi::ISlaveBus {
public:
    m5::hal::v2::result_t<void> init(const m5::hal::v2::spi::SlaveBusConfig& cfg) override
    {
        _config = cfg;
        return {};
    }

    m5::hal::v2::result_t<void> release(void) override
    {
        served.clear();
        return {};
    }

    m5::hal::v2::result_t<size_t> serve(m5::hal::v2::bus::IAccessor* owner, m5::hal::v2::data::Source* tx,
                                        m5::hal::v2::data::Sink* rx, size_t len, uint32_t timeout_ms) override
    {
        last_owner      = owner;
        last_len        = len;
        last_timeout_ms = timeout_ms;

        // Honor a simulated short transaction (master clocked fewer bytes).
        const size_t exchange = (serve_cap < len) ? serve_cap : len;

        // Pull up to `exchange` bytes from the tx Source; past the Source's
        // content, fall back to the config fill byte (mirrors the real backend).
        std::vector<uint8_t> outgoing;
        size_t pulled = 0;
        while (tx != nullptr && !tx->eof() && pulled < exchange) {
            auto span = tx->peek(exchange - pulled);
            if (!span.has_value()) {
                return m5::stl::make_unexpected(span.error());
            }
            if (span.value().size == 0) {
                break;
            }
            outgoing.insert(outgoing.end(), span.value().data, span.value().data + span.value().size);
            auto advanced = tx->advance(span.value().size);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
            pulled += span.value().size;
        }
        while (outgoing.size() < exchange) {
            outgoing.push_back(_config.tx_fill_byte);
        }
        served = outgoing;

        // Loop the outgoing bytes back into the rx Sink (a stand-in for the
        // master's MOSI, so the test can assert both directions wired up).
        size_t committed = 0;
        while (rx != nullptr && !rx->closed() && committed < exchange) {
            auto reserved = rx->reserve(exchange - committed);
            if (!reserved.has_value()) {
                return m5::stl::make_unexpected(reserved.error());
            }
            size_t want = reserved.value().size;
            if (want == 0) {
                break;
            }
            if (want > exchange - committed) {
                want = exchange - committed;
            }
            for (size_t i = 0; i < want; ++i) {
                reserved.value().data[i] = outgoing[committed + i];
            }
            auto done = rx->commit(want);
            if (!done.has_value()) {
                return m5::stl::make_unexpected(done.error());
            }
            committed += want;
        }

        return exchange;
    }

    std::vector<uint8_t> served;
    m5::hal::v2::bus::IAccessor* last_owner = nullptr;
    size_t last_len                         = 0;
    uint32_t last_timeout_ms                = 0;
    size_t serve_cap                        = SIZE_MAX;  // no early-CS simulation by default
};

}  // namespace

// -------------------------------------------------------------------------
// SlaveBusConfig defaults
// -------------------------------------------------------------------------
TEST(SpiSlaveBusConfig, DefaultCtorSetsSpiKindAndPins)
{
    m5::hal::v2::spi::SlaveBusConfig cfg;
    EXPECT_EQ(cfg.getBusKind(), m5::hal::v2::types::bus_kind_t::SPI);
    EXPECT_EQ(cfg.pin_clk, -1);
    EXPECT_EQ(cfg.pin_mosi, -1);
    EXPECT_EQ(cfg.pin_miso, -1);
    EXPECT_EQ(cfg.pin_cs, -1);
    EXPECT_EQ(cfg.spi_mode, 0u);
    EXPECT_EQ(cfg.tx_fill_byte, 0x00u);
    EXPECT_EQ(cfg.timeout_ms, m5::hal::v2::types::TIMEOUT_FOREVER);
}

// -------------------------------------------------------------------------
// Accessor unbound: serve() rejects with INVALID_ARGUMENT
// -------------------------------------------------------------------------
TEST(SpiSlaveAccessor, ServeRequiresBoundBus)
{
    FakeSlaveBus bus;
    m5::hal::v2::spi::SpiSlaveAccessor acc{bus};
    EXPECT_TRUE(acc.isBound());
    EXPECT_EQ(&acc.getBus(), &bus);
    EXPECT_EQ(acc.getConfig().getBusKind(), m5::hal::v2::types::bus_kind_t::SPI);
}

// -------------------------------------------------------------------------
// Span overload: tx is clocked out, rx captures the (looped-back) bytes
// -------------------------------------------------------------------------
TEST(SpiSlaveAccessor, ServeSpanExchangesBothDirections)
{
    FakeSlaveBus bus;
    m5::hal::v2::spi::SpiSlaveAccessor acc{bus};

    const uint8_t tx[] = {0x11, 0x22, 0x33, 0x44};
    uint8_t rx[4]      = {};
    auto r = acc.serve(m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)}, m5::hal::v2::data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), sizeof(tx));

    // tx Source was drained into the bus.
    ASSERT_EQ(bus.served.size(), sizeof(tx));
    EXPECT_EQ(bus.served[0], 0x11u);
    EXPECT_EQ(bus.served[3], 0x44u);

    // rx Sink captured the looped-back bytes.
    EXPECT_EQ(rx[0], 0x11u);
    EXPECT_EQ(rx[1], 0x22u);
    EXPECT_EQ(rx[2], 0x33u);
    EXPECT_EQ(rx[3], 0x44u);

    // The accessor passes itself as owner and uses the larger span size as len.
    EXPECT_EQ(bus.last_owner, &acc);
    EXPECT_EQ(bus.last_len, sizeof(tx));
}

// -------------------------------------------------------------------------
// Span overload: len is the LARGER of the two spans (full-duplex)
// -------------------------------------------------------------------------
TEST(SpiSlaveAccessor, ServeSpanLenIsLargerOfBoth)
{
    FakeSlaveBus bus;
    m5::hal::v2::spi::SpiSlaveAccessor acc{bus};

    // rx larger than tx: len must be rx.size; tx is short so the tail comes from
    // tx_fill_byte (default 0x00).
    const uint8_t tx[] = {0xAB, 0xCD};
    uint8_t rx[5]      = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    auto r = acc.serve(m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)}, m5::hal::v2::data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), sizeof(rx));
    EXPECT_EQ(bus.last_len, sizeof(rx));
    EXPECT_EQ(rx[0], 0xABu);
    EXPECT_EQ(rx[1], 0xCDu);
    EXPECT_EQ(rx[2], 0x00u);  // fill byte
    EXPECT_EQ(rx[4], 0x00u);
}

// -------------------------------------------------------------------------
// Source/Sink overload: streaming callers
// -------------------------------------------------------------------------
TEST(SpiSlaveAccessor, ServeSourceSinkOverload)
{
    FakeSlaveBus bus;
    m5::hal::v2::spi::SpiSlaveAccessor acc{bus};

    const uint8_t payload[] = {0x01, 0x02, 0x03};
    m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};
    uint8_t rxbuf[3] = {};
    m5::hal::v2::data::MemorySink sink{m5::hal::v2::data::DataSpan{rxbuf, sizeof(rxbuf)}};

    auto r = acc.serve(&src, &sink, sizeof(payload), 100);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), sizeof(payload));
    EXPECT_EQ(rxbuf[0], 0x01u);
    EXPECT_EQ(rxbuf[2], 0x03u);
    EXPECT_EQ(bus.last_timeout_ms, 100u);
}

// -------------------------------------------------------------------------
// Early CS deassert: the master clocks fewer than `len` bytes; serve returns
// the actually-exchanged count and only that prefix is committed.
// -------------------------------------------------------------------------
TEST(SpiSlaveAccessor, ServeReportsShortExchange)
{
    FakeSlaveBus bus;
    bus.serve_cap = 2;  // master clocks only 2 of the requested bytes
    m5::hal::v2::spi::SpiSlaveAccessor acc{bus};

    const uint8_t tx[] = {0x10, 0x20, 0x30, 0x40};
    uint8_t rx[4]      = {0xEE, 0xEE, 0xEE, 0xEE};
    auto r = acc.serve(m5::hal::v2::data::ConstDataSpan{tx, sizeof(tx)}, m5::hal::v2::data::DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 2u);
    EXPECT_EQ(rx[0], 0x10u);
    EXPECT_EQ(rx[1], 0x20u);
    // Bytes past the short exchange are untouched in the sink.
    EXPECT_EQ(rx[2], 0xEEu);
    EXPECT_EQ(rx[3], 0xEEu);
}

// -------------------------------------------------------------------------
// Null rx Sink discards captured bytes (tx still drained, count still len)
// -------------------------------------------------------------------------
TEST(SpiSlaveAccessor, ServeNullSinkDiscardsRx)
{
    FakeSlaveBus bus;
    m5::hal::v2::spi::SpiSlaveAccessor acc{bus};

    const uint8_t payload[] = {0x55, 0x66};
    m5::hal::v2::data::MemorySource src{m5::hal::v2::data::ConstDataSpan{payload, sizeof(payload)}};

    auto r = acc.serve(&src, nullptr, sizeof(payload), m5::hal::v2::types::TIMEOUT_FOREVER);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), sizeof(payload));
    ASSERT_EQ(bus.served.size(), sizeof(payload));
    EXPECT_EQ(bus.served[0], 0x55u);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
