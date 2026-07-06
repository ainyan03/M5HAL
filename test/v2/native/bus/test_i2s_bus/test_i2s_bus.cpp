// SPDX-License-Identifier: MIT
#include "../static_bus_view_contract.hpp"

#include <M5HAL_v2.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

// i2s::Bus facade (phase 1) + i2s::BusView intern (phase 2) + RX / full-duplex
// tests.
//
// I2S is offered only by the espidf variant (ESP-only), so there is no I2S
// backend on a native build. To exercise the facade + BusView + the RX path on
// the host, this file defines a TEST-LOCAL fake backend (no public surface
// change): it accepts any pins and loops writes into an internal FIFO that
// reads drain (so a TX write followed by an RX read round-trips the bytes).
// Tests focus on facade construction, write / read forwarding, the query API,
// BusView interning by (BCLK, WS, DOUT, DIN), the RxAccessor, full duplex, and
// the role config field.

namespace m5::hal::v2::i2s {

// Test-only config + backend. Defining a BackendFor specialization for a
// test-local config keeps the production variant landscape unchanged
// (I2S = espidf only) while letting Bus::init / BusView::acquire run natively.
struct FakeBusConfig : public IBusConfig {
    using IBusConfig::IBusConfig;
};

class FakeBus : public IBus {
public:
    m5::hal::v2::result_t<void> init(const FakeBusConfig& cfg)
    {
        _config = cfg;  // slice pins/kind for getConfig()
        return {};
    }

    // Drains the source into an internal FIFO and reports the bytes accepted, so
    // the facade's write-forwarding is observable and a later read() returns the
    // same bytes (the loopback used by the full-duplex test).
    m5::hal::v2::result_t<size_t> write(bus::IAccessor* /*owner*/, const AccessConfig& /*cfg*/, data::Source* src,
                                        size_t len) override
    {
        // Mirror the espidf direction guard: writing on an RX-only bus (no DOUT)
        // is not supported, so the wrong-direction guard (F2) is exercised here.
        if (_config.pin_dout < 0) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::NOT_IMPLEMENTED);
        }
        size_t done = 0;
        if (src != nullptr) {
            while (!src->eof() && done < len) {
                auto span = src->peek(len - done);
                if (!span.has_value() || span.value().size == 0) {
                    break;
                }
                _fifo.insert(_fifo.end(), span.value().data, span.value().data + span.value().size);
                if (!src->advance(span.value().size).has_value()) {
                    break;
                }
                done += span.value().size;
            }
        }
        return done;
    }

    m5::hal::v2::result_t<size_t> writableBytes(bus::IAccessor* /*owner*/, const AccessConfig& /*cfg*/) override
    {
        // RX-only bus (no DOUT): nothing is writable (mirrors espidf F2 guard).
        if (_config.pin_dout < 0) {
            return static_cast<size_t>(0);
        }
        return _config.tx_buffer_size;
    }

    // Pushes up to `len` FIFO bytes into the sink and reports the count drained,
    // so the facade's read-forwarding is observable.
    m5::hal::v2::result_t<size_t> read(bus::IAccessor* /*owner*/, const AccessConfig& /*cfg*/, data::Sink* dst,
                                       size_t len) override
    {
        // Mirror the espidf direction guard: reading on a TX-only bus (no DIN)
        // is not supported, so the wrong-direction guard (F2) is exercised here.
        if (_config.pin_din < 0) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::NOT_IMPLEMENTED);
        }
        size_t done = 0;
        if (dst != nullptr) {
            while (!_fifo.empty() && done < len) {
                auto reserved = dst->reserve(len - done);
                if (!reserved.has_value() || reserved.value().size == 0) {
                    break;
                }
                const size_t take = std::min(reserved.value().size, _fifo.size());
                for (size_t i = 0; i < take; ++i) {
                    reserved.value().data[i] = _fifo[i];
                }
                if (!dst->commit(take).has_value()) {
                    break;
                }
                _fifo.erase(_fifo.begin(), _fifo.begin() + static_cast<std::vector<uint8_t>::difference_type>(take));
                done += take;
            }
        }
        return done;
    }

    m5::hal::v2::result_t<size_t> readableBytes(bus::IAccessor* /*owner*/, const AccessConfig& /*cfg*/) override
    {
        // TX-only bus (no DIN): nothing is readable (mirrors espidf F2 guard).
        if (_config.pin_din < 0) {
            return static_cast<size_t>(0);
        }
        return _fifo.size();
    }

private:
    std::vector<uint8_t> _fifo;
};

template <>
struct BackendFor<FakeBusConfig> {
    using type = FakeBus;
};

}  // namespace m5::hal::v2::i2s

namespace {
namespace v2 = m5::hal::v2;
}

// ---- i2s::Bus facade (phase 1) -----------------------------------------------

TEST(I2sBusFacade, InitWithFakeBackendSucceeds)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 4;
    cfg.pin_ws   = 5;
    cfg.pin_dout = 6;
    cfg.pin_din  = 7;
    auto r       = facade.init(cfg);
    ASSERT_TRUE(r.has_value());
}

TEST(I2sBusFacade, WriteForwardedToBackend)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 4;
    cfg.pin_ws   = 5;
    cfg.pin_dout = 6;
    ASSERT_TRUE(facade.init(cfg).has_value());

    const uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    v2::data::MemorySource src{v2::data::ConstDataSpan{payload, sizeof(payload)}};
    v2::i2s::AccessConfig acc;
    auto r = facade.write(nullptr, acc, &src, sizeof(payload));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), sizeof(payload));
}

TEST(I2sBusFacade, WritableBytesForwardedToBackend)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk       = 4;
    cfg.pin_ws         = 5;
    cfg.pin_dout       = 6;  // TX wired so writableBytes is meaningful
    cfg.tx_buffer_size = 4096;
    ASSERT_TRUE(facade.init(cfg).has_value());

    v2::i2s::AccessConfig acc;
    auto r = facade.writableBytes(nullptr, acc);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), 4096u);
}

TEST(I2sBusFacade, QueryApiBeforeInitReturnsDefaults)
{
    v2::i2s::Bus facade;
    // No backend yet — the base-class safe defaults apply.
    EXPECT_EQ(facade.backendKind(), v2::types::backend_kind_t::Software);
    EXPECT_EQ(facade.controllerId(), -1);
    EXPECT_EQ(facade.maxFrequency(), 0u);
}

// ---- i2s::BusView intern (phase 2) -------------------------------------------

TEST(I2sBusView, SamePinsReturnSameInstance)
{
    auto& hal = v2::getM5_Hal();

    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 8;
    cfg.pin_ws   = 9;
    cfg.pin_dout = 10;
    cfg.pin_din  = 11;

    auto a = hal.I2S.acquire(cfg);
    ASSERT_TRUE(a.has_value()) << "first acquire should succeed";
    ASSERT_TRUE(a.value()) << "returned shared_ptr must be non-null";

    auto b = hal.I2S.acquire(cfg);
    ASSERT_TRUE(b.has_value()) << "second acquire with same pins should succeed";
    EXPECT_EQ(a.value().get(), b.value().get()) << "same wiring -> same interned instance";
}

TEST(I2sBusView, SameIdentityWithDifferentRoleIsRejected)
{
    auto& hal = v2::getM5_Hal();

    v2::i2s::FakeBusConfig cfg_a;
    cfg_a.pin_bclk = 20;
    cfg_a.pin_ws   = 21;
    cfg_a.pin_dout = 22;
    cfg_a.pin_din  = 23;
    cfg_a.role     = v2::i2s::IBusConfig::Role::Master;

    v2::i2s::FakeBusConfig cfg_b = cfg_a;
    cfg_b.role                   = v2::i2s::IBusConfig::Role::Slave;  // same wiring, different clock ownership

    auto a = hal.I2S.acquire(cfg_a);
    ASSERT_TRUE(a.has_value());

    auto b = hal.I2S.acquire(cfg_b);
    ASSERT_FALSE(b.has_value());
    EXPECT_EQ(b.error(), v2::error::error_t::INVALID_STATE);
}

TEST(I2sBusView, DifferentBclkPinsReturnDistinctInstances)
{
    auto& hal = v2::getM5_Hal();

    v2::i2s::FakeBusConfig cfg_a;
    cfg_a.pin_bclk = 12;
    cfg_a.pin_ws   = 13;
    cfg_a.pin_dout = 14;
    cfg_a.pin_din  = 15;

    v2::i2s::FakeBusConfig cfg_b;
    cfg_b.pin_bclk = 16;  // different BCLK -> different bus
    cfg_b.pin_ws   = 13;
    cfg_b.pin_dout = 14;
    cfg_b.pin_din  = 15;

    auto a = hal.I2S.acquire(cfg_a);
    auto b = hal.I2S.acquire(cfg_b);
    ASSERT_TRUE(a.has_value());
    ASSERT_TRUE(b.has_value());
    EXPECT_NE(a.value().get(), b.value().get()) << "different BCLK -> distinct instances";
}

TEST(I2sBusView, InvalidPinsReturnError)
{
    auto& hal = v2::getM5_Hal();

    v2::i2s::FakeBusConfig cfg;
    // pin_bclk = -1 (invalid sentinel) -> INVALID_ARGUMENT
    cfg.pin_bclk = -1;
    cfg.pin_ws   = 4;
    cfg.pin_dout = 5;

    auto r = hal.I2S.acquire(cfg);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), v2::error::error_t::INVALID_ARGUMENT);
}

TEST(I2sBusView, AcquiredBusReturnsKindI2S)
{
    auto& hal = v2::getM5_Hal();

    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 17;
    cfg.pin_ws   = 18;
    cfg.pin_dout = 19;
    cfg.pin_din  = 20;

    auto r = hal.I2S.acquire(cfg);
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value()->getBusKind(), v2::types::bus_kind_t::I2S);
}

TEST(I2sBusView, StaticPolicyCommitIsNoop)
{
    auto& hal = v2::getM5_Hal();
    v2::test::bus_contract::expectStaticCommitSurface(hal.I2S);
}

TEST(I2sBusView, LogicalAcquireSurfaceExistsButIsStaticPolicy)
{
    auto& hal = v2::getM5_Hal();
    v2::i2s::LogicalBusConfig req;
    req.pin_bclk                               = 21;
    req.pin_ws                                 = 22;
    req.pin_dout                               = 23;
    v2::i2s::LogicalBusConfig invalid_identity = req;
    invalid_identity.pin_bclk                  = -1;

    v2::test::bus_contract::expectStaticLogicalAcquireContract(hal.I2S, req, invalid_identity);
}

// Co-own: a TxAccessor built from the acquire temporary keeps the bus alive
// after the temporary shared_ptr drops (i2s TxAccessor is also a StreamWriter,
// so the shared_ptr ctor must forward through the multiple-inheritance base).
TEST(I2sBusViewCoOwn, AccessorOutlivesAcquireTemporary)
{
    auto& hal = v2::getM5_Hal();
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 8;
    cfg.pin_ws   = 9;
    cfg.pin_dout = 10;
    v2::i2s::AccessConfig acc;

    // Co-own straight from the acquire temporary (no `*value()` deref).
    v2::i2s::TxAccessor tx{hal.I2S.acquire(cfg).value(), acc};
    // Reading the bus config goes through the live bus object: the only strong
    // reference now is the accessor's, so this proves co-own kept it alive.
    EXPECT_EQ(tx.getBusConfig().getBusKind(), v2::types::bus_kind_t::I2S);
}

// ---- i2s::Bus facade RX forward (phase 1) ------------------------------------

TEST(I2sBusFacade, ReadForwardedToBackend)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 4;
    cfg.pin_ws   = 5;
    cfg.pin_dout = 6;  // TX wired to prime the loopback FIFO
    cfg.pin_din  = 7;  // RX wired so the read path is supported
    ASSERT_TRUE(facade.init(cfg).has_value());

    // Prime the FIFO via a write, then read it back through the facade.
    const uint8_t payload[] = {0x10, 0x20, 0x30, 0x40};
    v2::data::MemorySource src{v2::data::ConstDataSpan{payload, sizeof(payload)}};
    v2::i2s::AccessConfig acc;
    ASSERT_TRUE(facade.write(nullptr, acc, &src, sizeof(payload)).has_value());

    uint8_t out[sizeof(payload)] = {};
    v2::data::MemorySink sink{v2::data::DataSpan{out, sizeof(out)}};
    auto r = facade.read(nullptr, acc, &sink, sizeof(out));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), sizeof(payload));
    EXPECT_EQ(0, std::memcmp(payload, out, sizeof(payload)));
}

TEST(I2sBusFacade, ReadableBytesForwardedToBackend)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 4;
    cfg.pin_ws   = 5;
    cfg.pin_dout = 6;  // TX wired to prime the loopback FIFO
    cfg.pin_din  = 7;  // RX wired so readableBytes is meaningful
    ASSERT_TRUE(facade.init(cfg).has_value());

    v2::i2s::AccessConfig acc;
    // Empty FIFO -> 0 readable.
    auto r0 = facade.readableBytes(nullptr, acc);
    ASSERT_TRUE(r0.has_value());
    EXPECT_EQ(r0.value(), 0u);

    const uint8_t payload[] = {0x01, 0x02, 0x03};
    v2::data::MemorySource src{v2::data::ConstDataSpan{payload, sizeof(payload)}};
    ASSERT_TRUE(facade.write(nullptr, acc, &src, sizeof(payload)).has_value());

    auto r1 = facade.readableBytes(nullptr, acc);
    ASSERT_TRUE(r1.has_value());
    EXPECT_EQ(r1.value(), sizeof(payload));
}

// ---- Wrong-direction guards (F2): no lazy channel for the unwired direction --

// TX-only bus (DOUT wired, DIN = -1): the read path is not supported and the
// readable-bytes query reports zero. On espidf these return BEFORE ensureChannel
// so no RX channel is created as a side effect; the fake mirrors the guard.
TEST(I2sBusFacade, TxOnlyRejectsReadAndReportsZeroReadable)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 4;
    cfg.pin_ws   = 5;
    cfg.pin_dout = 6;
    cfg.pin_din  = -1;  // TX-only wiring
    ASSERT_TRUE(facade.init(cfg).has_value());

    v2::i2s::AccessConfig acc;
    uint8_t out[4] = {};
    v2::data::MemorySink sink{v2::data::DataSpan{out, sizeof(out)}};
    auto rd = facade.read(nullptr, acc, &sink, sizeof(out));
    ASSERT_FALSE(rd.has_value());
    EXPECT_EQ(rd.error(), v2::error::error_t::NOT_IMPLEMENTED);

    auto rb = facade.readableBytes(nullptr, acc);
    ASSERT_TRUE(rb.has_value());
    EXPECT_EQ(rb.value(), 0u);
}

// RX-only bus (DIN wired, DOUT = -1): the write path is not supported and the
// writable-bytes query reports zero. Mirrors the espidf guard that returns
// BEFORE ensureChannel so no TX channel is created as a side effect.
TEST(I2sBusFacade, RxOnlyRejectsWriteAndReportsZeroWritable)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 4;
    cfg.pin_ws   = 5;
    cfg.pin_dout = -1;  // RX-only wiring
    cfg.pin_din  = 6;
    ASSERT_TRUE(facade.init(cfg).has_value());

    v2::i2s::AccessConfig acc;
    const uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    v2::data::MemorySource src{v2::data::ConstDataSpan{payload, sizeof(payload)}};
    auto wr = facade.write(nullptr, acc, &src, sizeof(payload));
    ASSERT_FALSE(wr.has_value());
    EXPECT_EQ(wr.error(), v2::error::error_t::NOT_IMPLEMENTED);

    auto wb = facade.writableBytes(nullptr, acc);
    ASSERT_TRUE(wb.has_value());
    EXPECT_EQ(wb.value(), 0u);
}

// ---- i2s::RxAccessor (phase 1) -----------------------------------------------

TEST(I2sRxAccessor, ReadDrainsBus)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 4;
    cfg.pin_ws   = 5;
    cfg.pin_dout = 6;
    cfg.pin_din  = 7;
    ASSERT_TRUE(facade.init(cfg).has_value());

    v2::i2s::AccessConfig acc;
    // Prime the FIFO directly through a TX accessor, then drain via RX accessor.
    v2::i2s::TxAccessor tx{facade, acc};
    const uint8_t payload[] = {0xAA, 0xBB, 0xCC, 0xDD};
    ASSERT_TRUE(tx.write(payload, sizeof(payload)).has_value());

    v2::i2s::RxAccessor rx{facade, acc};
    uint8_t out[sizeof(payload)] = {};
    auto r                       = rx.read(out, sizeof(out));
    ASSERT_TRUE(r.has_value());
    EXPECT_EQ(r.value(), sizeof(payload));
    EXPECT_EQ(0, std::memcmp(payload, out, sizeof(payload)));
}

TEST(I2sRxAccessor, ReadableBytesReportsBuffered)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 4;
    cfg.pin_ws   = 5;
    cfg.pin_dout = 6;
    cfg.pin_din  = 7;
    ASSERT_TRUE(facade.init(cfg).has_value());

    v2::i2s::AccessConfig acc;
    v2::i2s::RxAccessor rx{facade, acc};
    auto empty = rx.readableBytes();
    ASSERT_TRUE(empty.has_value());
    EXPECT_EQ(empty.value(), 0u);

    v2::i2s::TxAccessor tx{facade, acc};
    const uint8_t payload[] = {0x11, 0x22, 0x33, 0x44, 0x55};
    ASSERT_TRUE(tx.write(payload, sizeof(payload)).has_value());

    auto buffered = rx.readableBytes();
    ASSERT_TRUE(buffered.has_value());
    EXPECT_EQ(buffered.value(), sizeof(payload));
}

// Co-own: an RxAccessor built from the acquire temporary keeps the bus alive
// after the temporary shared_ptr drops (the shared_ptr ctor must forward
// through the StreamReader multiple-inheritance base, like the TX co-own test).
TEST(I2sRxBusViewCoOwn, AccessorOutlivesAcquireTemporary)
{
    auto& hal = v2::getM5_Hal();
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 30;
    cfg.pin_ws   = 31;
    cfg.pin_din  = 32;
    v2::i2s::AccessConfig acc;

    v2::i2s::RxAccessor rx{hal.I2S.acquire(cfg).value(), acc};
    EXPECT_EQ(rx.getBusConfig().getBusKind(), v2::types::bus_kind_t::I2S);
}

// ---- Full duplex: independent TX / RX channel locks --------------------------

TEST(I2sFullDuplex, WriteThenReadRoundTrips)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 40;
    cfg.pin_ws   = 41;
    cfg.pin_dout = 42;  // full-duplex wiring: both DOUT and DIN
    cfg.pin_din  = 43;
    ASSERT_TRUE(facade.init(cfg).has_value());

    v2::i2s::AccessConfig acc;
    v2::i2s::TxAccessor tx{facade, acc};
    v2::i2s::RxAccessor rx{facade, acc};

    // TX and RX hold independent channel locks: holding the TX window open must
    // not block taking the RX window (a single-mutex bus would deadlock here).
    ASSERT_TRUE(tx.beginAccess(0).has_value());
    ASSERT_TRUE(rx.beginAccess(0).has_value()) << "RX lock must be independent of the held TX lock";
    ASSERT_TRUE(rx.endAccess().has_value());
    ASSERT_TRUE(tx.endAccess().has_value());

    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34};
    auto wrote              = tx.write(payload, sizeof(payload));
    ASSERT_TRUE(wrote.has_value());
    EXPECT_EQ(wrote.value(), sizeof(payload));

    uint8_t out[sizeof(payload)] = {};
    auto got                     = rx.read(out, sizeof(out));
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got.value(), sizeof(payload));
    EXPECT_EQ(0, std::memcmp(payload, out, sizeof(payload)));
}

// The bundled Accessor (TX + RX) opens both channels and round-trips.
TEST(I2sFullDuplex, BundledAccessorRoundTrips)
{
    v2::i2s::Bus facade;
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 44;
    cfg.pin_ws   = 45;
    cfg.pin_dout = 46;
    cfg.pin_din  = 47;
    ASSERT_TRUE(facade.init(cfg).has_value());

    v2::i2s::AccessConfig acc;
    v2::i2s::Accessor dev{facade, acc};

    const uint8_t payload[] = {0x55, 0x66, 0x77, 0x88};
    ASSERT_TRUE(dev.write(payload, sizeof(payload)).has_value());

    uint8_t out[sizeof(payload)] = {};
    auto got                     = dev.read(out, sizeof(out));
    ASSERT_TRUE(got.has_value());
    EXPECT_EQ(got.value(), sizeof(payload));
    EXPECT_EQ(0, std::memcmp(payload, out, sizeof(payload)));
}

// ---- Role config field -------------------------------------------------------

TEST(I2sRole, RoleRidesOnAcquiredBusConfig)
{
    auto& hal = v2::getM5_Hal();
    v2::i2s::FakeBusConfig cfg;
    cfg.pin_bclk = 50;
    cfg.pin_ws   = 51;
    cfg.pin_din  = 52;
    cfg.role     = v2::i2s::IBusConfig::Role::Slave;

    auto r = hal.I2S.acquire(cfg);
    ASSERT_TRUE(r.has_value());
    // The acquired bus exposes the role that was set on the config (getConfig()
    // is covariant on i2s::IBus, so role survives the facade's slice to _config).
    EXPECT_EQ(r.value()->getConfig().role, v2::i2s::IBusConfig::Role::Slave);
}

TEST(I2sRole, DefaultRoleIsMaster)
{
    v2::i2s::IBusConfig cfg;
    EXPECT_EQ(cfg.role, v2::i2s::IBusConfig::Role::Master);
}

// gtest main (PlatformIO collects this via test_filter=v2/native/*).
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
