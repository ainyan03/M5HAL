// SPDX-License-Identifier: MIT
// Host regression harness for the espidf I2C slave backend
// (m5_hal/variants/frameworks/espidf/hal/i2c/slave.inl). The backend under
// test compiles UNMODIFIED (gated only by M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS, added
// alongside its existing ESP_PLATFORM gates) against the fake IDF header
// tree in ../fakes/include. See ../fakes/README.md for the harness design,
// its fidelity limits, and how to extend it to another peripheral.
//
// Each TEST drives the backend the same way real hardware would: an
// accessor (beginTransaction / read / write / endTransaction, the same
// public API an application uses) on one side, and hand-scripted fake ISR
// events (via fireIsr(), which pokes the fake i2c_dev_t model and then
// synchronously invokes the captured ISR handler) standing in for the wire
// on the other. No thread ever runs -- see ../fakes/README.md "Scope and
// fidelity limits" for why that is sufficient for these scenarios.

#include <gtest/gtest.h>
#include "support/gtest_watchdog.hpp"

#include <M5HAL_v2.hpp>

#include "m5_hal/hal/v2/bus/bus.inl"
#include "m5_hal/hal/v2/data/ring.inl"
#include "m5_hal/hal/v2/data/stream.inl"
#include "m5_hal/hal/v2/i2c/slave.inl"
#include "m5_hal/hal/v2/service/service.inl"

#include "m5_hal/variants/frameworks/espidf/hal/i2c/slave.inl"

#include <esp_intr_alloc.h>
#include <hal/i2c_ll.h>
#include <hal/i2c_types.h>
#include <soc/i2c_struct.h>

#include <algorithm>
#include <cstdint>
#include <numeric>
#include <vector>

namespace {

using m5::hal::v2::data::ConstDataSpan;
using m5::hal::v2::data::DataSpan;
using m5::hal::v2::i2c::SlaveBus_espidf;
using m5::hal::v2::i2c::SlaveBusConfig;
using m5::hal::v2::i2c::SlaveStreamAccessor;

// Every scenario fires the ISR before the accessor opens, so the transaction
// the accessor wants is ALWAYS already allocated -- beginTransaction can use
// a single non-blocking attempt (timeout 0). This is deliberate CI hygiene,
// not an optimization: the default TIMEOUT_FOREVER retry loop turns a
// regression that stops producing openable transactions into a test-process
// HANG (observed with a `_current = nullptr`-at-STOP mutation), while a
// 0-timeout attempt turns the same regression into an immediate FAIL.
constexpr uint32_t kBeginNonBlocking = 0;

SlaveBusConfig makeConfig()
{
    SlaveBusConfig cfg;
    cfg.pin_scl = 1;
    cfg.pin_sda = 2;
    cfg.address = 0x42;
    return cfg;
}

// The fake device model is a process-wide singleton per port (matches real
// hardware: I2C_NUM_0 is one physical peripheral -- see hal/i2c_ll.h).
// SlaveBus_espidf::init() resets the fields the state machine depends on
// (FIFO counts, interrupt mask, stretch), so each TEST's fresh Harness
// starts from a clean model as long as it calls init() first (which the
// Harness ctor does).
i2c_dev_t& fakeHw()
{
    return m5hal_hostharness::i2cDeviceFor(I2C_NUM_0);
}

// Appends bytes to the fake RX FIFO ("what the master just clocked in").
// Must be called with the fake FIFO fully drained by the state machine's
// last handleIsr() pass (i2c_ll_read_rxfifo consumes from the front and
// shifts the remainder down -- see soc/i2c_struct.h) -- every scenario below
// drains to 0 before the next prime, so plain append is safe here.
template <typename Container>
void primeRxFifo(const Container& bytes)
{
    auto& hw = fakeHw();
    for (uint8_t b : bytes) {
        ASSERT_LT(hw.rxfifo_count, i2c_dev_t::kFifoLen);
        hw.rxfifo[hw.rxfifo_count++] = b;
    }
}

// Scripts one fake ISR event: primes the interrupt-pending bit(s), the
// address-phase direction, and (for a STRETCH event) the stretch cause --
// then synchronously invokes the captured handler, exactly as
// SlaveBus_espidf::isrThunk would from a real interrupt. A STRETCH event
// always marks the fake stretch as held first (a STRETCH interrupt IS the
// hardware announcing it is holding SCL); the handler may clear it.
void fireIsr(uint32_t pending_bits, bool is_read,
             i2c_slave_stretch_cause_t stretch_cause = I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH)
{
    auto& hw = fakeHw();
    hw.int_st |= pending_bits;
    hw.slave_rw = is_read ? I2C_SLAVE_READ_BY_MASTER : I2C_SLAVE_WRITE_BY_MASTER;
    if ((pending_bits & I2C_SLAVE_STRETCH_INT_ENA_M) != 0) {
        hw.stretch_cause  = stretch_cause;
        hw.stretch_active = true;
    }
    m5hal_hostharness::fireLastIsr();
}

// Owns a fresh SlaveBus_espidf + bound accessor, inited against the fake
// device model. RAII teardown (~SlaveBus_espidf calls release()) frees the
// captured ISR handle and resets the fake model's interrupt mask, so the
// next Harness's init() sees a clean baseline.
struct Harness {
    SlaveBus_espidf bus;
    SlaveStreamAccessor acc{bus};

    Harness()
    {
        auto r = bus.init(makeConfig());
        if (!r.has_value()) {
            ADD_FAILURE() << "bus.init failed: err=" << m5::hal::v2::error::toString(r.error());
        }
    }
};

}  // namespace

// ---------------------------------------------------------------------------
// Baseline: the harness can drive a plain write and a plain read at all,
// before the intricate races below.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveHostHarness, InitRejectsNonSentinelNegativeAndOutOfRangeController)
{
    // Only -1 is the "backend default port" sentinel; every other negative
    // value is a corrupted / miscomputed index. An index at or beyond the
    // chip's HP port count (127 exceeds every chip's) is likewise rejected
    // before any hardware state is touched.
    SlaveBus_espidf bus;
    auto cfg       = makeConfig();
    cfg.controller = -2;
    auto r         = bus.init(cfg);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);

    cfg.controller = 127;
    r              = bus.init(cfg);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error(), m5::hal::v2::error::error_t::INVALID_ARGUMENT);
}

TEST(EspidfI2cSlaveHostHarness, BasicWriteTransactionIsFullyReadable)
{
    Harness h;

    const uint8_t written[] = {0x11, 0x22, 0x33};
    primeRxFifo(written);
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);

    ASSERT_TRUE(h.acc.beginTransaction(kBeginNonBlocking).has_value());
    uint8_t rx[sizeof(written)] = {};
    auto read                   = h.acc.read(DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(read.has_value()) << m5::hal::v2::error::toString(read.error());
    EXPECT_EQ(*read, sizeof(written));
    EXPECT_TRUE(std::equal(std::begin(written), std::end(written), rx));

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP
    auto complete = h.acc.transactionComplete();
    ASSERT_TRUE(complete.has_value());
    EXPECT_TRUE(*complete);
    EXPECT_TRUE(h.acc.endTransaction().has_value());
    EXPECT_EQ(h.bus.rxOverflowCount(), 0u);
}

TEST(EspidfI2cSlaveHostHarness, BasicReadTransactionServesComposedReply)
{
    Harness h;

    // Pure read: address-match stretch with no prior write phase.
    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, /*is_read=*/true, I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH);

    ASSERT_TRUE(h.acc.beginTransaction(kBeginNonBlocking).has_value());
    const uint8_t reply[] = {0xCA, 0xFE};
    auto write            = h.acc.write(ConstDataSpan{reply, sizeof(reply)});
    ASSERT_TRUE(write.has_value()) << m5::hal::v2::error::toString(write.error());
    EXPECT_EQ(*write, sizeof(reply));

    // write() released the stretch synchronously (no responder task
    // required -- see ../fakes/README.md) and staged the reply in the fake
    // TX FIFO.
    ASSERT_EQ(fakeHw().txfifo_count, sizeof(reply));
    EXPECT_TRUE(std::equal(std::begin(reply), std::end(reply), fakeHw().txfifo));
    EXPECT_FALSE(fakeHw().stretch_active);

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/true);  // STOP
    EXPECT_TRUE(h.acc.endTransaction().has_value());
}

TEST(EspidfI2cSlaveHostHarness, EndTransactionReleasesRxFullHold)
{
    Harness h;

    std::vector<uint8_t> fill(SlaveBus_espidf::kRxCapacity);
    std::iota(fill.begin(), fill.end(), uint8_t{0});
    primeRxFifo(fill);
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);

    ASSERT_TRUE(h.acc.beginTransaction(kBeginNonBlocking).has_value());

    primeRxFifo(std::vector<uint8_t>{0xA5});
    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, /*is_read=*/false, I2C_SLAVE_STRETCH_CAUSE_RX_FULL);

    EXPECT_TRUE(fakeHw().stretch_active);
    EXPECT_EQ(fakeHw().int_ena & (I2C_SLAVE_STRETCH_INT_ENA_M | I2C_RXFIFO_WM_INT_ENA_M), 0u)
        << "rx_full hold masks the level sources until the held transaction is drained or discarded";

    auto ended = h.acc.endTransaction();
    ASSERT_TRUE(ended.has_value()) << m5::hal::v2::error::toString(ended.error());

    EXPECT_FALSE(fakeHw().stretch_active);
    EXPECT_EQ(fakeHw().int_ena & (I2C_SLAVE_STRETCH_INT_ENA_M | I2C_RXFIFO_WM_INT_ENA_M),
              I2C_SLAVE_STRETCH_INT_ENA_M | I2C_RXFIFO_WM_INT_ENA_M);
}

// ---------------------------------------------------------------------------
// A write->RESTART->read boundary can arrive while the write transaction's RX
// ring is full and a tail byte remains in the HW FIFO. The read address match
// must not reclassify that RX_FULL hold as address_read: a reply queued before
// the consumer drains RX must not release SCL or touch the TX FIFO. Once read()
// frees the RX ring, the normal TX_EMPTY transition serves the already queued
// reply.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveHostHarness, RxFullHoldSurvivesReadAddressMatchUntilRxIsDrained)
{
    Harness h;

    std::vector<uint8_t> backlog(SlaveBus_espidf::kRxCapacity);
    std::iota(backlog.begin(), backlog.end(), uint8_t{0});
    primeRxFifo(backlog);
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);

    ASSERT_TRUE(h.acc.beginTransaction(kBeginNonBlocking).has_value());
    auto readable = h.acc.readableBytes();
    ASSERT_TRUE(readable.has_value()) << m5::hal::v2::error::toString(readable.error());
    ASSERT_EQ(*readable, backlog.size());

    // The additional write byte cannot fit, then the master immediately issues
    // a repeated-start read. The address-match pass itself drains this residual
    // FIFO tail and enters RX_FULL before it attempts to establish a TX hold.
    primeRxFifo(std::vector<uint8_t>{0xA5});
    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, /*is_read=*/true, I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH);
    ASSERT_TRUE(fakeHw().stretch_active);

    const uint8_t reply[] = {0xC3, 0x5A};
    auto queued           = h.acc.write(ConstDataSpan{reply, sizeof(reply)});
    ASSERT_TRUE(queued.has_value()) << m5::hal::v2::error::toString(queued.error());
    EXPECT_EQ(*queued, sizeof(reply));
    EXPECT_TRUE(fakeHw().stretch_active)
        << "RX back-pressure must retain SCL ownership until the consumer drains the full ring";
    EXPECT_EQ(fakeHw().txfifo_count, 0u)
        << "a reply queued during RX_FULL must wait for the post-release TX_EMPTY transition";

    std::vector<uint8_t> received(backlog.size());
    auto read = h.acc.read(DataSpan{received.data(), received.size()});
    ASSERT_TRUE(read.has_value()) << m5::hal::v2::error::toString(read.error());
    EXPECT_EQ(*read, received.size());
    EXPECT_EQ(received, backlog);
    EXPECT_FALSE(fakeHw().stretch_active) << "read() drains the FIFO tail before safely releasing RX_FULL";

    // The resumed read reaches an empty TX FIFO. Its normal continuation path
    // consumes the reply that was safely queued while RX back-pressure held SCL.
    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, /*is_read=*/true, I2C_SLAVE_STRETCH_CAUSE_TX_EMPTY);
    ASSERT_EQ(fakeHw().txfifo_count, sizeof(reply));
    EXPECT_TRUE(std::equal(std::begin(reply), std::end(reply), fakeHw().txfifo));
    EXPECT_FALSE(fakeHw().stretch_active);

    EXPECT_TRUE(h.acc.endTransaction().has_value());
}

TEST(EspidfI2cSlaveHostHarness, ReleaseTimeoutReturnsErrorAndInitDoesNotReenter)
{
    SlaveBus_espidf bus;
    auto init = bus.init(makeConfig());
    ASSERT_TRUE(init.has_value()) << m5::hal::v2::error::toString(init.error());

    auto released = bus.release();
    ASSERT_FALSE(released.has_value());
    EXPECT_EQ(released.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);

    auto reinit = bus.init(makeConfig());
    ASSERT_FALSE(reinit.has_value());
    EXPECT_EQ(reinit.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
}

// ---------------------------------------------------------------------------
// cc133e89: a reply composed while servicing a WRITE transaction must not
// leak onto the wire if a zero-gap READ address-match beats the accessor's
// endTransaction() of that write. Regresses the `_open == _current`
// stale-reply guard in snapshotResponseLocked() / write()'s release path.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveHostHarness, Cc133e89StaleComposedReplyDoesNotLeakIntoFollowingRead)
{
    Harness h;

    // Write transaction: master writes one byte, the accessor composes a
    // reply anyway (an application that always queues its next reply,
    // regardless of the transaction's own direction -- the pattern that
    // exposed the race), then STOPs.
    primeRxFifo(std::vector<uint8_t>{0x10});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    ASSERT_TRUE(h.acc.beginTransaction(kBeginNonBlocking).has_value());
    uint8_t rx_byte = 0;
    ASSERT_TRUE(h.acc.read(DataSpan{&rx_byte, 1}).has_value());
    EXPECT_EQ(rx_byte, 0x10);

    const uint8_t stale_reply[] = {0xDE, 0xAD};
    auto queued                 = h.acc.write(ConstDataSpan{stale_reply, sizeof(stale_reply)});
    ASSERT_TRUE(queued.has_value()) << m5::hal::v2::error::toString(queued.error());
    EXPECT_EQ(fakeHw().txfifo_count, 0u) << "no active read hold yet: the reply only queues onto the tx ring";

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP closes the write on the wire

    // Zero-gap follow-up read address-match, BEFORE the accessor closes the
    // write transaction it is still holding open -- the cc133e89 race. The
    // ISR allocates a FRESH transaction for the new read (_current), while
    // the accessor's `_open` still points at the just-completed write one.
    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, /*is_read=*/true, I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH);

    // serve()'s loop for the (stale, not-yet-closed) write transaction
    // writes again -- e.g. streaming more reply bytes before it has
    // observed transactionComplete() and called endTransaction(). This is
    // the EXACT call cc133e89's fix guards: `_open` (the write transaction)
    // is no longer the wire's transaction (`_current` is now the new read's
    // allocation), so this must NOT release the new read's stretch with the
    // stale/fill content -- that was the regression (a premature release
    // with an empty snapshot -> leading 0xFF fill byte, or the stale reply
    // itself, leaking onto the new read).
    const uint8_t more_stale[] = {0x99};
    auto stale_write           = h.acc.write(ConstDataSpan{more_stale, sizeof(more_stale)});
    ASSERT_TRUE(stale_write.has_value()) << m5::hal::v2::error::toString(stale_write.error());

    EXPECT_EQ(fakeHw().txfifo_count, 0u) << "write() onto the stale transaction must not release the new read's hold";
    EXPECT_TRUE(fakeHw().stretch_active) << "the new read's stretch must stay held pending a fresh compose";

    ASSERT_TRUE(h.acc.endTransaction().has_value());

    // The accessor opens the transaction the ISR allocated for the new read
    // and composes a fresh reply; this one DOES release the hold.
    ASSERT_TRUE(h.acc.beginTransaction(kBeginNonBlocking).has_value());
    const uint8_t fresh_reply[] = {0xBE, 0xEF, 0x01};
    auto fresh                  = h.acc.write(ConstDataSpan{fresh_reply, sizeof(fresh_reply)});
    ASSERT_TRUE(fresh.has_value()) << m5::hal::v2::error::toString(fresh.error());
    EXPECT_EQ(*fresh, sizeof(fresh_reply));

    ASSERT_EQ(fakeHw().txfifo_count, sizeof(fresh_reply));
    EXPECT_TRUE(std::equal(std::begin(fresh_reply), std::end(fresh_reply), fakeHw().txfifo));
    EXPECT_FALSE(fakeHw().stretch_active);

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/true);  // STOP
    EXPECT_TRUE(h.acc.endTransaction().has_value());
}

// ---------------------------------------------------------------------------
// a999a5fa: at STOP, whatever the HW FIFO still holds must land in the rx[]
// STOP-tail reserve (kRxArrayCapacity = 2x kRxCapacity) instead of being
// dropped, even when the ring's unread backlog is already near the
// back-pressure threshold. Exercises the exact boundary
// (kRxCapacity + SOC_I2C_FIFO_LEN == kRxArrayCapacity).
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveHostHarness, A999a5faStopTailSpillsIntoReserveWithoutOverflow)
{
    Harness h;

    // First pass: drain kRxCapacity - 1 bytes (near, but not at, the
    // back-pressure threshold).
    std::vector<uint8_t> first_batch(SlaveBus_espidf::kRxCapacity - 1);
    std::iota(first_batch.begin(), first_batch.end(), uint8_t{0});
    primeRxFifo(first_batch);
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);

    // STOP arrives with a full HW FIFO tail still queued (the master
    // clocked its last SOC_I2C_FIFO_LEN bytes with no later stretch to hold
    // it): the reserve must absorb all of it.
    std::vector<uint8_t> tail_batch(i2c_dev_t::kFifoLen);
    std::iota(tail_batch.begin(), tail_batch.end(), uint8_t{100});
    primeRxFifo(tail_batch);
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);

    ASSERT_TRUE(h.acc.beginTransaction(kBeginNonBlocking).has_value());
    std::vector<uint8_t> all(first_batch.size() + tail_batch.size());
    auto read = h.acc.read(DataSpan{all.data(), all.size()});
    ASSERT_TRUE(read.has_value()) << m5::hal::v2::error::toString(read.error());
    EXPECT_EQ(*read, all.size());

    std::vector<uint8_t> expected = first_batch;
    expected.insert(expected.end(), tail_batch.begin(), tail_batch.end());
    EXPECT_EQ(all, expected);
    EXPECT_EQ(h.bus.rxOverflowCount(), 0u);

    EXPECT_TRUE(h.acc.endTransaction().has_value());
}

// ---------------------------------------------------------------------------
// 38bf837d: a write-direction ISR pass must disable the level-type
// TXFIFO_WM source outright, not spin re-entering the ISR (the storm
// brake). Reproduces a stale TXFIFO_WM left armed from a read whose STOP
// teardown was skipped.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveHostHarness, TxfifoWmStormBrakeDisablesOnStaleWriteDirection)
{
    Harness h;

    fakeHw().int_ena |= I2C_TXFIFO_WM_INT_ENA_M;
    fireIsr(I2C_TXFIFO_WM_INT_ENA_M, /*is_read=*/false);

    EXPECT_EQ(fakeHw().int_ena & I2C_TXFIFO_WM_INT_ENA_M, 0u)
        << "a write-direction pass must disable TXFIFO_WM, or the level-type source spins the ISR forever";
}

// ---------------------------------------------------------------------------
// SPLIT write->read: the read address-match following a write's STOP must
// allocate a FRESH transaction, never reuse the just-completed write one --
// including when both events coalesce into a single ISR pass (handleIsr's
// STOP-before-STRETCH ordering contract).
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveHostHarness, SplitWriteThenReadAllocatesFreshTransactionOnMergedIsrPass)
{
    Harness h;

    // Register-pointer write phase: address-match (write direction) with
    // one pending RX byte.
    primeRxFifo(std::vector<uint8_t>{0x07});
    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, /*is_read=*/false, I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH);
    ASSERT_TRUE(h.acc.beginTransaction(kBeginNonBlocking).has_value());
    uint8_t reg_ptr = 0;
    ASSERT_TRUE(h.acc.read(DataSpan{&reg_ptr, 1}).has_value());
    EXPECT_EQ(reg_ptr, 0x07);

    // Merged single ISR pass: the write's STOP and the follow-up read's
    // address-match stretch arrive TOGETHER (both interrupt-pending bits
    // set before one handleIsr() invocation) -- a delayed ISR pass that
    // coalesced both bus events. Scripted directly (not via fireIsr(), which
    // only carries one cause) to make the coalescing explicit.
    {
        auto& hw        = fakeHw();
        hw.rxfifo_count = 0;
        hw.int_st |= (I2C_TRANS_COMPLETE_INT_ENA_M | I2C_SLAVE_STRETCH_INT_ENA_M);
        hw.slave_rw       = I2C_SLAVE_READ_BY_MASTER;
        hw.stretch_cause  = I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH;
        hw.stretch_active = true;
        m5hal_hostharness::fireLastIsr();
    }

    // The write transaction is now complete on the wire; the accessor still
    // holds it open (it has not called endTransaction() yet).
    auto complete = h.acc.transactionComplete();
    ASSERT_TRUE(complete.has_value());
    EXPECT_TRUE(*complete);
    ASSERT_TRUE(h.acc.endTransaction().has_value());

    // Opening the next transaction must reach the NEW slot the merged
    // pass's STRETCH branch allocated -- readableBytes()==0 (a pure read
    // never received rx bytes) proves it is not the write transaction's
    // slot, which had 1 unread byte's worth of history.
    ASSERT_TRUE(h.acc.beginTransaction(kBeginNonBlocking).has_value());
    auto readable = h.acc.readableBytes();
    ASSERT_TRUE(readable.has_value());
    EXPECT_EQ(*readable, 0u) << "must not reuse the completed write transaction's slot";

    EXPECT_TRUE(h.acc.endTransaction().has_value());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
