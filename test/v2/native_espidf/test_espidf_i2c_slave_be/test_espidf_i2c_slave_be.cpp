// SPDX-License-Identifier: MIT
// Host regression harness for the espidf I2C slave backend's classic-ESP32
// (no clock-stretch) BE flavor register-map ISR fast path
// (m5_hal/variants/frameworks/espidf/hal/i2c/slave.inl's
// M5HAL_ESPIDF_I2C_SLAVE_LL_BE section, bound via
// m5_hal/hal/v2/i2c/slave.hpp's ISlaveBus::bindIsrRegMap). The backend under
// test compiles UNMODIFIED against the same fake IDF header tree as
// ../test_espidf_i2c_slave/, except this env's M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_NO_STRETCH_CAPABILITY
// build flag flips soc/soc_caps.h's SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE to 0,
// which selects M5HAL_ESPIDF_I2C_SLAVE_LL_BE instead of
// M5HAL_ESPIDF_I2C_SLAVE_LL -- see ../fakes/README.md.
//
// The LL flavor's harness (../test_espidf_i2c_slave/) exercises the shared
// transaction-window machinery (RX/TX rings, STOP races); this one is scoped
// to what is DIFFERENT about the BE fast path: the ISR interprets RX bytes as
// register-map pointer/data directly (bypassing the Transaction rx[]/tx[]
// rings entirely) and keeps the TX FIFO pre-composed from reg_file/onRead.
// Each TEST scripts fake ISR events (fireIsr(), which pokes the fake
// i2c_dev_t model and synchronously invokes the captured ISR handler) and
// inspects the resulting fake TX/RX FIFO content and hook call counts --
// timing (the microsecond WTR window this fast path exists for) is out of
// this harness's reach (see ../fakes/README.md "Scope and fidelity limits");
// what IS in reach, and is what these tests pin, is that the compose/ingest
// happens INSIDE one ISR pass rather than needing a later task wakeup.
//
// serve()'s fast-path branch itself (SlaveRegMapAccessor::serve() waiting on
// transactionComplete()) is NOT exercised here: this harness's fake
// FreeRTOS never actually blocks a task (../fakes/README.md "No scheduler"),
// so a serve() call on a transaction that has not yet been completed by a
// STOP fireIsr() would spin the test process forever instead of failing.
// Every scenario below instead drives the underlying SlaveStreamAccessor
// (acc.stream()) directly with a non-blocking beginTransaction(0), mirroring
// ../test_espidf_i2c_slave/'s kBeginNonBlocking pattern.

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
#include <memory>
#include <utility>
#include <vector>

namespace {

using m5::hal::v2::data::DataSpan;
using m5::hal::v2::i2c::SlaveBus_espidf;
using m5::hal::v2::i2c::SlaveBusConfig;
using m5::hal::v2::i2c::SlaveRegMapAccessor;

// Same non-blocking-open rationale as ../test_espidf_i2c_slave/: every
// scenario fires the ISR before the accessor opens, so the transaction it
// wants is ALWAYS already allocated.
constexpr uint32_t kBeginNonBlocking = 0;

SlaveBusConfig makeConfig()
{
    SlaveBusConfig cfg;
    cfg.pin_scl = 1;
    cfg.pin_sda = 2;
    cfg.address = 0x42;
    // TxUnderrun::Fill is the default already -- BE rejects ::Stretch at
    // init() (no clock stretch to hold a reply compose on), so this is the
    // only legal value here.
    return cfg;
}

// Fake device model access -- see ../test_espidf_i2c_slave/'s identical
// helper for the process-wide-singleton-per-port rationale.
i2c_dev_t& fakeHw()
{
    return m5hal_hostharness::i2cDeviceFor(I2C_NUM_0);
}

template <typename Container>
void primeRxFifo(const Container& bytes)
{
    auto& hw = fakeHw();
    for (uint8_t b : bytes) {
        ASSERT_LT(hw.rxfifo_count, i2c_dev_t::kFifoLen);
        hw.rxfifo[hw.rxfifo_count++] = b;
    }
}

// Scripts one fake ISR event -- see ../test_espidf_i2c_slave/'s identical
// helper. `is_read` is accepted for signature parity with that file but the
// BE flavor's handleIsr() never reads hw.slave_rw (no address-match stretch
// to gate on; only RX_WM/TX_WM/STOP drive it).
void fireIsr(uint32_t pending_bits, bool is_read)
{
    auto& hw = fakeHw();
    hw.int_st |= pending_bits;
    hw.slave_rw = is_read ? I2C_SLAVE_READ_BY_MASTER : I2C_SLAVE_WRITE_BY_MASTER;
    m5hal_hostharness::fireLastIsr();
}

// Records every onRead/onWrite hook invocation. `reg_file` mirrors the
// Harness's backing store so countingOnRead can return a passthrough value
// (keeping the fast-path TX FIFO content assertions identical to the no-hook
// scenarios), while still counting/capturing every call for the "hook fires
// the expected byte count" scenario.
struct HookCounters {
    int on_read_calls  = 0;
    int on_write_calls = 0;
    std::vector<std::pair<uint8_t, uint8_t>> writes;
    const uint8_t* reg_file = nullptr;
};

uint8_t countingOnRead(uint8_t reg, void* ctx)
{
    auto* counters = static_cast<HookCounters*>(ctx);
    ++counters->on_read_calls;
    return counters->reg_file[reg];
}

void countingOnWrite(uint8_t reg, uint8_t value, void* ctx)
{
    auto* counters = static_cast<HookCounters*>(ctx);
    ++counters->on_write_calls;
    counters->writes.emplace_back(reg, value);
}

// Owns a fresh SlaveBus_espidf, inits it (BE flavor: M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_NO_STRETCH_CAPABILITY
// makes M5HAL_ESPIDF_I2C_SLAVE_LL_BE win -- see this file's header comment),
// THEN constructs the SlaveRegMapAccessor -- that ORDER matters:
// SlaveRegMapAccessor's constructor calls bindIsrRegMap immediately, which
// primes the TX FIFO from reg_file right away only once _hw is set (a no-op
// otherwise, see rebuildTxRegMapLocked's header contract), matching the
// real device fixture's own bus.init() -> accessor construction order
// (test/v2/hil/i2c_slave/device/i2c_slave.cpp). acc is built in the
// constructor BODY (not a same-class member with in-class initializers) so
// it runs strictly after bus.init(), not interleaved by member declaration
// order.
struct Harness {
    uint8_t reg_file[256] = {};
    SlaveBus_espidf bus;
    std::unique_ptr<SlaveRegMapAccessor> acc;

    // counters == nullptr: no hooks (fast path serves reg_file directly).
    // counters != nullptr: wires countingOnRead/countingOnWrite in, with
    // counters->reg_file pointed at THIS Harness's own backing store BEFORE
    // setOnRead/setOnWrite can trigger a bind-time compose that calls them --
    // acc->setOnRead's immediate rebind+rebuild (see SlaveRegMapAccessor::
    // rebindIsrRegMapIfBound) invokes countingOnRead synchronously, so
    // counters->reg_file must already be valid at that point, not wired up
    // by the caller afterwards.
    explicit Harness(HookCounters* counters = nullptr)
    {
        for (int i = 0; i < 256; ++i) {
            reg_file[i] = static_cast<uint8_t>(i);
        }
        auto r = bus.init(makeConfig());
        if (!r.has_value()) {
            ADD_FAILURE() << "bus.init failed: err=" << m5::hal::v2::error::toString(r.error());
        }
        acc = std::make_unique<SlaveRegMapAccessor>(bus, DataSpan{reg_file, sizeof(reg_file)});
        if (counters != nullptr) {
            counters->reg_file = reg_file;
            acc->setOnRead(&countingOnRead, counters);
            acc->setOnWrite(&countingOnWrite, counters);
        }
    }
};

// Expected register-map window starting at `pointer`, `count` bytes, 8-bit
// wrap -- matches reg_file[i] == i (Harness's default content).
std::vector<uint8_t> expectedWindow(uint8_t pointer, size_t count)
{
    std::vector<uint8_t> out(count);
    for (size_t i = 0; i < count; ++i) {
        out[i] = static_cast<uint8_t>(pointer + i);
    }
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. WTR-equivalent: the pointer byte's RX_WM ISR pass ingests the pointer
//    AND composes the TX FIFO reply in that SAME fireIsr() call -- no later
//    task wakeup is involved. This one-pass completion is the fast path's
//    entire reason to exist (a repeated-START read follows within
//    microseconds, well inside a serve() task's scheduling latency).
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveBeHostHarness, PointerByteComposesReplyWithinSameIsrPass)
{
    Harness h;

    primeRxFifo(std::vector<uint8_t>{0x10});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);

    ASSERT_EQ(fakeHw().txfifo_count, i2c_dev_t::kFifoLen);
    const auto want = expectedWindow(0x10, i2c_dev_t::kFifoLen);
    EXPECT_TRUE(std::equal(want.begin(), want.end(), fakeHw().txfifo));

    ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());
    auto readable = h.acc->stream().readableBytes();
    ASSERT_TRUE(readable.has_value());
    EXPECT_EQ(*readable, 0u) << "the pointer byte itself is not a readable payload byte";

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/true);  // STOP
    auto complete = h.acc->stream().transactionComplete();
    ASSERT_TRUE(complete.has_value());
    EXPECT_TRUE(*complete);
    EXPECT_TRUE(h.acc->stream().endTransaction().has_value());
    EXPECT_EQ(h.bus.rxOverflowCount(), 0u) << "the fast path never touches Transaction.rx[], so it cannot overflow";
}

TEST(EspidfI2cSlaveBeHostHarness, ReleaseTimeoutReturnsErrorAndInitDoesNotReenter)
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
// 2. SPLIT: a register-pointer write, STOP, then a separate pure-read
//    transaction resolves against the pointer the write set -- with NO new
//    RX activity in between. The STOP handler's unconditional rebuild is
//    what serves this (mirrors ESP32_I2C_slave_example's rebuildTxFifo()
//    call at every STOP, not just on RX).
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveBeHostHarness, SplitReadAfterStopUsesPersistedPointer)
{
    Harness h;

    primeRxFifo(std::vector<uint8_t>{0x30});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP of the write
    EXPECT_TRUE(h.acc->stream().endTransaction().has_value());

    // No RX_WM at all for the follow-up pure read -- the STOP rebuild above
    // already composed this window from the persisted pointer.
    ASSERT_EQ(fakeHw().txfifo_count, i2c_dev_t::kFifoLen);
    const auto want = expectedWindow(0x30, i2c_dev_t::kFifoLen);
    EXPECT_TRUE(std::equal(want.begin(), want.end(), fakeHw().txfifo));

    ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/true);  // STOP of the read
    EXPECT_TRUE(h.acc->stream().endTransaction().has_value());
}

// ---------------------------------------------------------------------------
// 3. Streaming read past one FIFO load, WITH an 8-bit wrap in the window:
//    successive TX_WM continuations keep auto-incrementing from where the
//    previous fill left off (no gap, no repeat).
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveBeHostHarness, TxWmContinuationAutoIncrementsAcrossEightBitWrap)
{
    Harness h;
    constexpr uint8_t kPointer = 0xF0;  // 240: the first 32-byte window wraps at 256.

    primeRxFifo(std::vector<uint8_t>{kPointer});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    ASSERT_EQ(fakeHw().txfifo_count, i2c_dev_t::kFifoLen);
    auto want = expectedWindow(kPointer, i2c_dev_t::kFifoLen);
    EXPECT_TRUE(std::equal(want.begin(), want.end(), fakeHw().txfifo)) << "first (wrapping) window";

    // Simulate the master having clocked out the whole first FIFO load: the
    // fake TX model tracks queued-not-yet-consumed count only (see
    // fakes/README.md), so emptying it is what "the master read it all"
    // looks like from i2c_ll_get_txfifo_len()'s free-space computation.
    fakeHw().txfifo_count = 0;
    fireIsr(I2C_TXFIFO_WM_INT_ENA_M, /*is_read=*/true);

    ASSERT_EQ(fakeHw().txfifo_count, i2c_dev_t::kFifoLen);
    want = expectedWindow(static_cast<uint8_t>(kPointer + i2c_dev_t::kFifoLen), i2c_dev_t::kFifoLen);
    EXPECT_TRUE(std::equal(want.begin(), want.end(), fakeHw().txfifo))
        << "continuation window picks up exactly where the first left off";
}

// ---------------------------------------------------------------------------
// 4. STOP resets pointer-received (not the pointer value itself): the next
//    transaction's first byte is treated as a NEW pointer, not data at the
//    old pointer + accumulated write_offset.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveBeHostHarness, StopResetsPointerReceivedForNextTransaction)
{
    Harness h;

    primeRxFifo(std::vector<uint8_t>{0x05, 0xAA});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());
    EXPECT_EQ(h.reg_file[0x05], 0xAA) << "regMapWriteByte applies synchronously, in the ISR pass itself";
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP
    EXPECT_TRUE(h.acc->stream().endTransaction().has_value());

    primeRxFifo(std::vector<uint8_t>{0xBB});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());

    // If STOP had NOT reset pointer-received, 0xBB would have been stored as
    // DATA at reg_file[0x05 + 1] = reg_file[0x06] instead of becoming the new
    // pointer -- reg_file[0x06] must stay untouched (still its default == 6).
    EXPECT_EQ(h.reg_file[0x06], 0x06) << "0xBB must be consumed as the NEW pointer, not written as data";
    ASSERT_EQ(fakeHw().txfifo_count, i2c_dev_t::kFifoLen);
    const auto want = expectedWindow(0xBB, i2c_dev_t::kFifoLen);
    EXPECT_TRUE(std::equal(want.begin(), want.end(), fakeHw().txfifo)) << "TX FIFO rebuilt from the NEW pointer 0xBB";

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/true);  // STOP
    EXPECT_TRUE(h.acc->stream().endTransaction().has_value());
}

// ---------------------------------------------------------------------------
// 5. The pointer persists across MULTIPLE transactions with no intervening
//    write (repeat-read), not just across a single STOP gap.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveBeHostHarness, PointerPersistsAcrossRepeatedPureReads)
{
    Harness h;

    primeRxFifo(std::vector<uint8_t>{0x50});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP of the write
    EXPECT_TRUE(h.acc->stream().endTransaction().has_value());

    const auto want = expectedWindow(0x50, i2c_dev_t::kFifoLen);

    // Two more pure-read transactions, neither with any RX activity: each
    // STOP's rebuild must keep composing from the SAME pointer.
    for (int i = 0; i < 2; ++i) {
        ASSERT_EQ(fakeHw().txfifo_count, i2c_dev_t::kFifoLen);
        EXPECT_TRUE(std::equal(want.begin(), want.end(), fakeHw().txfifo)) << "iteration " << i;
        ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());
        fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/true);  // STOP
        EXPECT_TRUE(h.acc->stream().endTransaction().has_value());
    }
}

// ---------------------------------------------------------------------------
// 6. onRead/onWrite fire exactly the expected number of times: onWrite once
//    per actual data byte (not the pointer byte), onRead once per byte the
//    ISR composes into the TX FIFO -- including the read-ahead the
//    unconditional rebuild-on-every-RX/STOP costs (documented BE trade-off,
//    not a bug: every rebuild recomposes the WHOLE FIFO from scratch rather
//    than incrementally, matching ESP32_I2C_slave_example's reference).
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveBeHostHarness, HooksFireExpectedByteCounts)
{
    HookCounters counters;
    Harness h{&counters};
    // Harness's own setOnRead/setOnWrite calls each trigger their own
    // rebindIsrRegMapIfBound() -> bindIsrRegMap() -> rebuildTxRegMapLocked()
    // rebuild (a fresh compose per rebind, unconditionally -- see
    // bindIsrRegMap's header contract), so on_read_calls is already nonzero
    // here. Measure DELTAS from this baseline, not absolute counts, so the
    // assertions below pin only the RX-drain/STOP-triggered composition this
    // scenario is actually about.
    const int baseline_reads = counters.on_read_calls;

    primeRxFifo(std::vector<uint8_t>{0x20, 0x01, 0x02, 0x03});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());

    EXPECT_EQ(counters.on_write_calls, 3) << "3 data bytes after the pointer byte";
    const std::vector<std::pair<uint8_t, uint8_t>> want_writes = {{0x20, 0x01}, {0x21, 0x02}, {0x22, 0x03}};
    EXPECT_EQ(counters.writes, want_writes);
    // The RX drain's rebuild composes one full FIFO load (read-ahead cost,
    // even though this is a write transaction -- see the class comment on
    // drainRxRegMapLocked's unconditional rebuild).
    EXPECT_EQ(counters.on_read_calls - baseline_reads, static_cast<int>(i2c_dev_t::kFifoLen));

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP: another full rebuild.
    EXPECT_TRUE(h.acc->stream().endTransaction().has_value());
    EXPECT_EQ(counters.on_read_calls - baseline_reads, static_cast<int>(i2c_dev_t::kFifoLen) * 2);
    EXPECT_EQ(counters.on_write_calls, 3) << "STOP does not re-ingest already-applied writes";
}

// ---------------------------------------------------------------------------
// 7. The accessor's pointer() must reflect the fast path's wire-side pointer,
//    not a stale task-context field the ISR fast path never touches.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveBeHostHarness, AccessorPointerReflectsIsrFastPathPointer)
{
    Harness h;
    EXPECT_EQ(h.acc->pointer(), 0x00) << "no pointer received yet";

    primeRxFifo(std::vector<uint8_t>{0x77});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());

    EXPECT_EQ(h.acc->pointer(), 0x77) << "pointer() must reflect the ISR fast path's wire-side pointer";

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP
    EXPECT_TRUE(h.acc->stream().endTransaction().has_value());
}

// ---------------------------------------------------------------------------
// 8. Binding ownership: a LATER accessor's bind supersedes an EARLIER one's,
//    and destroying the superseded (earlier) accessor must NOT tear out the
//    later one's still-active fast path.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveBeHostHarness, LaterAccessorBindSurvivesEarlierAccessorDestruction)
{
    SlaveBus_espidf bus;
    ASSERT_TRUE(bus.init(makeConfig()).has_value());

    uint8_t reg_a[256] = {};
    uint8_t reg_b[256] = {};
    for (int i = 0; i < 256; ++i) {
        reg_a[i] = static_cast<uint8_t>(i);
        reg_b[i] = static_cast<uint8_t>(0xFF - i);
    }

    auto acc_a = std::make_unique<SlaveRegMapAccessor>(bus, DataSpan{reg_a, sizeof(reg_a)});
    auto acc_b = std::make_unique<SlaveRegMapAccessor>(bus, DataSpan{reg_b, sizeof(reg_b)});

    // acc_b's construction bind is the "later wins" bind: the backend's single
    // slot now points at acc_b's binding, not acc_a's.
    acc_a.reset();  // destroying the SUPERSEDED accessor must not unbind acc_b.

    primeRxFifo(std::vector<uint8_t>{0x10});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);

    ASSERT_EQ(fakeHw().txfifo_count, i2c_dev_t::kFifoLen);
    std::vector<uint8_t> want_b(i2c_dev_t::kFifoLen);
    for (size_t i = 0; i < want_b.size(); ++i) {
        want_b[i] = static_cast<uint8_t>(0xFF - static_cast<uint8_t>(0x10 + i));
    }
    EXPECT_TRUE(std::equal(want_b.begin(), want_b.end(), fakeHw().txfifo))
        << "fast path still serves acc_b's reg_file after acc_a (the superseded binder) was destroyed";

    ASSERT_TRUE(acc_b->stream().beginTransaction(kBeginNonBlocking).has_value());
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP
    EXPECT_TRUE(acc_b->stream().endTransaction().has_value());
}

// ---------------------------------------------------------------------------
// 9. A (re)bind resets the binding's per-transaction wire state
//    (pointer_received / write_offset / tx_offset), so a hook change
//    mid-transaction cannot leave a byte misinterpreted as data at the OLD
//    write offset once the binding comes back.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveBeHostHarness, RebindResetsPerTransactionState)
{
    HookCounters counters;
    Harness h{&counters};

    // Start a write, ingest the pointer byte + one data byte -- pointer_received
    // and write_offset are now both live in the mid-transaction binding.
    primeRxFifo(std::vector<uint8_t>{0x40, 0xCC});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    ASSERT_TRUE(h.acc->stream().beginTransaction(kBeginNonBlocking).has_value());
    EXPECT_EQ(h.reg_file[0x40], 0xCC);

    // Re-bind mid-transaction (a hook change): per the bind contract this
    // resets pointer_received/write_offset/tx_offset on the binding, so the
    // NEXT byte is treated as a fresh pointer rather than data at 0x40+1.
    h.acc->setOnRead(&countingOnRead, &counters);

    primeRxFifo(std::vector<uint8_t>{0x99});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);

    EXPECT_EQ(h.reg_file[0x41], 0x41) << "0x99 must be consumed as the reset binding's new pointer, not as data";
    EXPECT_EQ(h.acc->pointer(), 0x99);

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP
    EXPECT_TRUE(h.acc->stream().endTransaction().has_value());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
