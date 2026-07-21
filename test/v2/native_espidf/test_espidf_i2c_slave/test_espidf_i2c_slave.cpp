// SPDX-License-Identifier: MIT
// Host regression harness for the espidf I2C slave backend
// (m5_hal/variants/frameworks/espidf/hal/i2c/slave.inl). The backend under
// test compiles UNMODIFIED (gated only by M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS, added
// alongside its existing ESP_PLATFORM gates) against the fake IDF header
// tree in ../fakes/include. See ../fakes/README.md for the harness design,
// its fidelity limits, and how to extend it to another peripheral.
//
// Each TEST drives the backend the same way real hardware would: an
// accessor (openWireFrame / read / write / closeWireFrame, the same
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
#include <array>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

namespace {

using m5::hal::v2::data::ConstDataSpan;
using m5::hal::v2::data::DataSpan;
using m5::hal::v2::i2c::SlaveAccessConfig;
using m5::hal::v2::i2c::SlaveAccessor;
using m5::hal::v2::i2c::SlaveBus_espidf;
using m5::hal::v2::i2c::SlaveBusConfig;
using m5::hal::v2::i2c::SlaveStreamAccessor;

void unusedTaskEntry(void*)
{
}

TEST(FreeRtosRuntimeMutex, ReportsTimeoutAndInvalidUnlock)
{
    namespace runtime = ::m5::hal::v2::runtime;
    using error_t     = ::m5::hal::v2::error::error_t;

    runtime::Mutex mutex;
    auto locked = mutex.lock(0);
    ASSERT_TRUE(locked.has_value()) << "err=" << ::m5::hal::v2::error::toString(locked.error());

    auto contended = mutex.lock(0);
    ASSERT_FALSE(contended.has_value());
    EXPECT_EQ(contended.error(), error_t::TIMEOUT_ERROR);

    auto unlocked = mutex.unlock();
    ASSERT_TRUE(unlocked.has_value()) << "err=" << ::m5::hal::v2::error::toString(unlocked.error());

    auto invalid = mutex.unlock();
    ASSERT_FALSE(invalid.has_value());
    EXPECT_EQ(invalid.error(), error_t::INVALID_STATE);
}

TEST(FreeRtosRuntimeMutex, PreservesTimeoutBudgetAtSemaphoreSeam)
{
    namespace runtime = ::m5::hal::v2::runtime;

    runtime::Mutex mutex;

    auto locked = mutex.lock(0);
    ASSERT_TRUE(locked.has_value());
    EXPECT_EQ(m5hal_fake_last_semaphore_take_ticks, 0u);
    ASSERT_TRUE(mutex.unlock().has_value());

    locked = mutex.lock(17);
    ASSERT_TRUE(locked.has_value());
    // 17 ms at 100 Hz is 1.7 ticks. The expected literal is deliberately
    // independent of the production conversion helper under test.
    EXPECT_EQ(m5hal_fake_last_semaphore_take_ticks, 2u);
    ASSERT_TRUE(mutex.unlock().has_value());

    locked = mutex.lock(::m5::hal::v2::types::TIMEOUT_FOREVER);
    ASSERT_TRUE(locked.has_value());
    EXPECT_EQ(m5hal_fake_last_semaphore_take_ticks, portMAX_DELAY);
    ASSERT_TRUE(mutex.unlock().has_value());
}

TEST(FreeRtosRuntimeEvent, LatchesMergesAndPreservesTimeoutBudget)
{
    namespace runtime = ::m5::hal::v2::runtime;
    using error_t     = ::m5::hal::v2::error::error_t;

    runtime::Event event;
    event.notify();
    event.notify();  // xSemaphoreGive() returns pdFALSE: a successful merge, not an error.
    auto consumed = event.wait(0);
    ASSERT_TRUE(consumed.has_value()) << "err=" << ::m5::hal::v2::error::toString(consumed.error());
    EXPECT_EQ(m5hal_fake_last_semaphore_take_ticks, 0u);

    auto empty = event.wait(17);
    ASSERT_FALSE(empty.has_value());
    EXPECT_EQ(empty.error(), error_t::TIMEOUT_ERROR);
    EXPECT_EQ(m5hal_fake_last_semaphore_take_ticks, 2u);

    event.notify();
    auto forever = event.wait(::m5::hal::v2::types::TIMEOUT_FOREVER);
    ASSERT_TRUE(forever.has_value()) << "err=" << ::m5::hal::v2::error::toString(forever.error());
    EXPECT_EQ(m5hal_fake_last_semaphore_take_ticks, portMAX_DELAY);
}

TEST(FreeRtosRuntimeTask, StartMapsArgumentStateAndCreateFailures)
{
    namespace runtime = ::m5::hal::v2::runtime;
    using error_t     = ::m5::hal::v2::error::error_t;

    runtime::Task task;

    auto null_entry = task.start(nullptr, nullptr);
    ASSERT_FALSE(null_entry.has_value());
    EXPECT_EQ(null_entry.error(), error_t::INVALID_ARGUMENT);

    auto zero_stack = task.start(&unusedTaskEntry, nullptr, nullptr, 0);
    ASSERT_FALSE(zero_stack.has_value());
    EXPECT_EQ(zero_stack.error(), error_t::INVALID_ARGUMENT);

    if constexpr (sizeof(size_t) > sizeof(uint32_t)) {
        const auto oversized = static_cast<size_t>(std::numeric_limits<uint32_t>::max()) + 1u;
        auto bad_stack       = task.start(&unusedTaskEntry, nullptr, nullptr, oversized);
        ASSERT_FALSE(bad_stack.has_value());
        EXPECT_EQ(bad_stack.error(), error_t::INVALID_ARGUMENT);
    }

    auto negative_priority = task.start(&unusedTaskEntry, nullptr, nullptr, 4096, -1);
    ASSERT_FALSE(negative_priority.has_value());
    EXPECT_EQ(negative_priority.error(), error_t::INVALID_ARGUMENT);

    auto bad_priority = task.start(&unusedTaskEntry, nullptr, nullptr, 4096, configMAX_PRIORITIES);
    ASSERT_FALSE(bad_priority.has_value());
    EXPECT_EQ(bad_priority.error(), error_t::INVALID_ARGUMENT);

    auto bad_core = task.start(&unusedTaskEntry, nullptr, nullptr, 4096, 1, portNUM_PROCESSORS);
    ASSERT_FALSE(bad_core.has_value());
    EXPECT_EQ(bad_core.error(), error_t::INVALID_ARGUMENT);

    auto low_core = task.start(&unusedTaskEntry, nullptr, nullptr, 4096, 1, ::m5::hal::v2::types::TASK_CORE_SAME - 1);
    ASSERT_FALSE(low_core.has_value());
    EXPECT_EQ(low_core.error(), error_t::INVALID_ARGUMENT);

    m5hal_hostharness::failNextPinnedTaskCreate();
    auto exhausted = task.start(&unusedTaskEntry, nullptr);
    ASSERT_FALSE(exhausted.has_value());
    EXPECT_EQ(exhausted.error(), error_t::OUT_OF_RESOURCE);
    EXPECT_FALSE(task.joinable());

    m5hal_hostharness::runNextPinnedTaskSynchronously();
    auto started = task.start(&unusedTaskEntry, nullptr);
    ASSERT_TRUE(started.has_value());
    EXPECT_TRUE(task.joinable());

    auto invalid_while_joinable = task.start(nullptr, nullptr);
    ASSERT_FALSE(invalid_while_joinable.has_value());
    EXPECT_EQ(invalid_while_joinable.error(), error_t::INVALID_ARGUMENT);

    auto duplicate = task.start(&unusedTaskEntry, nullptr);
    ASSERT_FALSE(duplicate.has_value());
    EXPECT_EQ(duplicate.error(), error_t::INVALID_STATE);

    task.join();
    EXPECT_FALSE(task.joinable());
}

class IdleRunnerService final : public ::m5::hal::v2::service::IService {
    ::m5::hal::v2::service::ServicePoll serviceImpl(const ::m5::hal::v2::service::ServiceContext&) override
    {
        return ::m5::hal::v2::service::ServiceResult::Idle;
    }
};

TEST(FreeRtosServiceRunner, ImplicitTaskFailureIsExactAndRollsBackRegistration)
{
    using error_t = ::m5::hal::v2::error::error_t;

    ::m5::hal::v2::service::ServiceRunner runner;
    IdleRunnerService service;

    m5hal_hostharness::failNextPinnedTaskCreate();
    auto added = runner.add(service);
    ASSERT_FALSE(added.has_value());
    EXPECT_EQ(added.error(), error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(runner.size(), size_t{0});
    EXPECT_FALSE(runner.autoRunActive());

    // Rollback must leave the service reusable rather than as a hidden
    // duplicate. The next add reaches Task::start again and reports that
    // second launch attempt's exact failure.
    m5hal_hostharness::failNextPinnedTaskCreate();
    auto retried = runner.add(service);
    ASSERT_FALSE(retried.has_value());
    EXPECT_EQ(retried.error(), error_t::OUT_OF_RESOURCE);
    EXPECT_EQ(runner.size(), size_t{0});
}

// Every scenario fires the ISR before the accessor opens, so the transaction
// the accessor wants is ALWAYS already allocated -- openWireFrame can use
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

SlaveBusConfig makeLegacyConfig()
{
    auto cfg                     = makeConfig();
    cfg.legacy_wire_frame_window = true;
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
// device model. RAII teardown (~SlaveBus_espidf calls teardownBackend()) frees the
// captured ISR handle and resets the fake model's interrupt mask, so the
// next Harness's init() sees a clean baseline.
struct Harness {
    SlaveBus_espidf bus;
    SlaveStreamAccessor acc{bus};

    Harness()
    {
        auto r = bus.init(makeLegacyConfig());
        if (!r.has_value()) {
            ADD_FAILURE() << "bus.init failed: err=" << m5::hal::v2::error::toString(r.error());
        }
    }
};

SlaveAccessConfig byteQueueConfig()
{
    SlaveAccessConfig config;
    config.tx_mode = m5::hal::v2::slave::QueueMode::Byte;
    config.rx_mode = m5::hal::v2::slave::QueueMode::Byte;
    return config;
}

struct QueueHarness {
    SlaveBus_espidf bus;
    m5::hal::v2::slave::StaticSlaveQueueStorage<128, 128, 1, 1> queues;
    m5::hal::v2::i2c::StaticI2cSegmentStorage<1> segments;
    SlaveAccessor acc{bus, queues.tx(), queues.rx(), segments.storage(), byteQueueConfig()};

    QueueHarness()
    {
        auto r = bus.init(makeConfig());
        if (!r.has_value()) ADD_FAILURE() << "bus.init failed: " << m5::hal::v2::error::toString(r.error());
    }
};

}  // namespace

TEST(EspidfI2cSlaveQueuedHostHarness, ByteLifecycleRxAndEnd)
{
    QueueHarness h;
    auto begun = h.acc.beginAccess(100);
    ASSERT_TRUE(begun.has_value()) << m5::hal::v2::error::toString(begun.error());
    const uint8_t wire[] = {0x11, 0x22, 0x33};
    primeRxFifo(wire);
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, false);

    uint8_t received[sizeof(wire)]{};
    auto read = h.acc.read({received, sizeof(received)});
    ASSERT_TRUE(read.has_value());
    EXPECT_EQ(*read, sizeof(wire));
    EXPECT_TRUE(std::equal(std::begin(wire), std::end(wire), received));
    EXPECT_TRUE(h.acc.endAccess(100).has_value());
    EXPECT_FALSE(h.acc.inAccess());
}

TEST(EspidfI2cSlaveQueuedHostHarness, StuckWorkerEndDeletesTaskDetachesAndKeepsBackendBroken)
{
    QueueHarness h;
    const uint32_t deleted_before = m5hal_hostharness::deletedTaskCount();
    ASSERT_TRUE(h.acc.beginAccess(100).has_value());
    h.bus.testHoldQueuedWorkerSession();
    auto ended = h.acc.endAccess(0);
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_TRUE(h.bus.testQueuedEndpointsDetached());
    EXPECT_FALSE(h.bus.testQueuedProducerTaskPresent());
    EXPECT_EQ(m5hal_hostharness::deletedTaskCount(), deleted_before + 1);
    EXPECT_FALSE(h.acc.beginAccess(0).has_value());
    h.bus.testReleaseHeldQueuedWorkerSession();
    ASSERT_TRUE(h.bus.close().has_value());
    ASSERT_TRUE(h.bus.init(makeConfig()).has_value());
    ASSERT_TRUE(h.acc.beginAccess(100).has_value());
    ASSERT_TRUE(h.acc.endAccess(100).has_value());
}

TEST(EspidfI2cSlaveQueuedHostHarness, BeginRollbackStopsStuckWorkerAndDetaches)
{
    QueueHarness h;
    const uint32_t deleted_before = m5hal_hostharness::deletedTaskCount();
    h.bus.testFailNextQueuedBeginWithHeldWorker();
    auto begun = h.acc.beginAccess(100);
    ASSERT_FALSE(begun.has_value());
    EXPECT_EQ(begun.error(), m5::hal::v2::error::error_t::INVALID_STATE);
    EXPECT_TRUE(h.bus.testQueuedEndpointsDetached());
    EXPECT_FALSE(h.bus.testQueuedProducerTaskPresent());
    EXPECT_EQ(m5hal_hostharness::deletedTaskCount(), deleted_before + 1);
    h.bus.testReleaseHeldQueuedWorkerSession();
}

TEST(EspidfI2cSlaveQueuedHostHarness, TxLoadDoesNotPopUntilStopAndSuffixIsRetained)
{
    QueueHarness h;
    const uint8_t reply[] = {0xA0, 0xA1, 0xA2, 0xA3};
    ASSERT_TRUE(h.acc.write({reply, sizeof(reply)}).has_value());
    auto begun = h.acc.beginAccess(100);
    ASSERT_TRUE(begun.has_value()) << m5::hal::v2::error::toString(begun.error());

    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, true, I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH);
    EXPECT_EQ(h.acc.writable(), 128u - sizeof(reply));
    ASSERT_GE(fakeHw().txfifo_count, sizeof(reply));
    // Two bytes reached the master and the next byte moved into the shifter
    // before the early NACK. FIFO occupancy alone cannot confirm that third byte.
    fakeHw().txfifo_count = i2c_dev_t::kFifoLen - 3;
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, true);
    EXPECT_EQ(h.acc.writable(), 126u);
    EXPECT_NE(fakeHw().int_ena & I2C_SLAVE_STRETCH_INT_ENA_M, 0u);

    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, true, I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH);
    EXPECT_FALSE(fakeHw().stretch_active);
    ASSERT_GE(fakeHw().txfifo_count, 2u);
    EXPECT_EQ(fakeHw().txfifo[0], 0xA2);
    EXPECT_EQ(fakeHw().txfifo[1], 0xA3);
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, true);
    EXPECT_TRUE(h.acc.endAccess(100).has_value());
}

TEST(EspidfI2cSlaveQueuedHostHarness, EndAccessDrainsResidualHardwareRx)
{
    QueueHarness h;
    EXPECT_EQ(fakeHw().slave_address, 0x3FFu);
    EXPECT_TRUE(fakeHw().slave_address_10bit);
    EXPECT_EQ(fakeHw().int_ena & I2C_LL_INTR_MASK, 0u);
    const unsigned updates_before_begin = fakeHw().update_count;
    auto begun                          = h.acc.beginAccess(100);
    ASSERT_TRUE(begun.has_value()) << m5::hal::v2::error::toString(begun.error());
    EXPECT_GT(fakeHw().update_count, updates_before_begin);
    EXPECT_EQ(fakeHw().slave_address, makeConfig().address);
    EXPECT_FALSE(fakeHw().slave_address_10bit);

    const uint8_t wire[] = {0x31, 0x32, 0x33};
    primeRxFifo(wire);
    const unsigned updates_before_end = fakeHw().update_count;
    ASSERT_TRUE(h.acc.endAccess(100).has_value());
    EXPECT_GT(fakeHw().update_count, updates_before_end);
    EXPECT_EQ(fakeHw().slave_address, 0x3FFu);
    EXPECT_TRUE(fakeHw().slave_address_10bit);

    uint8_t received[sizeof(wire)]{};
    auto read = h.acc.read({received, sizeof(received)});
    ASSERT_TRUE(read.has_value());
    EXPECT_EQ(*read, sizeof(wire));
    EXPECT_TRUE(std::equal(std::begin(wire), std::end(wire), received));
}

TEST(EspidfI2cSlaveQueuedHostHarness, EndAccessHardStopsBusyWireAndKeepsBackendBroken)
{
    QueueHarness h;
    ASSERT_TRUE(h.acc.beginAccess(100).has_value());
    fakeHw().bus_busy = true;

    auto ended = h.acc.endAccess(0);
    ASSERT_FALSE(ended.has_value());
    EXPECT_EQ(ended.error(), m5::hal::v2::error::error_t::TIMEOUT_ERROR);
    EXPECT_FALSE(h.acc.inAccess());
    EXPECT_FALSE(h.acc.beginAccess(0).has_value());

    // Keep the process-wide fake sane for the next test's fresh init.
    fakeHw().bus_busy = false;
}

TEST(EspidfI2cSlaveQueuedHostHarness, StopTailWithoutBridgeSpaceIsAccountedAndBreaksAcceptance)
{
    QueueHarness h;
    ASSERT_TRUE(h.acc.beginAccess(100).has_value());
    std::array<uint8_t, i2c_dev_t::kFifoLen> batch{};
    std::iota(batch.begin(), batch.end(), uint8_t{0});

    // Four batches fill the caller RX queue. Four more remain staged in the
    // fixed ISR bridge because the worker cannot replay them yet.
    for (size_t i = 0; i < 8; ++i) {
        primeRxFifo(batch);
        fireIsr(I2C_RXFIFO_WM_INT_ENA_M, false);
    }
    EXPECT_EQ(h.acc.readable(), 128u);

    // The STOP tail has nowhere safe to live. It must be explicitly dropped,
    // accounted, and fence subsequent address matches instead of leaking into
    // the next transaction.
    primeRxFifo(batch);
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, false);
    EXPECT_EQ(fakeHw().rxfifo_count, 0u);
    EXPECT_EQ(fakeHw().slave_address, 0x3FFu);
    EXPECT_TRUE(fakeHw().slave_address_10bit);
    EXPECT_GE(h.acc.rxStatus().dropped_bytes, batch.size());

    auto ended = h.acc.endAccess(100);
    EXPECT_FALSE(ended.has_value());
    EXPECT_FALSE(h.acc.beginAccess(0).has_value());
}

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

    ASSERT_TRUE(h.acc.openWireFrame(kBeginNonBlocking).has_value());
    uint8_t rx[sizeof(written)] = {};
    auto read                   = h.acc.read(DataSpan{rx, sizeof(rx)});
    ASSERT_TRUE(read.has_value()) << m5::hal::v2::error::toString(read.error());
    EXPECT_EQ(*read, sizeof(written));
    EXPECT_TRUE(std::equal(std::begin(written), std::end(written), rx));

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP
    auto complete = h.acc.wireFrameComplete();
    ASSERT_TRUE(complete.has_value());
    EXPECT_TRUE(*complete);
    EXPECT_TRUE(h.acc.closeWireFrame().has_value());
    EXPECT_EQ(h.bus.rxOverflowCount(), 0u);
}

TEST(EspidfI2cSlaveHostHarness, BasicReadTransactionServesComposedReply)
{
    Harness h;

    // Pure read: address-match stretch with no prior write phase.
    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, /*is_read=*/true, I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH);

    ASSERT_TRUE(h.acc.openWireFrame(kBeginNonBlocking).has_value());
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
    EXPECT_TRUE(h.acc.closeWireFrame().has_value());
}

TEST(EspidfI2cSlaveHostHarness, CloseWireFrameReleasesRxFullHold)
{
    Harness h;

    std::vector<uint8_t> fill(SlaveBus_espidf::kRxCapacity);
    std::iota(fill.begin(), fill.end(), uint8_t{0});
    primeRxFifo(fill);
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);

    ASSERT_TRUE(h.acc.openWireFrame(kBeginNonBlocking).has_value());

    primeRxFifo(std::vector<uint8_t>{0xA5});
    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, /*is_read=*/false, I2C_SLAVE_STRETCH_CAUSE_RX_FULL);

    EXPECT_TRUE(fakeHw().stretch_active);
    EXPECT_EQ(fakeHw().int_ena & (I2C_SLAVE_STRETCH_INT_ENA_M | I2C_RXFIFO_WM_INT_ENA_M), 0u)
        << "rx_full hold masks the level sources until the held transaction is drained or discarded";

    auto ended = h.acc.closeWireFrame();
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

    ASSERT_TRUE(h.acc.openWireFrame(kBeginNonBlocking).has_value());
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

    EXPECT_TRUE(h.acc.closeWireFrame().has_value());
}

TEST(EspidfI2cSlaveHostHarness, ForcedCloseCompletesCleanupAndAllowsReinit)
{
    using m5hal_hostharness::I2cClockEvent;
    m5hal_hostharness::resetI2cClockTrace();

    SlaveBus_espidf bus;
    auto init = bus.init(makeConfig());
    ASSERT_TRUE(init.has_value()) << m5::hal::v2::error::toString(init.error());
    EXPECT_EQ(m5hal_hostharness::i2cClockTrace(),
              (std::vector<I2cClockEvent>{I2cClockEvent::BusEnable, I2cClockEvent::ControllerEnable}));

    auto closed = bus.close();
    ASSERT_TRUE(closed.has_value());
    EXPECT_EQ(m5hal_hostharness::i2cClockTrace(),
              (std::vector<I2cClockEvent>{I2cClockEvent::BusEnable, I2cClockEvent::ControllerEnable,
                                          I2cClockEvent::ControllerDisable, I2cClockEvent::BusDisable}));

    auto reinit = bus.init(makeConfig());
    ASSERT_TRUE(reinit.has_value());
    EXPECT_EQ(m5hal_hostharness::i2cClockTrace(),
              (std::vector<I2cClockEvent>{I2cClockEvent::BusEnable, I2cClockEvent::ControllerEnable,
                                          I2cClockEvent::ControllerDisable, I2cClockEvent::BusDisable,
                                          I2cClockEvent::BusEnable, I2cClockEvent::ControllerEnable}));
}

TEST(EspidfI2cSlaveHostHarness, SuccessfulCloseDisablesClocksInReverseOrder)
{
    using m5hal_hostharness::I2cClockEvent;
    m5hal_hostharness::resetI2cClockTrace();

    SlaveBus_espidf bus;
    auto init = bus.init(makeConfig());
    ASSERT_TRUE(init.has_value()) << m5::hal::v2::error::toString(init.error());
    EXPECT_EQ(m5hal_hostharness::i2cClockTrace(),
              (std::vector<I2cClockEvent>{I2cClockEvent::BusEnable, I2cClockEvent::ControllerEnable}));

    m5hal_hostharness::runCreatedTaskOnNextDelay();
    auto closed = bus.close();
    ASSERT_TRUE(closed.has_value()) << m5::hal::v2::error::toString(closed.error());
    EXPECT_EQ(m5hal_hostharness::i2cClockTrace(),
              (std::vector<I2cClockEvent>{I2cClockEvent::BusEnable, I2cClockEvent::ControllerEnable,
                                          I2cClockEvent::ControllerDisable, I2cClockEvent::BusDisable}));
}

// ---------------------------------------------------------------------------
// cc133e89: a reply composed while servicing a WRITE transaction must not
// leak onto the wire if a zero-gap READ address-match beats the accessor's
// closeWireFrame() of that write. Regresses the `_open == _current`
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
    ASSERT_TRUE(h.acc.openWireFrame(kBeginNonBlocking).has_value());
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
    // observed wireFrameComplete() and called closeWireFrame(). This is
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

    ASSERT_TRUE(h.acc.closeWireFrame().has_value());

    // The accessor opens the transaction the ISR allocated for the new read
    // and composes a fresh reply; this one DOES release the hold.
    ASSERT_TRUE(h.acc.openWireFrame(kBeginNonBlocking).has_value());
    const uint8_t fresh_reply[] = {0xBE, 0xEF, 0x01};
    auto fresh                  = h.acc.write(ConstDataSpan{fresh_reply, sizeof(fresh_reply)});
    ASSERT_TRUE(fresh.has_value()) << m5::hal::v2::error::toString(fresh.error());
    EXPECT_EQ(*fresh, sizeof(fresh_reply));

    ASSERT_EQ(fakeHw().txfifo_count, sizeof(fresh_reply));
    EXPECT_TRUE(std::equal(std::begin(fresh_reply), std::end(fresh_reply), fakeHw().txfifo));
    EXPECT_FALSE(fakeHw().stretch_active);

    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/true);  // STOP
    EXPECT_TRUE(h.acc.closeWireFrame().has_value());
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

    ASSERT_TRUE(h.acc.openWireFrame(kBeginNonBlocking).has_value());
    std::vector<uint8_t> all(first_batch.size() + tail_batch.size());
    auto read = h.acc.read(DataSpan{all.data(), all.size()});
    ASSERT_TRUE(read.has_value()) << m5::hal::v2::error::toString(read.error());
    EXPECT_EQ(*read, all.size());

    std::vector<uint8_t> expected = first_batch;
    expected.insert(expected.end(), tail_batch.begin(), tail_batch.end());
    EXPECT_EQ(all, expected);
    EXPECT_EQ(h.bus.rxOverflowCount(), 0u);

    EXPECT_TRUE(h.acc.closeWireFrame().has_value());
}

// ---------------------------------------------------------------------------
// When all transaction slots are occupied, allocating the next wire
// transaction recycles the oldest non-open slot. Any unread RX backlog in that
// slot is genuinely lost and must therefore contribute its byte count to
// rxOverflowCount(). A zero-RX transaction must contribute nothing.
//
// A partially read transaction cannot be an eviction victim through the public
// API: read() requires openWireFrame(), which makes it `_open`, and the
// allocator explicitly excludes `_open`; closeWireFrame() discards/frees it.
// Thus the host harness can exercise the all-unread and zero-unread boundaries,
// while the production subtraction remains defensive against rx_read > rx_size.
// ---------------------------------------------------------------------------

TEST(EspidfI2cSlaveHostHarness, TransactionSlotEvictionCountsAllUnreadRxBytes)
{
    Harness h;

    const std::vector<uint8_t> oldest = {0x10, 0x20, 0x30, 0x40, 0x50};
    primeRxFifo(oldest);
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP

    // Fill the remaining slots with completed, unread write transactions.
    for (size_t i = 1; i < SlaveBus_espidf::kMaxTransactions; ++i) {
        primeRxFifo(std::vector<uint8_t>{static_cast<uint8_t>(0x80 + i)});
        fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
        fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP
    }
    ASSERT_EQ(h.bus.rxOverflowCount(), 0u);

    // Allocating transaction kMaxTransactions + 1 recycles the oldest slot.
    primeRxFifo(std::vector<uint8_t>{0xEE});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    EXPECT_EQ(h.bus.rxOverflowCount(), oldest.size());
}

TEST(EspidfI2cSlaveHostHarness, TransactionSlotEvictionWithZeroUnreadRxAddsNothing)
{
    Harness h;

    // A completed pure-read transaction occupies the oldest slot with rx_size=0.
    fireIsr(I2C_SLAVE_STRETCH_INT_ENA_M, /*is_read=*/true, I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH);
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/true);  // STOP

    // Fill the other slots with one unread byte each.
    for (size_t i = 1; i < SlaveBus_espidf::kMaxTransactions; ++i) {
        primeRxFifo(std::vector<uint8_t>{static_cast<uint8_t>(0x90 + i)});
        fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
        fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP
    }

    // First recycle drops the zero-RX slot; the byte counter must stay unchanged.
    primeRxFifo(std::vector<uint8_t>{0xE0});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    EXPECT_EQ(h.bus.rxOverflowCount(), 0u);
    fireIsr(I2C_TRANS_COMPLETE_INT_ENA_M, /*is_read=*/false);  // STOP

    // The next recycle drops the one-byte transaction that is now oldest.
    primeRxFifo(std::vector<uint8_t>{0xE1});
    fireIsr(I2C_RXFIFO_WM_INT_ENA_M, /*is_read=*/false);
    EXPECT_EQ(h.bus.rxOverflowCount(), 1u);
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
    ASSERT_TRUE(h.acc.openWireFrame(kBeginNonBlocking).has_value());
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
    // holds it open (it has not called closeWireFrame() yet).
    auto complete = h.acc.wireFrameComplete();
    ASSERT_TRUE(complete.has_value());
    EXPECT_TRUE(*complete);
    ASSERT_TRUE(h.acc.closeWireFrame().has_value());

    // Opening the next transaction must reach the NEW slot the merged
    // pass's STRETCH branch allocated -- readableBytes()==0 (a pure read
    // never received rx bytes) proves it is not the write transaction's
    // slot, which had 1 unread byte's worth of history.
    ASSERT_TRUE(h.acc.openWireFrame(kBeginNonBlocking).has_value());
    auto readable = h.acc.readableBytes();
    ASSERT_TRUE(readable.has_value());
    EXPECT_EQ(*readable, 0u) << "must not reuse the completed write transaction's slot";

    EXPECT_TRUE(h.acc.closeWireFrame().has_value());
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    m5hal_test_support::installGtestWatchdog();
    return RUN_ALL_TESTS();
}
