// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_SLAVE_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_SLAVE_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v2/i2c/slave.hpp"

#if defined(ESP_PLATFORM)
#include <soc/soc_caps.h>
#endif

// The espidf slave backend ships in one of two flavors:
//   * LL stretch primitive: direct LL control with a real SCL clock stretch, so
//     reads can hold the master while an accessor composes the reply. This avoids
//     the v2 driver's write-then-read 0x85 corruption. Available on every SoC that
//     can report the stretch cause (esp32 s2/s3/c3/c6/h2/p4); the classic ESP32
//     has no slave stretch (i2c_ll_slave_enable_scl_stretch is a no-op there).
//   * v2 driver (M5HAL_ESPIDF_I2C_HAS_SLAVE_V2, e.g. classic ESP32 without
//     stretch-cause): the existing best-effort fill-only path, kept verbatim.
// The transaction-window state machine is HW-independent and shared by both.
//
// The LL path is written against the i2c_ll_* helpers (which abstract the
// SoC-specific register layout) plus a handful of portable direct pokes, so it
// compiles on all stretch-cause SoCs. It needs three headers that are not present
// on every SoC/IDF combo, so each is guarded with __has_include rather than an
// explicit SoC list -- the capability + header probes self-select the SoCs the
// code can actually build on:
//   * soc/i2c_periph.h  -- i2c_periph_signal for the SCL-output-route fix
//                          (IDF 6 relocates this shared header)
//   * soc/i2c_struct.h  -- the few portable direct-poke fields
//   * hal/i2c_ll.h      -- the i2c_ll_* helpers (absent on preview targets that
//                          report the capability but ship no I2C HAL yet, e.g.
//                          esp32h4 in IDF 5.5 -> falls back to no LL backend)
// CI coverage: quick builds s3/c3/c6 (PIO) + h2/p4 (idf); full builds all 10
// official chips on release-v5.5 AND release-v6.0, plus the preview SoCs h21/h4
// on v6.0 (idf component lane; h4 has no LL slave backend per above). NOTE: only
// ESP32-S3 is HW-validated -- the other stretch-cause SoCs are compile-verified
// (capability present) but not yet bench-tested.
#if defined(ESP_PLATFORM) && (defined(SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE) && SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE) && \
    __has_include(<soc/i2c_periph.h>) && __has_include(<soc/i2c_struct.h>) && __has_include(<hal/i2c_ll.h>)
#define M5HAL_ESPIDF_I2C_SLAVE_LL 1
#else
#define M5HAL_ESPIDF_I2C_SLAVE_LL 0
#endif

// IRAM placement for the LL stretch ISR path. By default the I2C-slave ISR and
// every function/datum it reaches live in IRAM, so the slave keeps answering its
// clock-stretched master even while the flash cache is disabled (an OTA / NVS /
// SPIFFS write running on another task). That robustness costs ~1-2 KB of IRAM.
// A build that never writes flash while the I2C slave is active can reclaim it by
// defining M5HAL_ESPIDF_I2C_SLAVE_IRAM_ISR=0: that drops both the ESP_INTR_FLAG_IRAM
// registration and the IRAM_ATTR on the ISR-reachable functions. The trade-off is
// that an interrupt arriving during a flash-cache-disabled window is deferred,
// which can stall (or time out) the master mid-transaction. Default on = safe; the
// I2C slave is clocked by an external master, so a deferred ISR strands the bus.
//
// M5HAL_I2C_SLAVE_ISR_IRAM is the attribute applied to every function reachable
// from the LL ISR (declared once, honored at the definition). It is empty unless
// the LL path is active AND the option is on, so v2-driver builds and opt-out
// builds carry no needless IRAM. M5HAL_I2C_SLAVE_ISR_INTR_FLAGS matches it for the
// esp_intr_alloc() call.
#ifndef M5HAL_ESPIDF_I2C_SLAVE_IRAM_ISR
#define M5HAL_ESPIDF_I2C_SLAVE_IRAM_ISR 1
#endif
#if M5HAL_ESPIDF_I2C_SLAVE_LL && M5HAL_ESPIDF_I2C_SLAVE_IRAM_ISR
#define M5HAL_I2C_SLAVE_ISR_IRAM       IRAM_ATTR
#define M5HAL_I2C_SLAVE_ISR_INTR_FLAGS (ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3)
#else
#define M5HAL_I2C_SLAVE_ISR_IRAM
#define M5HAL_I2C_SLAVE_ISR_INTR_FLAGS (ESP_INTR_FLAG_LEVEL3)
#endif

#if defined(ESP_PLATFORM) && (M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_HAS_SLAVE_V2)

#if M5HAL_ESPIDF_I2C_SLAVE_LL
#include <esp_intr_alloc.h>
#include <hal/i2c_types.h>
#include <soc/i2c_struct.h>
#else
#include <driver/i2c_slave.h>
#endif
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace m5::hal::v2::i2c {

class SlaveBus_espidf : public i2c::ISlaveBus {
public:
    static constexpr size_t kMaxTransactions = 4;
    // RX ring = 32B (= HW FIFO depth). With write-direction back-pressure (RX_FULL
    // stretch held while the ring is full, masked via the M1 hold machine), this is
    // not a correctness cap -- nothing is dropped -- only a throughput knob: a
    // smaller ring stretches the master more often. 32 is confirmed by HW measurement
    // with the event-driven consumer (the serve() task drains the ring promptly on the
    // ISR's activity notification, so a 32B ring rarely stalls even at 800kHz).
    // TX is also a power-of-two ring: write() tops it up as the master drains the
    // reply, so a reply larger than kTxCapacity streams across one read. TX stays 64
    // to keep the RegMap single 64B reply compose in one window (shrinking it waits
    // on a RegMap reply loop).
    static constexpr size_t kRxCapacity = 32;
    static constexpr size_t kTxCapacity = 64;
    // rx[]/tx[] are indexed as rings via (idx & (cap - 1)); require powers of two.
    static_assert((kRxCapacity & (kRxCapacity - 1)) == 0, "kRxCapacity must be a power of two");
    static_assert((kTxCapacity & (kTxCapacity - 1)) == 0, "kTxCapacity must be a power of two");
#if !M5HAL_ESPIDF_I2C_SLAVE_LL
    static constexpr size_t kDriverRxDepth = 128;
    static constexpr size_t kDriverTxDepth = 128;
#endif

    SlaveBus_espidf() = default;
    ~SlaveBus_espidf() override
    {
        (void)release();
    }

    result_t<void> init(const i2c::SlaveBusConfig& cfg) override;
    result_t<void> release(void) override;
    result_t<void> beginTransaction(bus::IAccessor* owner, uint32_t timeout_ms = 0) override;
    result_t<void> endTransaction(bus::IAccessor* owner) override;
    result_t<size_t> read(bus::IAccessor* owner, data::DataSpan dst) override;
    result_t<size_t> write(bus::IAccessor* owner, data::ConstDataSpan src) override;
    result_t<size_t> readableBytes(bus::IAccessor* owner) override;
    result_t<bool> transactionComplete(bus::IAccessor* owner) override;
    service::IService* service() override
    {
        return nullptr;
    }
    // Event-driven consumer wait: block the serve() task on a notification the ISR
    // gives on any RX/TX/STOP activity (short safety timeout), so a fast master is
    // drained promptly. Overrides the base 1 ms poll.
    result_t<bool> waitForActivity(bus::IAccessor* owner, uint32_t timeout_ms) override;

    // Number of received write bytes dropped. On the streaming LL path the RX is a
    // kRxCapacity-deep power-of-two ring, so this counts bytes lost only when the
    // consumer fell behind and the UNREAD backlog reached kRxCapacity (with timely
    // read()s a single transaction may receive far more than kRxCapacity total). On
    // the non-LL callback path the whole transaction arrives post-STOP with no
    // draining, so kRxCapacity is a hard per-transaction cap and this counts the
    // truncated tail. Either way, non-zero means received bytes were dropped.
    size_t rxOverflowCount() const
    {
        return _rx_overflow_count;
    }

private:
    // ---- Shared transaction-window state machine (HW independent) ----------
    struct Transaction {
        bool in_use             = false;
        bool complete           = false;
        bool opened             = false;
        uint8_t rx[kRxCapacity] = {};
        size_t rx_size          = 0;
        size_t rx_read          = 0;
        uint8_t tx[kTxCapacity] = {};
        size_t tx_size          = 0;
        size_t tx_read          = 0;
        uint32_t seq            = 0;
    };

    static void taskThunk(void* arg);

    void resetStateLocked();
    // allocateTransactionLocked and notifyTaskFromISR are reached from the LL ISR
    // (drainRxLocked falls into allocate when _current is null; handleIsr allocates
    // a fresh slot for a pure read and notifies the responder task on a read
    // stretch). With ESP_INTR_FLAG_IRAM that whole call chain must be in IRAM, so
    // they carry the ISR-IRAM attribute (empty on the v2 path / opt-out builds).
    Transaction* M5HAL_I2C_SLAVE_ISR_IRAM allocateTransactionLocked();
    Transaction* newestTransactionLocked();
    Transaction* oldestOpenableTransactionLocked();
    void discardTransactionLocked(Transaction& txn);
    bool isOpenOwnerLocked(bus::IAccessor* owner) const;
    void M5HAL_I2C_SLAVE_ISR_IRAM notifyTaskFromISR(bool& task_woken);
    // Wake the serve() consumer (the task blocked in waitForActivity) on RX/TX/STOP
    // activity. Reached from the LL ISR, so it carries the ISR-IRAM attribute.
    void M5HAL_I2C_SLAVE_ISR_IRAM notifyConsumerFromISR(bool& task_woken);
    void notifyTaskFromTask();
    void requestTaskLoop();
    ::TickType_t requestWaitTicksLocked(::TickType_t now_tick) const;
    bool collectPendingWrite(uint8_t* dst, size_t& len, bool& generated_fill);
    void markWriteCommitted(size_t len);
    static size_t readableBytesOf(const Transaction& txn);

    ::TaskHandle_t _task = nullptr;
    // Wakes the serve() consumer (the app task blocked in waitForActivity) on RX/TX/STOP
    // activity, so it drains without a fixed poll delay. A DEDICATED binary semaphore --
    // not the consumer task's direct notification -- so it never collides with an
    // app that uses xTaskNotify on its own task (any notification index): the prior
    // direct-notify design raced (lost / spurious wakeups) against such an app. Created
    // lazily on the first waitForActivity (task context) and lives for the bus lifetime;
    // the ISR only gives it (xSemaphoreGiveFromISR) once it exists. Independent of
    // CONFIG_FREERTOS_TASK_NOTIFICATION_ARRAY_ENTRIES (which defaults to 1, so a
    // dedicated notification index is not available without a non-default config).
    ::SemaphoreHandle_t _consumer_sem = nullptr;
    ::StaticSemaphore_t _consumer_sem_buf;
    ::portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    Transaction _transactions[kMaxTransactions];
    Transaction* _current       = nullptr;
    Transaction* _open          = nullptr;
    bus::IAccessor* _open_owner = nullptr;
    // A read request is outstanding from the read address-match until STOP. This
    // is independent of the stretch hold: after a stretch is released by a fill-
    // byte fallback (budget expired), _hold_kind returns to none but _request_pending
    // stays true until STOP, so a late write() may still notify the responder
    // task. That wake-up is a harmless no-op (the task finds _hold_kind == none
    // and loops back); _request_pending is cleared in the STOP cleanup.
    bool _request_pending      = false;
    bool _task_stop            = false;
    bool _task_running         = false;
    bool _pending_fill         = false;
    size_t _pending_commit_len = 0;
    uint32_t _next_seq         = 1;
    size_t _rx_overflow_count  = 0;
    ::TickType_t _request_tick = 0;

    // ---- HW-facing layer (path specific) -----------------------------------
#if M5HAL_ESPIDF_I2C_SLAVE_LL
    // ESP32-S3 (and SoCs with stretch-cause): direct LL control. The ISR drains
    // RX into the current transaction and, on a read stretch, wakes the responder
    // task which fills the TX FIFO from the open transaction's tx queue and then
    // releases the stretch.
    static void M5HAL_I2C_SLAVE_ISR_IRAM isrThunk(void* arg);
    void M5HAL_I2C_SLAVE_ISR_IRAM handleIsr();
    // Drain `count` RX-FIFO bytes into the current transaction's rx ring. With
    // can_hold=true (mid-transaction) it pulls only as many bytes as the ring has
    // free space for and LEAVES the rest in the HW FIFO, returning true so the
    // caller asserts the RX_FULL back-pressure hold (no byte is dropped). With
    // can_hold=false (STOP / no later lift) it drains the whole count, dropping any
    // tail that no longer fits (surfaced via rxOverflowCount). Returns true iff it
    // stopped early on a full ring with bytes still pending in the FIFO.
    bool M5HAL_I2C_SLAVE_ISR_IRAM drainRxLocked(uint32_t count, bool can_hold);
    // Refills _resp from the open transaction's tx ring (only when _resp is fully
    // sent). Reached from the LL ISR's TX_EMPTY continuation (to stream a >FIFO reply
    // from the tx ring without task latency) as well as the responder task, so it
    // carries the ISR-IRAM attribute.
    void M5HAL_I2C_SLAVE_ISR_IRAM snapshotResponseLocked();
    void M5HAL_I2C_SLAVE_ISR_IRAM fillTxFromRespLocked();
    void M5HAL_I2C_SLAVE_ISR_IRAM enterTxHoldFromIsrLocked(bool& task_woken, bool address_read);
    // Write-direction back-pressure: assert HoldKind::rx_full and MASK the stretch +
    // RX-water-mark interrupts (without clearing the RX_FULL stretch the HW already
    // holds), so the ISR does not spin on the still-full FIFO. read() lifts it once
    // it drains the ring. Idempotent (a second RX_FULL in the same hold is a no-op).
    void M5HAL_I2C_SLAVE_ISR_IRAM enterRxHoldFromIsrLocked();
    uint32_t M5HAL_I2C_SLAVE_ISR_IRAM clearHoldLocked();
    void M5HAL_I2C_SLAVE_ISR_IRAM enableMaskedInterrupts(uint32_t mask);
    void restorePins();

    ::i2c_dev_t* _hw      = nullptr;
    ::intr_handle_t _intr = nullptr;
    int _pin_scl          = -1;
    int _pin_sda          = -1;
    enum class HoldKind : uint8_t{none, address_read, tx_empty, rx_full};
    HoldKind _hold_kind      = HoldKind::none;
    uint32_t _masked_intrs   = 0;
    uint32_t _baseline_intrs = 0;

    // Backend-owned snapshot of the composed reply for the in-flight read. The
    // accessor writes its reply into the open transaction's tx queue; on the first
    // fill we copy it here and serve the whole read (including the TX_EMPTY refills
    // a >FIFO read needs) from this buffer. That decouples the byte stream the
    // master is clocking from the accessor's transaction lifecycle -- the app may
    // endTransaction() right after write(), and a refill must not chase a freed
    // transaction (which raced as intermittent mid-read underruns).
    uint8_t _resp[kTxCapacity] = {};
    size_t _resp_len           = 0;
    size_t _resp_pos           = 0;
#else
    // Classic ESP32 (no stretch-cause): keep the v2 driver fill-only backend.
    static bool onReceive(::i2c_slave_dev_handle_t handle, const ::i2c_slave_rx_done_event_data_t* evt_data,
                          void* user_data);
    static bool onRequest(::i2c_slave_dev_handle_t handle, const ::i2c_slave_request_event_data_t* evt_data,
                          void* user_data);

    ::i2c_slave_dev_handle_t _handle = nullptr;
#endif
};

}  // namespace m5::hal::v2::i2c

#endif

#endif
