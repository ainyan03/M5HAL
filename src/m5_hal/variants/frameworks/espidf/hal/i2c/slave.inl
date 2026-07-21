// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_SLAVE_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_SLAVE_INL

#include "slave.hpp"
#include "../../../../../hal/v2/i2c/slave_accessor.hpp"

// M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS: mirrors the same gate in slave.hpp so this .inl
// compiles unmodified under the native host regression harness.
#if (defined(ESP_PLATFORM) || defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)) && \
    (M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_SLAVE_LL_BE || M5HAL_ESPIDF_I2C_HAS_SLAVE_V2)

#include "../../detail/esp_err_map.hpp"

#include <algorithm>
#include <esp_err.h>
#include <soc/soc_caps.h>
#include <string.h>
#if defined(ESP_PLATFORM) && __has_include(<esp_memory_utils.h>)
#include <esp_memory_utils.h>
#define M5HAL_DETAIL_I2C_SLAVE_HAS_INTERNAL_PTR_CHECK_ 1
#else
#define M5HAL_DETAIL_I2C_SLAVE_HAS_INTERNAL_PTR_CHECK_ 0
#endif

#include "../../../freertos/hal/runtime/time.hpp"
#include <freertos/task.h>

#ifndef M5HAL_DEBUG_ESPIDF_I2C_SLAVE_GPIO_MARKERS
#define M5HAL_DEBUG_ESPIDF_I2C_SLAVE_GPIO_MARKERS 0
#endif
#ifndef M5HAL_DEBUG_ESPIDF_I2C_SLAVE_TX_FILL_MARKER_PIN
#define M5HAL_DEBUG_ESPIDF_I2C_SLAVE_TX_FILL_MARKER_PIN 6
#endif
#ifndef M5HAL_DEBUG_ESPIDF_I2C_SLAVE_RX_DRAIN_MARKER_PIN
#define M5HAL_DEBUG_ESPIDF_I2C_SLAVE_RX_DRAIN_MARKER_PIN 7
#endif
#ifndef M5HAL_DEBUG_ESPIDF_I2C_SLAVE_STRETCH_MARKER_PIN
#define M5HAL_DEBUG_ESPIDF_I2C_SLAVE_STRETCH_MARKER_PIN 13
#endif
#ifndef M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_TX_WATERMARK
#define M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_TX_WATERMARK 0
#endif
#ifndef M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_CONTROLLER_CLOCK
#define M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_CONTROLLER_CLOCK 0
#endif

#if M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_SLAVE_LL_BE
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <hal/i2c_ll.h>
#include <soc/i2c_periph.h>
#if __has_include(<esp_private/periph_ctrl.h>)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif

#if M5HAL_DEBUG_ESPIDF_I2C_SLAVE_GPIO_MARKERS
// Non-perturbing GPIO markers for logic-analyzer correlation (IRAM-safe gpio_ll).
// Off by default; build the slave with -DM5HAL_DEBUG_ESPIDF_I2C_SLAVE_GPIO_MARKERS=1 and wire the
// pins to the analyzer alongside SCL/SDA to see, for each 32-byte pause, WHO is
// pausing the bus:
//   TXFILL  (pin 6)  HIGH while the slave loads the TX FIFO  -> a read-side gap that
//                    lines up with this pulse is the slave refilling (TX_EMPTY).
//   RXDRAIN (pin 7)  HIGH while the slave reads the RX FIFO   -> a write-side gap on
//                    this pulse is the slave draining; a gap with NO pulse is the
//                    master pausing (its own HW FIFO refill), not the slave.
//   STRETCH (pin 13) HIGH while the stretch-cause ISR services a hold/refill.
#include <hal/gpio_ll.h>
#define M5HAL_DETAIL_I2C_SLAVE_MARK_HIGH_(pin) ::gpio_ll_set_level(&GPIO, (::gpio_num_t)(pin), 1)
#define M5HAL_DETAIL_I2C_SLAVE_MARK_LOW_(pin)  ::gpio_ll_set_level(&GPIO, (::gpio_num_t)(pin), 0)
#else
#define M5HAL_DETAIL_I2C_SLAVE_MARK_HIGH_(pin) ((void)0)
#define M5HAL_DETAIL_I2C_SLAVE_MARK_LOW_(pin)  ((void)0)
#endif
#endif

// The proactive TX water-mark top-up (refills the TX FIFO at FIFO/2 before it empties,
// smoothing the read-side 32-byte pause that the reactive TX_EMPTY stretch otherwise
// causes) can be turned off for A/B diagnosis with -DM5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_TX_WATERMARK=1.
// Default = on. With it off the TX FIFO is only refilled reactively on TX_EMPTY.
#if M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_TX_WATERMARK
#define M5HAL_DETAIL_I2C_SLAVE_TX_WATERMARK_ 0
#else
#define M5HAL_DETAIL_I2C_SLAVE_TX_WATERMARK_ 1
#endif

namespace m5::hal::v2::i2c {

namespace {
namespace impl_espidf_slave {

#if !(M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_SLAVE_LL_BE)
error::error_t mapEspErr(::esp_err_t err)
{
    switch (err) {
        case ESP_ERR_NOT_FOUND:
            return error::error_t::I2C_NO_ACK;
        case ESP_ERR_NOT_SUPPORTED:
            return error::error_t::UNSUPPORTED;
        default:
            return ::m5::variants::frameworks::espidf::detail::mapEspErrCommon(err, error::error_t::I2C_BUS_ERROR);
    }
}
#endif

::TickType_t ticks(uint32_t timeout_ms)
{
    return ::m5::hal::v2::detail::timeoutMsToTicks(timeout_ms);
}

// Resolves SlaveBusConfig::controller (a claimed hardware controller index,
// or -1 for "backend default -- ledger-external, self-responsibility") to a
// validated port number. -1 becomes port 0, the pre-claim hardcoded default.
// Any other negative value, an out-of-range index, or an LP_I2C index is
// rejected: clock-stretch / ISR slave mode drives only a plain HP instance.
// ESP-IDF numbers LP_I2C ports after all HP ports (SOC_HP_I2C_NUM is the
// HP-only count on a chip that splits the two; SOC_I2C_NUM is the sole port
// count on a chip that does not), so rejecting everything from
// SOC_HP_I2C_NUM upward (falling back to SOC_I2C_NUM when the chip has no
// HP/LP split) excludes the LP range on every chip shape.
::m5::hal::v2::result_t<int8_t> resolveSlaveControllerPort(int8_t controller)
{
    if (controller == -1) {
        return int8_t{0};
    }
    if (controller < 0) {
        // Only -1 is the documented default-port sentinel; any other
        // negative value is a corrupted / miscomputed index, not a request.
        return m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
#if defined(SOC_HP_I2C_NUM)
    constexpr int8_t kPortLimit = static_cast<int8_t>(SOC_HP_I2C_NUM);
#elif defined(SOC_I2C_NUM)
    constexpr int8_t kPortLimit = static_cast<int8_t>(SOC_I2C_NUM);
#else
    constexpr int8_t kPortLimit = 1;
#endif
    if (controller >= kPortLimit) {
        return m5::stl::make_unexpected(::m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    return controller;
}

bool waitTaskStopped(::portMUX_TYPE* mux, bool& task_running)
{
    // pdMS_TO_TICKS(1) truncates to 0 at the default 100 Hz tick rate, where
    // vTaskDelay(0) only yields. Delay a whole tick and bound the wait by
    // elapsed ticks. The budget itself truncates to 0 below 10 Hz, so floor it
    // at one tick to keep the wait non-empty at any configTICK_RATE_HZ. The
    // outer iteration cap is a defensive backstop, independent of tick
    // advancement: for any configTICK_RATE_HZ <= 1000 the tick budget is
    // <= 100, so on real hardware the elapsed-tick check fires at or before
    // the cap (above 1000 Hz the cap fires first and merely shortens the
    // wait); its real job is to guard a host that never advances
    // xTaskGetTickCount() (no scheduler), where the tick check alone would
    // spin forever instead of returning the bounded false.
    const ::TickType_t start     = ::xTaskGetTickCount();
    const ::TickType_t raw_ticks = pdMS_TO_TICKS(100);
    const ::TickType_t budget    = (raw_ticks != 0) ? raw_ticks : 1;
    for (uint32_t i = 0; i < 100; ++i) {
        bool running = false;
        portENTER_CRITICAL_SAFE(mux);
        running = task_running;
        portEXIT_CRITICAL_SAFE(mux);
        if (!running) {
            return true;
        }
        if ((::xTaskGetTickCount() - start) >= budget) {
            return false;
        }
        ::vTaskDelay(1);
    }
    return false;
}

#if M5HAL_ESPIDF_I2C_SLAVE_LL_BE
// Interrupt bit names differ by chip generation (classic ESP32: FULL/EMPTY,
// every other slave-capable SoC: WM). Both mean "RX FIFO at/above threshold" /
// "TX FIFO below threshold"; probe for the WM name and fall back to the
// classic one, mirroring ESP32_I2C_slave_example's INT_RX_WM/INT_TX_WM.
#if defined(I2C_RXFIFO_WM_INT_ENA_M)
constexpr uint32_t kBeRxWmIntr = I2C_RXFIFO_WM_INT_ENA_M;
constexpr uint32_t kBeTxWmIntr = I2C_TXFIFO_WM_INT_ENA_M;
#else
constexpr uint32_t kBeRxWmIntr = I2C_RXFIFO_FULL_INT_ENA_M;
constexpr uint32_t kBeTxWmIntr = I2C_TXFIFO_EMPTY_INT_ENA_M;
#endif
#endif  // M5HAL_ESPIDF_I2C_SLAVE_LL_BE

}  // namespace impl_espidf_slave
}  // namespace

#if M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_SLAVE_LL_BE
result_t<void> SlaveBus_espidf::stopQueuedProducerAndQuiesce(uint32_t generation)
{
    // The wire and interrupt producers must already be fenced and the bridge
    // must already be Closing.  Stop the only remaining producer before any
    // caller-owned Accessor/Context pointer can be detached.
    portENTER_CRITICAL_SAFE(&_mux);
    _task_stop = true;
    portEXIT_CRITICAL_SAFE(&_mux);
    notifyTaskFromTask();

    const bool stopped = impl_espidf_slave::waitTaskStopped(&_mux, _task_running);
    if (!stopped) {
        auto* task = _task;
        if (task != nullptr) ::vTaskDelete(task);
        portENTER_CRITICAL_SAFE(&_mux);
        _task         = nullptr;
        _task_running = false;
        portEXIT_CRITICAL_SAFE(&_mux);
    } else {
        portENTER_CRITICAL_SAFE(&_mux);
        _task = nullptr;
        portEXIT_CRITICAL_SAFE(&_mux);
    }

    // A stopped producer task is not recreated until teardown/init. Keep
    // this backend fenced even if the bridge itself is reset after detachment.
    __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);

    bool quiesced = stopped ? _queued_bridge.workerForceQuiesce(generation)
                            : _queued_bridge.workerAbandonAfterProducerStopped(generation);
    if (!quiesced && stopped) {
        // The producer is known stopped, so abandonment is also a safe final
        // cleanup if the ordinary force-close invariant was unexpectedly lost.
        quiesced = _queued_bridge.workerAbandonAfterProducerStopped(generation);
    }
    if (!quiesced) {
        // Interrupts/wire were fenced by the caller and the producer task is
        // now gone. Recover independently of a corrupted state/generation so
        // teardown/re-init can never inherit a permanently Closing bridge.
        _queued_bridge.workerRecoverAfterAllProducersStopped();
    }
    return {};
}

#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
void SlaveBus_espidf::testHoldQueuedWorkerSession()
{
    if (_test_held_queued_worker != nullptr) return;
    auto worker = _queued_bridge.workerBeginSession(_queued_generation);
    if (worker.status() == detail::SlaveQueueBridgeResult::Accepted) {
        _test_held_queued_worker = new QueuedBridge::WorkerSession(static_cast<QueuedBridge::WorkerSession&&>(worker));
    }
}

void SlaveBus_espidf::testFailNextQueuedBeginWithHeldWorker()
{
    _test_fail_next_queued_begin = true;
}

void SlaveBus_espidf::testRunWorkerDuringQueuedEnd()
{
#if M5HAL_ESPIDF_I2C_SLAVE_LL_BE
    portENTER_CRITICAL_SAFE(&_mux);
    _queued_be_reload_pending          = true;
    _test_run_worker_during_queued_end = true;
    portEXIT_CRITICAL_SAFE(&_mux);
#endif
}

void SlaveBus_espidf::testReleaseHeldQueuedWorkerSession()
{
    delete _test_held_queued_worker;
    _test_held_queued_worker = nullptr;
}

bool SlaveBus_espidf::testQueuedEndpointsDetached() const
{
    return _queued_accessor == nullptr && _queued_context == nullptr;
}

bool SlaveBus_espidf::testQueuedProducerTaskPresent() const
{
    return _task != nullptr;
}
#endif
#endif

result_t<void> SlaveBus_espidf::resetForInitialization(void)
{
    auto outcome = teardownBackend();
    if (outcome.disposition == bus::CloseDisposition::Success) {
        return {};
    }
    if (outcome.disposition == bus::CloseDisposition::PartialOrUnknown) {
        quarantineLifecycleAfterPartialTeardown();
    }
    return m5::stl::make_unexpected(outcome.error_code);
}

// ===========================================================================
// HW-facing layer
// ===========================================================================
#if M5HAL_ESPIDF_I2C_SLAVE_LL
// ---------------------------------------------------------------------------
// ESP32-S3 path: LL stretch primitive.
//
// THE one non-obvious requirement: i2c_set_pin(..., I2C_MODE_SLAVE) leaves SCL
// input-only, so the slave can never drive SCL low to clock-stretch. We must
// reconnect the SCL output route to the GPIO matrix; without it the HW stretch
// logic fires internally but never reaches the wire, the master clocks through
// an empty FIFO and reads a leaked address byte + underrun.
// ---------------------------------------------------------------------------

result_t<void> SlaveBus_espidf::init(const i2c::SlaveBusConfig& cfg)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (cfg.pin_scl < 0 || cfg.pin_sda < 0 || cfg.address_is_10bit || cfg.address > 0x7Fu) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if !defined(SOC_I2C_SUPPORT_SLAVE) || !SOC_I2C_SUPPORT_SLAVE
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
#endif
    const auto port_r = impl_espidf_slave::resolveSlaveControllerPort(cfg.controller);
    if (!port_r.has_value()) {
        return m5::stl::make_unexpected(port_r.error());
    }

    if (_hw != nullptr || _task != nullptr || _intr != nullptr) {
        auto reset = resetForInitialization();
        if (!reset.has_value()) {
            return reset;
        }
    }
    _config  = cfg;
    _pin_scl = cfg.pin_scl;
    _pin_sda = cfg.pin_sda;

    {
        portENTER_CRITICAL_SAFE(&_mux);
        resetStateLocked();
        _hold_kind             = HoldKind::none;
        _masked_intrs          = 0;
        _baseline_intrs        = 0;
        _resp_len              = 0;
        _resp_pos              = 0;
        _task_stop             = false;
        _task_running          = true;
        _queued_active         = false;
        _queued_lifecycle_used = false;
        _queued_accessor       = nullptr;
        _queued_context        = nullptr;
        _queued_generation     = 0;
        __atomic_store_n(&_queued_broken, 0, __ATOMIC_RELAXED);
        __atomic_store_n(&_queued_dropped_rx, 0, __ATOMIC_RELAXED);
        portEXIT_CRITICAL_SAFE(&_mux);
    }
    if (::xTaskCreate(&SlaveBus_espidf::taskThunk, "m5hal_i2c_slave", 3072, this, configMAX_PRIORITIES - 1, &_task) !=
        pdPASS) {
        portENTER_CRITICAL_SAFE(&_mux);
        _task_running = false;
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    const ::i2c_port_t port = static_cast<::i2c_port_t>(port_r.value());
    ::i2c_dev_t* const hw   = I2C_LL_GET_HW(port);
    _hw                     = hw;
    _port                   = port_r.value();

    // periph_module_enable() is deprecated as "not functional" on newer SoCs
    // (C61/C5/P4 and others) after the RCC refactor. Use the same RCC API as
    // ESP-IDF's I2C driver (esp_driver_i2c/i2c_common.c) to enable the I2C0 bus
    // clock and reset its registers. This works across IDF 5.5/6.0 and all SoCs.
    // SoCs with independent RCC (c5/c6/c61/h2) can call i2c_ll_* directly;
    // RCC_ATOMIC is unnecessary there. On SoCs with shared RCC
    // (s2/s3/c3/p4), i2c_ll_enable_bus_clock requires
    // __DECLARE_RCC_ATOMIC_ENV and therefore must run inside
    // PERIPH_RCC_ATOMIC().
    // Deliberately omit the `::` qualifier here. In shared-RCC i2c_ll.h these
    // operations are function-like macros expanding to
    // `do {(void)__DECLARE_RCC_ATOMIC_ENV; ...} while(0)`. A qualifier would
    // produce `:: do{...}` and fail with "expected id-expression". The
    // unqualified names resolve to the global functions or macros without a
    // conflicting nearby declaration.
#if defined(SOC_RCC_IS_INDEPENDENT) && SOC_RCC_IS_INDEPENDENT
    i2c_ll_enable_bus_clock(port, true);
    i2c_ll_reset_register(port);
#else
    PERIPH_RCC_ATOMIC()
    {
        i2c_ll_enable_bus_clock(port, true);
        i2c_ll_reset_register(port);
    }
#endif

    // Functional (controller) clock, distinct from the APB bus clock above. C6/H2
    // boot code explicitly gates it off (PCR i2c_sclk_en) and the P4 reset value is
    // disabled, so without this the peripheral is dead on cold boot there; S3-class
    // chips only work because their reset value happens to be enabled. Must come
    // AFTER i2c_ll_reset_register (the reset clears it). On P4 the clock control is
    // shared across peripherals and i2c_ll_set_source_clk is an RCC-atomic
    // function-like macro (same `::`-prefix caveat as the RCC block above).
    // A/B diagnosis knob: -DM5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_CONTROLLER_CLOCK=1 skips this block
    // (the pre-fix behavior) to reproduce the cold-boot failure on C6/H2.
#if M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_CONTROLLER_CLOCK
    // skipped: cold-boot A/B baseline
#elif defined(SOC_PERIPH_CLK_CTRL_SHARED) && SOC_PERIPH_CLK_CTRL_SHARED
    PERIPH_RCC_ATOMIC()
    {
        i2c_ll_enable_controller_clock(hw, true);
        i2c_ll_set_source_clk(hw, I2C_CLK_SRC_DEFAULT);
    }
#else
    i2c_ll_enable_controller_clock(hw, true);
    i2c_ll_set_source_clk(hw, I2C_CLK_SRC_DEFAULT);
#endif

    // Route SDA and SCL to the I2C peripheral by hand, via the GPIO matrix,
    // WITHOUT the legacy driver/i2c.h i2c_set_pin(): every M5HAL build also links
    // the new (driver-ng) I2C master, and any legacy I2C driver call aborts at
    // runtime ("CONFLICT! driver_ng is not allowed to be used with this old
    // driver"). Manual muxing also keeps both lines open-drain in/out so the slave
    // can pull SCL low to clock-stretch -- i2c_set_pin(I2C_MODE_SLAVE) would leave
    // SCL input-only and defeat the stretch (the original reason this path existed).
    const ::gpio_num_t pins[2] = {static_cast<::gpio_num_t>(cfg.pin_sda), static_cast<::gpio_num_t>(cfg.pin_scl)};
    const uint32_t out_sig[2]  = {i2c_periph_signal[port].sda_out_sig, i2c_periph_signal[port].scl_out_sig};
    const uint32_t in_sig[2]   = {i2c_periph_signal[port].sda_in_sig, i2c_periph_signal[port].scl_in_sig};
    for (int i = 0; i < 2; ++i) {
        // Route the pad itself to the GPIO matrix (IO_MUX MCU_SEL): none of the
        // calls below touch it, so a pin whose reset function is not GPIO (e.g.
        // plain ESP32 GPIO1/3 = UART) would never reach the matrix without this.
        ::esp_rom_gpio_pad_select_gpio(pins[i]);
        (void)::gpio_set_level(pins[i], 1);
        (void)::gpio_set_direction(pins[i], GPIO_MODE_INPUT_OUTPUT_OD);
        (void)::gpio_set_pull_mode(pins[i], GPIO_PULLUP_ONLY);
        ::esp_rom_gpio_connect_out_signal(pins[i], out_sig[i], false, false);
        ::esp_rom_gpio_connect_in_signal(pins[i], in_sig[i], false);
    }

    // LL register init. Prefer the portable i2c_ll_* helpers so this compiles on
    // every stretch-cause SoC; only the few fields without a helper are poked
    // directly, and those (sda/scl_force_out, fifo_prt_en, fifo_addr_cfg_en) share
    // the same struct field name across all of them, so the pokes stay portable.
    ::i2c_ll_disable_intr_mask(hw, I2C_LL_INTR_MASK);
    ::i2c_ll_clear_intr_mask(hw, I2C_LL_INTR_MASK);
    ::i2c_ll_txfifo_rst(hw);
    ::i2c_ll_rxfifo_rst(hw);

    // Slave output config. Open-drain force-out is a direct (portable) poke; the
    // ACK level and auto-start use LL helpers. Auto-start is REQUIRED: it starts
    // the TX shifter from the preloaded FIFO on the read's address ACK; without it
    // the first byte slot leaks the slave address byte.
    hw->ctr.sda_force_out = 1;
    hw->ctr.scl_force_out = 1;
    ::i2c_ll_master_rx_full_ack_level(hw, 0);
#if M5HAL_DETAIL_ESPIDF_I2C_SLAVE_LL_V54_API_
    ::i2c_ll_slave_enable_auto_start(hw, true);
#else
    ::i2c_ll_slave_tx_auto_start_en(hw, true);
#endif

    if (cfg.legacy_wire_frame_window) {
        ::i2c_ll_set_slave_addr(hw, cfg.address, false);
    } else {
        ::i2c_ll_set_slave_addr(hw, 0x3FFu, true);
    }
    // Make the bus timeout effectively never fire so it cannot abort a long clock
    // stretch (the timeout register name is SoC-specific; the helper is portable).
    ::i2c_ll_set_tout(hw, I2C_LL_MAX_TIMEOUT);

    ::i2c_ll_set_sda_timing(hw, 10, 10);
    ::i2c_ll_master_set_filter(hw, 7);  // light noise filter (not load-bearing)
    // RX/TX water marks at half the HW FIFO (16B): the ISR is notified every ~16B
    // for a steady streaming cadence instead of per-byte. A short write (e.g. a
    // 1-byte register pointer) below the mark is still drained -- at the read
    // address-match stretch (write-then-read) or at STOP -- so it stays available.
    // RX_FULL (FIFO full) is the back-pressure safety above this mark.
    ::i2c_ll_set_rxfifo_full_thr(hw, SOC_I2C_FIFO_LEN / 2);
    ::i2c_ll_set_txfifo_empty_thr(hw, SOC_I2C_FIFO_LEN / 2);

    hw->fifo_conf.fifo_prt_en = 1;
#if M5HAL_DETAIL_ESPIDF_I2C_SLAVE_LL_V54_API_
    ::i2c_ll_enable_fifo_mode(hw, true);
#else
    ::i2c_ll_slave_set_fifo_mode(hw, true);
#endif
    // REQUIRED: do NOT store the matched address byte in the RX FIFO. With its
    // power-on default the slave address lands in RX and is mistaken for data.
    hw->fifo_conf.fifo_addr_cfg_en = 0;

    // Enable SCL stretch with a large protect count covering worst-case ISR/task
    // latency, then clear any stretch left from a prior session.
    ::i2c_ll_slave_enable_scl_stretch(hw, true);
    ::i2c_ll_slave_set_stretch_protect_num(hw, 0x3ff);
    ::i2c_ll_slave_clear_stretch(hw);

    if (::esp_intr_alloc(i2c_periph_signal[port].irq, M5HAL_DETAIL_I2C_SLAVE_ISR_INTR_FLAGS_,
                         &SlaveBus_espidf::isrThunk, this, &_intr) != ESP_OK) {
        auto reset = resetForInitialization();
        if (!reset.has_value()) {
            return reset;
        }
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    _baseline_intrs = I2C_RXFIFO_WM_INT_ENA_M | I2C_TRANS_COMPLETE_INT_ENA_M | I2C_SLAVE_STRETCH_INT_ENA_M;
    if (cfg.legacy_wire_frame_window) {
        ::i2c_ll_enable_intr_mask(hw, _baseline_intrs);
    }
    ::i2c_ll_update(hw);
#if M5HAL_DEBUG_ESPIDF_I2C_SLAVE_GPIO_MARKERS
    {
        const int mpins[3] = {M5HAL_DEBUG_ESPIDF_I2C_SLAVE_TX_FILL_MARKER_PIN,
                              M5HAL_DEBUG_ESPIDF_I2C_SLAVE_RX_DRAIN_MARKER_PIN,
                              M5HAL_DEBUG_ESPIDF_I2C_SLAVE_STRETCH_MARKER_PIN};
        for (int i = 0; i < 3; ++i) {
            ::gpio_set_direction((::gpio_num_t)mpins[i], GPIO_MODE_OUTPUT);
            ::gpio_set_level((::gpio_num_t)mpins[i], 0);
        }
    }
#endif
    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        (void)resetForInitialization();
        return initialized;
    }
    return {};
}

void SlaveBus_espidf::restorePins()
{
    if (_pin_scl >= 0) {
        (void)::gpio_reset_pin(static_cast<::gpio_num_t>(_pin_scl));
    }
    if (_pin_sda >= 0) {
        (void)::gpio_reset_pin(static_cast<::gpio_num_t>(_pin_sda));
    }
    _pin_scl = -1;
    _pin_sda = -1;
}

bus::CloseOutcome SlaveBus_espidf::teardownBackend(void)
{
    if (_queued_active && _queued_accessor != nullptr && _queued_context != nullptr) {
        (void)endOperationBackend(*_queued_context);
    }
    if (_intr != nullptr) {
        if (_hw != nullptr) {
            ::i2c_ll_disable_intr_mask(_hw, I2C_LL_INTR_MASK);
            ::i2c_ll_clear_intr_mask(_hw, I2C_LL_INTR_MASK);
        }
        if (::esp_intr_free(_intr) != ESP_OK) {
            return bus::CloseOutcome::partialOrUnknown(error::error_t::I2C_BUS_ERROR);
        }
        _intr = nullptr;
    }

    if (_task != nullptr) {
        portENTER_CRITICAL_SAFE(&_mux);
        _task_stop = true;
        portEXIT_CRITICAL_SAFE(&_mux);
        notifyTaskFromTask();
        if (!impl_espidf_slave::waitTaskStopped(&_mux, _task_running)) {
            ::vTaskDelete(_task);
            portENTER_CRITICAL_SAFE(&_mux);
            _task_running = false;
            portEXIT_CRITICAL_SAFE(&_mux);
        }
        _task = nullptr;
    }

    _queued_bridge.workerRecoverAfterAllProducersStopped();
    if (!_queued_bridge.workerReset()) {
        return bus::CloseOutcome::partialOrUnknown(error::error_t::INVALID_STATE);
    }

    restorePins();
    if (_hw != nullptr && _port >= 0) {
#if M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_CONTROLLER_CLOCK
        // The matching controller-clock enable was intentionally skipped.
#elif defined(SOC_PERIPH_CLK_CTRL_SHARED) && SOC_PERIPH_CLK_CTRL_SHARED
        PERIPH_RCC_ATOMIC()
        {
            i2c_ll_enable_controller_clock(_hw, false);
        }
#else
        i2c_ll_enable_controller_clock(_hw, false);
#endif

        const ::i2c_port_t port = static_cast<::i2c_port_t>(_port);
#if defined(SOC_RCC_IS_INDEPENDENT) && SOC_RCC_IS_INDEPENDENT
        i2c_ll_enable_bus_clock(port, false);
#else
        PERIPH_RCC_ATOMIC()
        {
            i2c_ll_enable_bus_clock(port, false);
        }
#endif
    }
    _port = -1;
    _hw   = nullptr;

    portENTER_CRITICAL_SAFE(&_mux);
    resetStateLocked();
    _hold_kind             = HoldKind::none;
    _masked_intrs          = 0;
    _baseline_intrs        = 0;
    _task_stop             = false;
    _task_running          = false;
    _queued_active         = false;
    _queued_lifecycle_used = false;
    _queued_accessor       = nullptr;
    _queued_context        = nullptr;
    _queued_generation     = 0;
    __atomic_store_n(&_queued_broken, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&_queued_dropped_rx, 0, __ATOMIC_RELAXED);
    portEXIT_CRITICAL_SAFE(&_mux);
    return bus::CloseOutcome::success();
}

result_t<void> SlaveBus_espidf::beginOperationBackend(bus::OperationContext<i2c::SlaveAccessConfig>& context)
{
    auto& accessor = operationOwner(context);
    if (context.config.tx_mode != slave::QueueMode::Byte || context.config.rx_mode != slave::QueueMode::Byte ||
        _config.tx_underrun != i2c::TxUnderrun::Fill || _config.legacy_wire_frame_window) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
#if M5HAL_DETAIL_I2C_SLAVE_HAS_INTERNAL_PTR_CHECK_
    if (!::esp_ptr_internal(&_queued_bridge)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
#endif
    portENTER_CRITICAL_SAFE(&_mux);
    bool legacy_in_use = false;
    for (const auto& transaction : _transactions) {
        legacy_in_use = legacy_in_use || transaction.in_use;
    }
    if (_hw == nullptr || _queued_active || legacy_in_use || _open != nullptr || _current != nullptr ||
        __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (!_queued_bridge.workerBegin(context.runtime.generation, _config.tx_fill_byte)) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _queued_accessor         = static_cast<SlaveAccessor*>(&accessor);
    _queued_context          = &context;
    _queued_generation       = context.runtime.generation;
    _queued_copied_unique    = 0;
    _queued_popped_confirmed = 0;
    _queued_worker_runs      = 0;
    __atomic_store_n(&_queued_dropped_rx, 0, __ATOMIC_RELAXED);
    _queued_active         = true;
    _queued_lifecycle_used = true;
    _queued_tx_ledger.reset();
    ::i2c_ll_txfifo_rst(_hw);
    ::i2c_ll_rxfifo_rst(_hw);
    ::i2c_ll_slave_clear_stretch(_hw);
    ::i2c_ll_clear_intr_mask(_hw, I2C_LL_INTR_MASK);
    portEXIT_CRITICAL_SAFE(&_mux);

    // Prime caller-prequeued TX bytes synchronously while every peripheral
    // interrupt is still masked. Enabling address-match first would let an
    // external master observe fill bytes merely because the worker task had
    // not run yet.
    processQueuedWorker();

    portENTER_CRITICAL_SAFE(&_mux);
    if (!_queued_active || _queued_accessor != &accessor || _queued_context != &context ||
        __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0) {
        portEXIT_CRITICAL_SAFE(&_mux);
        (void)_queued_bridge.workerRequestClose(context.runtime.generation);
        const auto aborted = stopQueuedProducerAndQuiesce(context.runtime.generation);
        portENTER_CRITICAL_SAFE(&_mux);
        _queued_active     = false;
        _queued_accessor   = nullptr;
        _queued_context    = nullptr;
        _queued_generation = 0;
        portEXIT_CRITICAL_SAFE(&_mux);
        const bool reset = _queued_bridge.workerReset();
        if (!reset) return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        if (!aborted.has_value()) return m5::stl::make_unexpected(aborted.error());
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    ::i2c_ll_set_slave_addr(_hw, _config.address, false);
    ::i2c_ll_enable_intr_mask(_hw, _baseline_intrs);
    ::i2c_ll_update(_hw);
    portEXIT_CRITICAL_SAFE(&_mux);
    return {};
}

result_t<void> SlaveBus_espidf::endOperationBackend(bus::OperationContext<i2c::SlaveAccessConfig>& context)
{
    auto& accessor            = operationOwner(context);
    const uint32_t generation = context.runtime.generation;
    uint32_t dropped_rx       = 0;
    bool hard_stopped         = false;
    portENTER_CRITICAL_SAFE(&_mux);
    if (!_queued_active || _queued_accessor != &accessor || _queued_context != &context || _hw == nullptr) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    // Fence new address matches before waiting for an already accepted wire
    // transaction to reach STOP. A 10-bit address cannot match this bus's
    // configured 7-bit address, while the transaction already in progress no
    // longer consults the address comparator.
    ::i2c_ll_set_slave_addr(_hw, 0x3FFu, true);
    ::i2c_ll_update(_hw);
    portEXIT_CRITICAL_SAFE(&_mux);

    while (::i2c_ll_is_bus_busy(_hw)) {
        if (bus::remainingTimeout(context.runtime, runtime::millis()) == 0) {
            hard_stopped = true;
            break;
        }
        ::vTaskDelay(1);
    }

    portENTER_CRITICAL_SAFE(&_mux);
    ::i2c_ll_disable_intr_mask(_hw, I2C_LL_INTR_MASK);
    if (hard_stopped) {
#if defined(SOC_PERIPH_CLK_CTRL_SHARED) && SOC_PERIPH_CLK_CTRL_SHARED
        PERIPH_RCC_ATOMIC()
        {
            i2c_ll_enable_controller_clock(_hw, false);
        }
#else
        i2c_ll_enable_controller_clock(_hw, false);
#endif
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
    }
    uint32_t free = 0;
    if (!hard_stopped) ::i2c_ll_get_txfifo_len(_hw, &free);
    const uint32_t occupancy = free >= SOC_I2C_FIFO_LEN ? 0 : SOC_I2C_FIFO_LEN - free;
    if (!hard_stopped) {
        auto session = _queued_bridge.isrBegin(generation);
        if (session.status() == detail::SlaveQueueBridgeResult::Accepted) {
            uint32_t rx = 0;
            ::i2c_ll_get_rxfifo_cnt(_hw, &rx);
            if (rx != 0) {
                auto prepared = _queued_bridge.prepareRx(session, rx);
                if (prepared.status == detail::SlaveQueueBridgeResult::Accepted) {
                    if (prepared.first.size != 0) {
                        ::i2c_ll_read_rxfifo(_hw, prepared.first.data, static_cast<uint32_t>(prepared.first.size));
                    }
                    if (prepared.second.size != 0) {
                        ::i2c_ll_read_rxfifo(_hw, prepared.second.data, static_cast<uint32_t>(prepared.second.size));
                    }
                    if (_queued_bridge.commitRx(session) != detail::SlaveQueueBridgeResult::Accepted) {
                        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
                    }
                } else {
                    uint8_t scratch[SOC_I2C_FIFO_LEN];
                    while (rx != 0) {
                        const uint32_t count = std::min<uint32_t>(rx, sizeof(scratch));
                        ::i2c_ll_read_rxfifo(_hw, scratch, count);
                        dropped_rx += count;
                        rx -= count;
                    }
                }
            }
            if (_queued_bridge.boundaryRequired()) {
                (void)_queued_bridge.abortBoundary(session, occupancy);
            }
        } else {
            __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        }
    }
    if (!hard_stopped) {
        ::i2c_ll_txfifo_rst(_hw);
        ::i2c_ll_rxfifo_rst(_hw);
        ::i2c_ll_slave_clear_stretch(_hw);
    }
    (void)_queued_bridge.workerRequestClose(generation);
    portEXIT_CRITICAL_SAFE(&_mux);
    if (dropped_rx != 0) {
        auto& queued_accessor = static_cast<SlaveAccessor&>(accessor);
        queued_accessor.backendRecordDroppedBytes(dropped_rx);
        queued_accessor.backendEvents().publish(slave::SlaveEvent::Overflow, queued_accessor.readable(),
                                                queued_accessor.writable(), generation);
    }
    if (hard_stopped) restorePins();
    notifyTaskFromTask();
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
    processQueuedWorker();
#endif

    bool forced      = false;
    bool abort_error = false;
    for (;;) {
        if (_queued_bridge.workerTryQuiesce(generation)) break;
        const uint32_t remaining = bus::remainingTimeout(context.runtime, runtime::millis());
        if (remaining == 0) {
            forced      = true;
            abort_error = !stopQueuedProducerAndQuiesce(generation).has_value();
            break;
        }
        notifyTaskFromTask();
        ::vTaskDelay(1);
    }

    portENTER_CRITICAL_SAFE(&_mux);
    _queued_active     = false;
    _queued_accessor   = nullptr;
    _queued_context    = nullptr;
    _queued_generation = 0;
    __atomic_store_n(&_queued_broken,
                     (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0 || _queued_bridge.broken()) ? 1u : 0u,
                     __ATOMIC_RELEASE);
    portEXIT_CRITICAL_SAFE(&_mux);
    const bool reset = _queued_bridge.workerReset();
    if (!reset) return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    const bool broken = __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0;
    if (abort_error) return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    if (hard_stopped || forced) return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    if (broken) return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    return {};
}

void SlaveBus_espidf::notifyOperationActivity(bus::IAccessor* owner)
{
    portENTER_CRITICAL_SAFE(&_mux);
    const bool notify = _queued_active && _queued_accessor == owner;
    portEXIT_CRITICAL_SAFE(&_mux);
    if (notify) notifyTaskFromTask();
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
    if (notify) processQueuedWorker();
#endif
}

void SlaveBus_espidf::processQueuedWorker()
{
    SlaveAccessor* accessor                                = nullptr;
    bus::OperationContext<i2c::SlaveAccessConfig>* context = nullptr;
    uint32_t generation                                    = 0;
    portENTER_CRITICAL_SAFE(&_mux);
    if (_queued_active) {
        accessor   = _queued_accessor;
        context    = _queued_context;
        generation = _queued_generation;
    }
    portEXIT_CRITICAL_SAFE(&_mux);
    if (accessor == nullptr || context == nullptr) return;

    auto worker = _queued_bridge.workerBeginSession(generation);
    if (worker.status() != detail::SlaveQueueBridgeResult::Accepted) return;
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
    if (_test_fail_next_queued_begin) {
        _test_fail_next_queued_begin = false;
        _test_held_queued_worker = new QueuedBridge::WorkerSession(static_cast<QueuedBridge::WorkerSession&&>(worker));
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        return;
    }
#endif

    auto publish = [&](slave::SlaveEvent events) {
        accessor->backendEvents().publish(events, accessor->readable(), accessor->writable(), generation);
    };
    const uint32_t dropped_rx = __atomic_exchange_n(&_queued_dropped_rx, 0, __ATOMIC_ACQ_REL);
    if (dropped_rx != 0) {
        accessor->backendRecordDroppedBytes(dropped_rx);
        publish(slave::SlaveEvent::Overflow);
    }
    if (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0 || _queued_bridge.broken()) {
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        publish(slave::SlaveEvent::BusBroken);
    }
    auto confirm = [&](const detail::SlaveTxTotals& totals, bool confirm_bridge_real) -> bool {
        if (totals.real != 0) {
            if (confirm_bridge_real && _queued_bridge.workerConfirmTxReal(worker, static_cast<uint32_t>(totals.real)) !=
                                           detail::SlaveQueueBridgeResult::Accepted) {
                return false;
            }
            auto popped = accessor->backendTxQueue().popBytes(totals.real);
            if (!popped.has_value()) {
                return false;
            }
            _queued_popped_confirmed += totals.real;
            publish(slave::SlaveEvent::TxSpace);
        }
        if (totals.fill != 0) {
            accessor->backendTxQueue().recordUnderrun(static_cast<uint32_t>(totals.fill));
            publish(slave::SlaveEvent::Underrun);
        }
        return true;
    };

    for (size_t iteration = 0; iteration < kQueuedEventCapacity; ++iteration) {
        detail::SlaveQueueRawEvent event;
        uint8_t rx_bytes[kQueuedRxCapacity];
        const auto peeked = _queued_bridge.workerPeekStep(worker, event, rx_bytes, sizeof(rx_bytes));
        if (peeked == detail::SlaveQueueWorkerStep::Empty) break;
        if (peeked != detail::SlaveQueueWorkerStep::Ready) {
            __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
            publish(slave::SlaveEvent::BusBroken);
            break;
        }

        bool commit   = true;
        bool boundary = false;
        detail::SlaveTxBoundaryResult boundary_result{};
        switch (event.kind) {
            case detail::SlaveQueueRawEventKind::RxPayload: {
                if (accessor->backendRxWritable() < event.count) return;
                auto written = accessor->backendWriteRx({rx_bytes, event.count});
                if (!written.has_value() || *written != event.count) return;
                publish(slave::SlaveEvent::RxAvailable);
                break;
            }
            case detail::SlaveQueueRawEventKind::TxLoadedReal:
                commit = _queued_tx_ledger.load(detail::SlaveTxProvenance::Real, event.count).has_value();
                break;
            case detail::SlaveQueueRawEventKind::TxLoadedFill:
                commit = _queued_tx_ledger.load(detail::SlaveTxProvenance::Fill, event.count).has_value();
                break;
            case detail::SlaveQueueRawEventKind::FifoOccupancy: {
                auto observed = _queued_tx_ledger.observeFifoOccupancy(event.value);
                commit        = observed.has_value() && confirm(*observed, true);
                break;
            }
            case detail::SlaveQueueRawEventKind::StopBoundary:
            case detail::SlaveQueueRawEventKind::TxEmptyBoundary:
            case detail::SlaveQueueRawEventKind::AbortBoundary: {
                const auto evidence = event.kind == detail::SlaveQueueRawEventKind::TxEmptyBoundary
                                          ? detail::SlaveTxBoundaryEvidence::ShifterDrained
                                          : detail::SlaveTxBoundaryEvidence::ShifterAmbiguous;
                auto stopped        = _queued_tx_ledger.confirmBoundaryAndDiscard(event.value, evidence);
                if (stopped.has_value()) {
                    boundary_result = *stopped;
                    boundary        = true;
                } else {
                    commit = false;
                }
                break;
            }
        }
        if (!commit || _queued_bridge.workerCommitStep(worker) != detail::SlaveQueueBridgeResult::Accepted) {
            __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
            publish(slave::SlaveEvent::BusBroken);
            break;
        }
        if (boundary) {
            if (_queued_bridge.workerResolveTxBoundary(worker, static_cast<uint32_t>(boundary_result.confirmed.real),
                                                       static_cast<uint32_t>(boundary_result.unclocked.real)) !=
                    detail::SlaveQueueBridgeResult::Accepted ||
                !confirm(boundary_result.confirmed, false)) {
                __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
                publish(slave::SlaveEvent::BusBroken);
                break;
            }
        }
    }

    if (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) == 0 &&
        _queued_bridge.workerState() == detail::SlaveQueueBridgeState::Active) {
        const size_t writable = _queued_bridge.workerTxWritable();
        const size_t borrowed = _queued_copied_unique - _queued_popped_confirmed;
        if (writable != 0) {
            auto bytes = accessor->backendTxQueue().peekBytes(borrowed + writable);
            if (bytes.has_value()) {
                const size_t available  = bytes->first.size + bytes->second.size;
                const size_t copy_count = available > borrowed ? std::min(writable, available - borrowed) : 0;
                uint8_t local[kQueuedTxCapacity];
                for (size_t i = 0; i < copy_count; ++i) {
                    const size_t source = borrowed + i;
                    local[i]            = source < bytes->first.size
                                              ? static_cast<const uint8_t*>(bytes->first.data)[source]
                                              : static_cast<const uint8_t*>(bytes->second.data)[source - bytes->first.size];
                }
                if (copy_count != 0 && _queued_bridge.workerStageTx(worker, local, copy_count) ==
                                           detail::SlaveQueueBridgeResult::Accepted) {
                    _queued_copied_unique += copy_count;
                }
            }
        }
    }
    portENTER_CRITICAL_SAFE(&_mux);
    ++_queued_worker_runs;
    portEXIT_CRITICAL_SAFE(&_mux);
    if (_queued_bridge.broken()) {
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        publish(slave::SlaveEvent::BusBroken);
    }
    // The worker lease ends before the task-side ISR lease below. Together with
    // _mux, bridge sessions make the hardware FIFO a sole serialized consumer.
    resumeQueuedHardwareFromTask();
}

void SlaveBus_espidf::resumeQueuedHardwareFromTask()
{
    portENTER_CRITICAL_SAFE(&_mux);
    if (!_queued_active || _hw == nullptr || __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return;
    }
    auto session = _queued_bridge.isrBegin(_queued_generation);
    if (session.status() != detail::SlaveQueueBridgeResult::Accepted) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return;
    }
    if (_hold_kind == HoldKind::rx_full) {
        uint32_t rx = 0;
        ::i2c_ll_get_rxfifo_cnt(_hw, &rx);
        auto prepared = _queued_bridge.prepareRx(session, rx);
        if (prepared.status == detail::SlaveQueueBridgeResult::Accepted) {
            if (prepared.first.size != 0) {
                ::i2c_ll_read_rxfifo(_hw, prepared.first.data, static_cast<uint8_t>(prepared.first.size));
            }
            if (prepared.second.size != 0) {
                ::i2c_ll_read_rxfifo(_hw, prepared.second.data, static_cast<uint8_t>(prepared.second.size));
            }
            (void)_queued_bridge.commitRx(session);
            const uint32_t masked = clearHoldLocked();
            ::i2c_ll_slave_clear_stretch(_hw);
            enableMaskedInterrupts(masked);
        }
    } else if (_hold_kind == HoldKind::address_read || _hold_kind == HoldKind::tx_empty) {
        uint32_t free = 0;
        ::i2c_ll_get_txfifo_len(_hw, &free);
        uint8_t local[SOC_I2C_FIFO_LEN];
        auto prepared = _queued_bridge.prepareTx(session, local, std::min<size_t>(free, sizeof(local)));
        if (prepared.status == detail::SlaveQueueBridgeResult::Accepted) {
            ::i2c_ll_write_txfifo(_hw, local, static_cast<uint32_t>(prepared.count));
            (void)_queued_bridge.commitTxLoaded(session);
            const uint32_t masked = clearHoldLocked();
            ::i2c_ll_slave_clear_stretch(_hw);
            enableMaskedInterrupts(masked);
        }
    }
    portEXIT_CRITICAL_SAFE(&_mux);
}

// Drain RX-FIFO bytes into the current transaction's rx ring as raw stream bytes
// (no register interpretation). Back-pressure aware -- see the header contract.
// Must run with _mux held.
bool SlaveBus_espidf::drainRxLocked(uint32_t count, bool can_hold)
{
    if (count == 0) {
        return false;
    }
    if (_current == nullptr) {
        _current = allocateTransactionLocked();
        if (_current == nullptr) {
            // No transaction slot to hold the bytes: drain to scratch to keep the HW
            // sane (we cannot back-pressure without a buffer) and surface the loss.
            uint8_t scratch[SOC_I2C_FIFO_LEN];
            while (count) {
                uint8_t c = (count > SOC_I2C_FIFO_LEN) ? SOC_I2C_FIFO_LEN : static_cast<uint8_t>(count);
                ::i2c_ll_read_rxfifo(_hw, scratch, c);
                _rx_overflow_count += c;
                count -= c;
            }
            return false;
        }
    }
    uint8_t buf[SOC_I2C_FIFO_LEN];
    M5HAL_DETAIL_I2C_SLAVE_MARK_HIGH_(M5HAL_DEBUG_ESPIDF_I2C_SLAVE_RX_DRAIN_MARKER_PIN);
    // The STOP-tail reserve (rx[] beyond the back-pressure threshold) must absorb
    // the deepest possible FIFO tail: at STOP the backlog is <= kRxCapacity (the
    // hold machine enforces it) and the FIFO holds <= SOC_I2C_FIFO_LEN.
    static_assert(kRxCapacity + SOC_I2C_FIFO_LEN <= kRxArrayCapacity,
                  "rx[] reserve cannot absorb a full HW FIFO at STOP");
    while (count) {
        // rx[] is a power-of-two ring: the cap bounds the UNREAD backlog
        // (rx_size - rx_read), not the per-transaction total. While read() drains
        // (freeing space), this ISR path can store far more than kRxCapacity bytes
        // across one transaction. Mid-transaction (can_hold) the cap is the
        // back-pressure threshold; at STOP (no later stretch can hold the master)
        // it is the full array, so the FIFO tail spills into the reserve instead
        // of dropping.
        const size_t cap       = can_hold ? kRxCapacity : kRxArrayCapacity;
        const size_t backlog   = _current->rx_size - _current->rx_read;
        const size_t freespace = (backlog < cap) ? (cap - backlog) : 0;
        if (freespace == 0) {
            if (can_hold) {
                // Ring full mid-transaction: leave the remaining `count` bytes in the
                // HW FIFO so the master stays held under the RX_FULL stretch (the
                // caller masks the hold). read() resumes us once it frees ring space.
                // Zero bytes are dropped -- this is the write-direction back-pressure.
                M5HAL_DETAIL_I2C_SLAVE_MARK_LOW_(M5HAL_DEBUG_ESPIDF_I2C_SLAVE_RX_DRAIN_MARKER_PIN);
                return true;
            }
            // STOP with even the reserve full: defensive only -- the hold machine
            // bounds the backlog to kRxCapacity, so backlog + FIFO tail fits the
            // array (static_assert above). Drain and surface via rxOverflow.
            uint8_t c = (count > SOC_I2C_FIFO_LEN) ? SOC_I2C_FIFO_LEN : static_cast<uint8_t>(count);
            ::i2c_ll_read_rxfifo(_hw, buf, c);
            _rx_overflow_count += c;
            count -= c;
            continue;
        }
        uint8_t c = (count > SOC_I2C_FIFO_LEN) ? SOC_I2C_FIFO_LEN : static_cast<uint8_t>(count);
        if (c > freespace) {
            c = static_cast<uint8_t>(freespace);
        }
        ::i2c_ll_read_rxfifo(_hw, buf, c);
        for (uint8_t i = 0; i < c; ++i) {
            _current->rx[(_current->rx_size++) & (kRxArrayCapacity - 1)] = buf[i];
        }
        count -= c;
    }
    M5HAL_DETAIL_I2C_SLAVE_MARK_LOW_(M5HAL_DEBUG_ESPIDF_I2C_SLAVE_RX_DRAIN_MARKER_PIN);
    return false;
}

// Snapshot the accessor's composed reply (the open transaction's unread tx bytes)
// into the backend-owned _resp buffer. From here the whole read -- including the
// TX_EMPTY refills a longer-than-FIFO read needs -- is served out of _resp, whose
// lifetime the backend controls, so a refill never chases a transaction the app
// has already ended. Must run with _mux held.
void SlaveBus_espidf::snapshotResponseLocked()
{
    // Only refill once the current snapshot is fully sent, so a refill never clobbers
    // unsent reply bytes. tx[] is a power-of-two ring; read at the wrapped cursor.
    if (_resp_pos < _resp_len) {
        return;
    }
    _resp_len        = 0;
    _resp_pos        = 0;
    Transaction* txn = _open;
    // Stale-reply guard (same _open == _current rule as the RX_FULL lift in read()):
    // pay out only when the accessor's open transaction IS the transaction on the
    // wire. A write's STOP clears _current in the ISR, but _open lingers until
    // serve() closes it -- on a slow CPU (H2 @ 96MHz) a zero-gap follow-up read's
    // address-match beats that close, and the reply composed DURING the completed
    // write (pre-write register state; expires-at-STOP by design) must not leak into
    // the new read. On a mismatch _resp stays empty, so the caller keeps the stretch
    // held until the accessor opens the wire's transaction and composes afresh (or
    // the stretch budget expires into fill bytes).
    if (txn == nullptr || txn != _current) {
        return;
    }
    while (txn->tx_read < txn->tx_size && _resp_len < kTxCapacity) {
        _resp[_resp_len++] = txn->tx[(txn->tx_read++) & (kTxCapacity - 1)];
    }
}

// Fill the TX FIFO from the response snapshot (_resp_pos.._resp_len), advancing
// _resp_pos. If nothing is left but the master keeps reading, emit a single fill
// byte so the bus does not stall (underrun). Must run with _mux held.
void SlaveBus_espidf::fillTxFromRespLocked()
{
    uint32_t freelen = 0;
    ::i2c_ll_get_txfifo_len(_hw, &freelen);  // free bytes in the TX FIFO
    if (freelen > SOC_I2C_FIFO_LEN) {
        freelen = SOC_I2C_FIFO_LEN;
    }
    if (freelen == 0) {
        return;
    }
    uint8_t buf[SOC_I2C_FIFO_LEN];
    uint32_t n = 0;
    while (n < freelen && _resp_pos < _resp_len) {
        buf[n++] = _resp[_resp_pos++];
    }
    if (n == 0) {
        buf[n++] = _config.tx_fill_byte;  // underrun: keep the read moving
    }
    M5HAL_DETAIL_I2C_SLAVE_MARK_HIGH_(M5HAL_DEBUG_ESPIDF_I2C_SLAVE_TX_FILL_MARKER_PIN);
    ::i2c_ll_write_txfifo(_hw, buf, static_cast<uint8_t>(n));
    M5HAL_DETAIL_I2C_SLAVE_MARK_LOW_(M5HAL_DEBUG_ESPIDF_I2C_SLAVE_TX_FILL_MARKER_PIN);
}

void SlaveBus_espidf::enterTxHoldFromIsrLocked(bool& task_woken, bool address_read)
{
    // RX back-pressure owns the physical stretch until read() has first drained
    // the ring and the residual FIFO tail. Do not reclassify it as a TX hold:
    // write() would then release SCL while RX still has no space. Once read()
    // lifts RX_FULL, the read reaches TX_EMPTY and re-enters here to establish
    // the normal TX hold (or refill immediately from an already queued reply).
    if (_hold_kind == HoldKind::rx_full) {
        return;
    }
    _hold_kind       = address_read ? HoldKind::address_read : HoldKind::tx_empty;
    _request_pending = true;
    _request_tick    = ::xTaskGetTickCountFromISR();
    if ((_masked_intrs & I2C_SLAVE_STRETCH_INT_ENA_M) == 0) {
        _masked_intrs |= I2C_SLAVE_STRETCH_INT_ENA_M;
        ::i2c_ll_clear_intr_mask(_hw, I2C_SLAVE_STRETCH_INT_ENA_M);
        ::i2c_ll_disable_intr_mask(_hw, I2C_SLAVE_STRETCH_INT_ENA_M);
    }
    notifyTaskFromISR(task_woken);
}

void SlaveBus_espidf::enterRxHoldFromIsrLocked()
{
    if (_hold_kind == HoldKind::rx_full) {
        return;  // already holding this transaction's full ring: do not re-mask
    }
    _hold_kind = HoldKind::rx_full;
    // Mask BOTH the stretch and the RX water mark: the HW already holds SCL low
    // (RX_FULL) and the FIFO stays at/over the mark while the ring is full, so an
    // unmasked ISR would spin re-entering here. Do NOT clear the stretch -- that is
    // the back-pressure. read() re-enables these (masked & baseline) and clears the
    // stretch once it drains the ring. No task notify / stretch budget: unlike a TX
    // underrun there is no fill fallback for RX (fabricating received bytes would
    // lose data); the consumer's read() is what lifts the hold.
    const uint32_t bits = (I2C_SLAVE_STRETCH_INT_ENA_M | I2C_RXFIFO_WM_INT_ENA_M) & ~_masked_intrs;
    if (bits != 0) {
        _masked_intrs |= bits;
        ::i2c_ll_clear_intr_mask(_hw, bits);
        ::i2c_ll_disable_intr_mask(_hw, bits);
    }
}

uint32_t SlaveBus_espidf::clearHoldLocked()
{
    const uint32_t masked = _masked_intrs;
    _hold_kind            = HoldKind::none;
    _masked_intrs         = 0;
    return masked;
}

void SlaveBus_espidf::enableMaskedInterrupts(uint32_t mask)
{
    ::i2c_dev_t* hw = _hw;
    if (hw != nullptr) {
        const uint32_t enable = mask & _baseline_intrs;
        if (enable != 0) {
            ::i2c_ll_enable_intr_mask(hw, enable);
        }
    }
}

void SlaveBus_espidf::isrThunk(void* arg)
{
    static_cast<SlaveBus_espidf*>(arg)->handleIsr();
}

void SlaveBus_espidf::handleIsr()
{
    ::i2c_dev_t* hw = _hw;
    if (hw == nullptr) {
        return;
    }
    bool woken = false;

    uint32_t ints = 0;
    ::i2c_ll_get_intr_mask(hw, &ints);
    ::i2c_ll_clear_intr_mask(hw, ints);

    uint32_t rx = 0;
    ::i2c_ll_get_rxfifo_cnt(hw, &rx);
    const bool is_read = (::i2c_ll_slave_get_read_write_status(hw) == I2C_SLAVE_READ_BY_MASTER);

    portENTER_CRITICAL_ISR(&_mux);

    if (_queued_active) {
        (void)handleQueuedIsrLocked(hw, ints, rx, is_read, woken);
        portEXIT_CRITICAL_ISR(&_mux);
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
        // Deterministic worker-context seam: the host fake has no scheduler.
        processQueuedWorker();
#endif
        if (woken) portYIELD_FROM_ISR();
        return;
    }

    // RX water-mark: drain the written bytes (raw) into the current transaction.
    // If the ring fills (consumer behind), assert the back-pressure hold so the
    // master is stretched until read() frees space -- no byte is dropped.
    if ((ints & I2C_RXFIFO_WM_INT_ENA_M) && rx) {
        if (drainRxLocked(rx, true)) {
            enterRxHoldFromIsrLocked();
        }
        rx = 0;
    }

    // TX water-mark: during a read the FIFO has drained to its threshold (FIFO/2).
    // Proactively top it up from the reply snapshot BEFORE it empties, so the master
    // clocks the reply out continuously instead of the slave stretching to refill at
    // every FIFO drain. The TX_EMPTY stretch below is the underrun FALLBACK (reply not
    // yet supplied), NOT the steady-state refill -- this proactive top-up is what
    // removes the 32-byte read-side pause (each FIFO-empty -> stretch -> refill). If
    // no reply bytes are queued yet, stop the water mark re-firing (it asserts while
    // the FIFO stays below the threshold); the FIFO then empties into the TX_EMPTY
    // stretch, which holds until the app streams more. Enabled on read-stretch release
    // and disabled at STOP / when the reply is exhausted (so it never fires on writes).
#if M5HAL_DETAIL_I2C_SLAVE_TX_WATERMARK_
    if (ints & I2C_TXFIFO_WM_INT_ENA_M) {
        if (is_read) {
            snapshotResponseLocked();
            if (_resp_pos < _resp_len) {
                fillTxFromRespLocked();
            } else {
                ::i2c_ll_disable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
            }
        } else {
            // Storm brake: the raw TXFIFO_WM source is level-type (valid while the
            // FIFO count stays below the threshold), and is_read (sr.slave_rw) only
            // tracks the most recent ADDRESS MATCH. If a read's STOP teardown was
            // skipped and a write follows, the WM keeps re-asserting with neither a
            // fill nor a disable on this path -- disable it here so a stale-direction
            // pass can never spin the ISR.
            ::i2c_ll_disable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
        }
    }
#endif

    // STOP: mark the transaction complete and discard the TX bytes the master did
    // not read (the window's Tx auto-vanish). Handled BEFORE the stretch causes:
    // when a delayed ISR pass sees both the previous transaction's STOP and the
    // next transaction's ADDRESS_MATCH stretch in one snapshot, the bus-order is
    // STOP first -- processing it after the stretch branch would tear down the
    // hold/_current that ADDRESS_MATCH just set up for the NEW transaction (and a
    // pure read following a write would silently re-use the completed transaction).
    if (ints & I2C_TRANS_COMPLETE_INT_ENA_M) {
        if (rx) {
            // Transaction ending: there is no later read() to lift a hold, so drain
            // all and drop any tail that no longer fits (back-pressure normally keeps
            // the ring from being full here; a drop means the consumer never caught up).
            drainRxLocked(rx, false);
            rx = 0;
        }
        // TX teardown is direction-INDEPENDENT: is_read is stale here both in the
        // race where the next transaction's address phase already ran before this
        // pass, and deterministically at the final STOP of a read->RESTART->write
        // composite. Gating on it leaves the level-type TXFIFO_WM armed with nothing
        // left to disable it (see the storm brake above). A write-STOP's TX FIFO is
        // idle, so the unconditional reset is harmless in that direction.
        ::i2c_ll_disable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
        ::i2c_ll_txfifo_rst(hw);
        if (_current != nullptr) {
            _current->complete = true;
        }
        // Close the current transaction so the NEXT bus transaction allocates a
        // fresh slot. Without this, a read that follows a separate write (SPLIT /
        // a register write then a plain read) re-uses the completed write
        // transaction instead of getting its own, so the accessor has nothing new
        // to open and the read intermittently goes unserved. Mirrors the software
        // backend's stopCondition().
        const uint32_t masked = clearHoldLocked();
        if (masked != 0) {
            ::i2c_ll_slave_clear_stretch(hw);
            enableMaskedInterrupts(masked);
        }
        _current         = nullptr;
        _request_pending = false;
        _resp_len        = 0;  // this read's reply is spent; next read snapshots afresh
        _resp_pos        = 0;
    }

    // Stretch: the HW is holding SCL low waiting for us.
    if (ints & I2C_SLAVE_STRETCH_INT_ENA_M) {
        M5HAL_DETAIL_I2C_SLAVE_MARK_HIGH_(M5HAL_DEBUG_ESPIDF_I2C_SLAVE_STRETCH_MARKER_PIN);
        ::i2c_slave_stretch_cause_t cause;
        ::i2c_ll_slave_get_stretch_cause(hw, &cause);
        if (cause == I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH) {
            if (_hold_kind != HoldKind::none) {
                const uint32_t masked = clearHoldLocked();
                _request_pending      = false;
                _resp_len             = 0;
                _resp_pos             = 0;
                enableMaskedInterrupts(masked);
            }
            if (rx) {
                // Write phase: any pending RX is raw write data. A fresh transaction's
                // ring is empty here so the hold rarely triggers, but honor it for
                // safety (and skip the write-branch clear below if it does).
                if (drainRxLocked(rx, true)) {
                    enterRxHoldFromIsrLocked();
                }
                rx = 0;
            }
            if (is_read) {
                // Read phase: keep the stretch HELD and wake the responder task
                // to fill the TX FIFO from the open transaction. Do NOT clear.
                if (_current == nullptr) {
                    // Pure read with no write phase in this transaction (e.g. a
                    // SPLIT read that follows a separate register-pointer write):
                    // allocate a transaction so the accessor has something to open
                    // and compose the reply into. A write-then-read already
                    // allocated _current while draining the register byte, so this
                    // only fires for read-only transactions.
                    _current = allocateTransactionLocked();
                }
                enterTxHoldFromIsrLocked(woken, true);
            } else if (_hold_kind != HoldKind::rx_full) {
                ::i2c_ll_slave_clear_stretch(hw);
            }
        } else if (cause == I2C_SLAVE_STRETCH_CAUSE_TX_EMPTY) {
            // Long read continuation (master reads past the first FIFO load). Refill
            // _resp from the tx ring (the app may have streamed more reply bytes) and,
            // if anything is available, top up the FIFO and release the stretch right
            // here in the ISR -- routing a continuation through the responder task adds
            // enough latency that a fast master underruns the FIFO at the 32-byte
            // boundary. If the tx ring AND _resp are both empty, the reply is not done
            // streaming yet: HOLD the stretch and wake the task, which fills + clears
            // once the app queues more (or emits a fill byte past the stretch budget).
            // This is what lets a reply far larger than kTxCapacity stream out (the
            // read-direction counterpart of the RX ring removing the write cap).
            snapshotResponseLocked();
            if (_resp_pos < _resp_len) {
                fillTxFromRespLocked();
                ::i2c_ll_slave_clear_stretch(hw);
#if M5HAL_DETAIL_I2C_SLAVE_TX_WATERMARK_
                // Re-arm the proactive water-mark top-up now that the reply is flowing.
                ::i2c_ll_enable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
#endif
            } else {
                enterTxHoldFromIsrLocked(woken, false);
            }
        } else if (cause == I2C_SLAVE_STRETCH_CAUSE_RX_FULL) {
            // Long write: drain what fits into the ring. If the ring is full (the
            // consumer has not drained it yet) KEEP the stretch held -- this is the
            // write-direction back-pressure; read() lifts it once it frees space, and
            // nothing is dropped. Otherwise (drained OK, or an already-asserted
            // rx_full hold from the WM path this pass) leave the held stretch alone.
            if (rx && drainRxLocked(rx, true)) {
                enterRxHoldFromIsrLocked();
            } else if (_hold_kind != HoldKind::rx_full) {
                ::i2c_ll_slave_clear_stretch(hw);
            }
            rx = 0;
        } else {
            ::i2c_ll_slave_clear_stretch(hw);
        }
        M5HAL_DETAIL_I2C_SLAVE_MARK_LOW_(M5HAL_DEBUG_ESPIDF_I2C_SLAVE_STRETCH_MARKER_PIN);
    }

    // Wake the serve() consumer on this activity (RX drained, hold entered, reply
    // room, or STOP) so it drains/fills without waiting out a poll delay -- the key
    // to keeping the RX ring from being full at the master's STOP at high speed.
    notifyConsumerFromISR(woken);

    portEXIT_CRITICAL_ISR(&_mux);

    if (woken) {
        portYIELD_FROM_ISR();
    }
}

bool SlaveBus_espidf::handleQueuedIsrLocked(::i2c_dev_t* hw, uint32_t ints, uint32_t rx, bool is_read, bool& task_woken)
{
    auto session = _queued_bridge.isrBegin(_queued_generation);
    if (session.status() != detail::SlaveQueueBridgeResult::Accepted) return false;
    bool activity = false;

    auto drain_rx = [&](uint32_t count) __attribute__((always_inline))
    {
        if (count == 0) return true;
        auto prepared = _queued_bridge.prepareRx(session, count);
        if (prepared.status == detail::SlaveQueueBridgeResult::NoSpace) {
            enterRxHoldFromIsrLocked();
            return false;
        }
        if (prepared.status != detail::SlaveQueueBridgeResult::Accepted) return false;
        if (prepared.first.size != 0) {
            ::i2c_ll_read_rxfifo(hw, prepared.first.data, static_cast<uint8_t>(prepared.first.size));
        }
        if (prepared.second.size != 0) {
            ::i2c_ll_read_rxfifo(hw, prepared.second.data, static_cast<uint8_t>(prepared.second.size));
        }
        if (_queued_bridge.commitRx(session) != detail::SlaveQueueBridgeResult::Accepted) return false;
        activity = true;
        return true;
    };
    auto load_tx = [&](bool address_read) __attribute__((always_inline))
    {
        uint32_t free = 0;
        ::i2c_ll_get_txfifo_len(hw, &free);
        if (free == 0) return true;
        uint8_t local[SOC_I2C_FIFO_LEN];
        auto prepared = _queued_bridge.prepareTx(session, local, std::min<size_t>(free, sizeof(local)));
        if (prepared.status != detail::SlaveQueueBridgeResult::Accepted) {
            enterTxHoldFromIsrLocked(task_woken, address_read);
            return false;
        }
        ::i2c_ll_write_txfifo(hw, local, static_cast<uint32_t>(prepared.count));
        if (_queued_bridge.commitTxLoaded(session) != detail::SlaveQueueBridgeResult::Accepted) return false;
        activity = true;
        return true;
    };

    if ((ints & I2C_RXFIFO_WM_INT_ENA_M) && rx) {
        if (drain_rx(rx)) rx = 0;
    }

#if M5HAL_DETAIL_I2C_SLAVE_TX_WATERMARK_
    if (ints & I2C_TXFIFO_WM_INT_ENA_M) {
        if (is_read) {
            uint32_t free = 0;
            ::i2c_ll_get_txfifo_len(hw, &free);
            const uint32_t occupancy = free >= SOC_I2C_FIFO_LEN ? 0 : SOC_I2C_FIFO_LEN - free;
            if (_queued_bridge.observeFifo(session, occupancy) == detail::SlaveQueueBridgeResult::Accepted) {
                activity = true;
            }
            if (!load_tx(false)) ::i2c_ll_disable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
        } else {
            ::i2c_ll_disable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
        }
    }
#endif

    if (ints & I2C_TRANS_COMPLETE_INT_ENA_M) {
        if (rx) {
            if (!drain_rx(rx)) {
                uint8_t scratch[SOC_I2C_FIFO_LEN];
                uint32_t remaining = rx;
                while (remaining != 0) {
                    const uint32_t count = std::min<uint32_t>(remaining, sizeof(scratch));
                    ::i2c_ll_read_rxfifo(hw, scratch, count);
                    remaining -= count;
                }
                __atomic_fetch_add(&_queued_dropped_rx, rx, __ATOMIC_RELAXED);
                __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
                ::i2c_ll_set_slave_addr(hw, 0x3FFu, true);
                ::i2c_ll_disable_intr_mask(hw, I2C_LL_INTR_MASK);
                ::i2c_ll_update(hw);
            }
            rx = 0;
        }
        uint32_t free = 0;
        ::i2c_ll_get_txfifo_len(hw, &free);
        const uint32_t occupancy = free >= SOC_I2C_FIFO_LEN ? 0 : SOC_I2C_FIFO_LEN - free;
        if (_queued_bridge.stopBoundary(session, occupancy) != detail::SlaveQueueBridgeResult::Accepted) {
            __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        } else {
            activity = true;
        }
        ::i2c_ll_disable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
        ::i2c_ll_txfifo_rst(hw);
        const uint32_t masked = clearHoldLocked();
        if (masked != 0 && __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) == 0) {
            ::i2c_ll_slave_clear_stretch(hw);
            enableMaskedInterrupts(masked);
        }
    }

    if (ints & I2C_SLAVE_STRETCH_INT_ENA_M) {
        ::i2c_slave_stretch_cause_t cause;
        ::i2c_ll_slave_get_stretch_cause(hw, &cause);
        if (cause == I2C_SLAVE_STRETCH_CAUSE_ADDRESS_MATCH) {
            if (rx && drain_rx(rx)) rx = 0;
            if (is_read) {
                if (load_tx(true)) {
                    ::i2c_ll_slave_clear_stretch(hw);
#if M5HAL_DETAIL_I2C_SLAVE_TX_WATERMARK_
                    ::i2c_ll_enable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
#endif
                }
            } else if (_hold_kind != HoldKind::rx_full) {
                ::i2c_ll_slave_clear_stretch(hw);
            }
        } else if (cause == I2C_SLAVE_STRETCH_CAUSE_TX_EMPTY) {
            if (_queued_bridge.txEmptyBoundary(session, 0) == detail::SlaveQueueBridgeResult::Accepted) {
                activity = true;
            }
            enterTxHoldFromIsrLocked(task_woken, false);
        } else if (cause == I2C_SLAVE_STRETCH_CAUSE_RX_FULL) {
            if (rx && drain_rx(rx)) {
                rx = 0;
                ::i2c_ll_slave_clear_stretch(hw);
            }
        } else {
            ::i2c_ll_slave_clear_stretch(hw);
        }
    }
    if (activity) notifyTaskFromISR(task_woken);
    return activity;
}

#elif M5HAL_ESPIDF_I2C_SLAVE_LL_BE
// ---------------------------------------------------------------------------
// Classic ESP32 path: LL best-effort (no clock stretch). Same GPIO-matrix /
// RCC / FIFO setup as the LL flavor (see its comments above for the "why"
// behind each step); the differences are called out below.
// ---------------------------------------------------------------------------

result_t<void> SlaveBus_espidf::init(const i2c::SlaveBusConfig& cfg)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (cfg.pin_scl < 0 || cfg.pin_sda < 0 || cfg.address_is_10bit || cfg.address > 0x7Fu) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if !defined(SOC_I2C_SUPPORT_SLAVE) || !SOC_I2C_SUPPORT_SLAVE
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
#endif
    // BE has no clock stretch, so nothing can hold the bus while a `stretch`
    // policy accessor composes its reply -- reject it up front (same guard the
    // v2 driver flavor uses for the same reason).
    if (cfg.tx_underrun == i2c::TxUnderrun::Stretch) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const auto port_r = impl_espidf_slave::resolveSlaveControllerPort(cfg.controller);
    if (!port_r.has_value()) {
        return m5::stl::make_unexpected(port_r.error());
    }

    if (_hw != nullptr || _task != nullptr || _intr != nullptr) {
        auto reset = resetForInitialization();
        if (!reset.has_value()) {
            return reset;
        }
    }
    _config  = cfg;
    _pin_scl = cfg.pin_scl;
    _pin_sda = cfg.pin_sda;

    {
        portENTER_CRITICAL_SAFE(&_mux);
        resetStateLocked();
        _resp_len                 = 0;
        _resp_pos                 = 0;
        _task_stop                = false;
        _task_running             = true;
        _queued_active            = false;
        _queued_lifecycle_used    = false;
        _queued_be_reload_pending = false;
        _queued_closing           = false;
        _queued_accessor          = nullptr;
        _queued_context           = nullptr;
        _queued_generation        = 0;
        __atomic_store_n(&_queued_broken, 0, __ATOMIC_RELAXED);
        __atomic_store_n(&_queued_dropped_rx, 0, __ATOMIC_RELAXED);
        portEXIT_CRITICAL_SAFE(&_mux);
    }
    if (::xTaskCreate(&SlaveBus_espidf::taskThunk, "m5hal_i2c_slave", 3072, this, configMAX_PRIORITIES - 1, &_task) !=
        pdPASS) {
        portENTER_CRITICAL_SAFE(&_mux);
        _task_running = false;
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    const ::i2c_port_t port = static_cast<::i2c_port_t>(port_r.value());
    ::i2c_dev_t* const hw   = I2C_LL_GET_HW(port);
    _hw                     = hw;
    _port                   = port_r.value();

#if defined(SOC_RCC_IS_INDEPENDENT) && SOC_RCC_IS_INDEPENDENT
    i2c_ll_enable_bus_clock(port, true);
    i2c_ll_reset_register(port);
#else
    PERIPH_RCC_ATOMIC()
    {
        i2c_ll_enable_bus_clock(port, true);
        i2c_ll_reset_register(port);
    }
#endif

#if M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_CONTROLLER_CLOCK
    // skipped: cold-boot A/B baseline
#elif defined(SOC_PERIPH_CLK_CTRL_SHARED) && SOC_PERIPH_CLK_CTRL_SHARED
    PERIPH_RCC_ATOMIC()
    {
        i2c_ll_enable_controller_clock(hw, true);
        i2c_ll_set_source_clk(hw, I2C_CLK_SRC_DEFAULT);
    }
#else
    i2c_ll_enable_controller_clock(hw, true);
    i2c_ll_set_source_clk(hw, I2C_CLK_SRC_DEFAULT);
#endif

    const ::gpio_num_t pins[2] = {static_cast<::gpio_num_t>(cfg.pin_sda), static_cast<::gpio_num_t>(cfg.pin_scl)};
    const uint32_t out_sig[2]  = {i2c_periph_signal[port].sda_out_sig, i2c_periph_signal[port].scl_out_sig};
    const uint32_t in_sig[2]   = {i2c_periph_signal[port].sda_in_sig, i2c_periph_signal[port].scl_in_sig};
    for (int i = 0; i < 2; ++i) {
        ::esp_rom_gpio_pad_select_gpio(pins[i]);
        (void)::gpio_set_level(pins[i], 1);
        (void)::gpio_set_direction(pins[i], GPIO_MODE_INPUT_OUTPUT_OD);
        (void)::gpio_set_pull_mode(pins[i], GPIO_PULLUP_ONLY);
        ::esp_rom_gpio_connect_out_signal(pins[i], out_sig[i], false, false);
        ::esp_rom_gpio_connect_in_signal(pins[i], in_sig[i], false);
    }

    ::i2c_ll_disable_intr_mask(hw, I2C_LL_INTR_MASK);
    ::i2c_ll_clear_intr_mask(hw, I2C_LL_INTR_MASK);
    ::i2c_ll_txfifo_rst(hw);
    ::i2c_ll_rxfifo_rst(hw);

    hw->ctr.sda_force_out = 1;
    hw->ctr.scl_force_out = 1;
    ::i2c_ll_master_rx_full_ack_level(hw, 0);
#if M5HAL_DETAIL_ESPIDF_I2C_SLAVE_LL_V54_API_
    ::i2c_ll_slave_enable_auto_start(hw, true);
#else
    ::i2c_ll_slave_tx_auto_start_en(hw, true);
#endif

    if (cfg.legacy_wire_frame_window) {
        ::i2c_ll_set_slave_addr(hw, cfg.address, false);
    } else {
        ::i2c_ll_set_slave_addr(hw, 0x3FFu, true);
    }
    ::i2c_ll_set_tout(hw, I2C_LL_MAX_TIMEOUT);

    ::i2c_ll_set_sda_timing(hw, 10, 10);
    ::i2c_ll_master_set_filter(hw, 7);  // light noise filter (not load-bearing)
    // RX water mark = 1 byte (LL uses FIFO/2): without a stretch to hold the
    // master, the ISR must capture a write's leading byte as fast as possible
    // (a repeated-START read can follow immediately). TX stays at FIFO/2, same
    // cadence as the LL flavor.
    ::i2c_ll_set_rxfifo_full_thr(hw, 1);
    ::i2c_ll_set_txfifo_empty_thr(hw, SOC_I2C_FIFO_LEN / 2);

#if !defined(CONFIG_IDF_TARGET_ESP32)
    // Classic ESP32 has no fifo_prt_en field; every other slave-capable SoC
    // resets to 1 but this keeps FIFO pointer control explicit.
    hw->fifo_conf.fifo_prt_en = 1;
#endif
#if M5HAL_DETAIL_ESPIDF_I2C_SLAVE_LL_V54_API_
    ::i2c_ll_enable_fifo_mode(hw, true);
#else
    ::i2c_ll_slave_set_fifo_mode(hw, true);
#endif
    hw->fifo_conf.fifo_addr_cfg_en = 0;

    // No clock-stretch enable block here: i2c_ll_slave_enable_scl_stretch is a
    // no-op on SoCs without SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE (that is what
    // selects this flavor in the first place), so skip it rather than call a
    // primitive that does nothing.

    if (::esp_intr_alloc(i2c_periph_signal[port].irq, M5HAL_DETAIL_I2C_SLAVE_ISR_INTR_FLAGS_,
                         &SlaveBus_espidf::isrThunk, this, &_intr) != ESP_OK) {
        auto reset = resetForInitialization();
        if (!reset.has_value()) {
            return reset;
        }
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    if (cfg.legacy_wire_frame_window) {
        ::i2c_ll_enable_intr_mask(
            hw, impl_espidf_slave::kBeRxWmIntr | I2C_TRANS_COMPLETE_INT_ENA_M | impl_espidf_slave::kBeTxWmIntr);
    }
    ::i2c_ll_update(hw);

    // Prime the TX FIFO (allocate a transaction and top it up with fill bytes,
    // same as the STOP handler in handleIsr()) so a read arriving before any
    // write()/STOP still gets fill bytes instead of stale/garbage data.
    if (cfg.legacy_wire_frame_window) {
        portENTER_CRITICAL_SAFE(&_mux);
        _current = allocateTransactionLocked();
        snapshotResponseLocked();
        fillTxFromRespLocked();
        portEXIT_CRITICAL_SAFE(&_mux);
    }

    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        (void)resetForInitialization();
        return initialized;
    }
    return {};
}

void SlaveBus_espidf::restorePins()
{
    if (_pin_scl >= 0) {
        (void)::gpio_reset_pin(static_cast<::gpio_num_t>(_pin_scl));
    }
    if (_pin_sda >= 0) {
        (void)::gpio_reset_pin(static_cast<::gpio_num_t>(_pin_sda));
    }
    _pin_scl = -1;
    _pin_sda = -1;
}

bus::CloseOutcome SlaveBus_espidf::teardownBackend(void)
{
    if (_queued_active && _queued_accessor != nullptr && _queued_context != nullptr) {
        (void)endOperationBackend(*_queued_context);
    }
    if (_intr != nullptr) {
        if (_hw != nullptr) {
            ::i2c_ll_disable_intr_mask(_hw, I2C_LL_INTR_MASK);
            ::i2c_ll_clear_intr_mask(_hw, I2C_LL_INTR_MASK);
        }
        if (::esp_intr_free(_intr) != ESP_OK) {
            return bus::CloseOutcome::partialOrUnknown(error::error_t::I2C_BUS_ERROR);
        }
        _intr = nullptr;
    }

    if (_task != nullptr) {
        portENTER_CRITICAL_SAFE(&_mux);
        _task_stop = true;
        portEXIT_CRITICAL_SAFE(&_mux);
        notifyTaskFromTask();
        if (!impl_espidf_slave::waitTaskStopped(&_mux, _task_running)) {
            ::vTaskDelete(_task);
            portENTER_CRITICAL_SAFE(&_mux);
            _task_running = false;
            portEXIT_CRITICAL_SAFE(&_mux);
        }
        _task = nullptr;
    }

    _queued_bridge.workerRecoverAfterAllProducersStopped();
    if (!_queued_bridge.workerReset()) {
        return bus::CloseOutcome::partialOrUnknown(error::error_t::INVALID_STATE);
    }

    restorePins();
    if (_hw != nullptr && _port >= 0) {
#if M5HAL_DEBUG_ESPIDF_I2C_SLAVE_NO_CONTROLLER_CLOCK
        // The matching controller-clock enable was intentionally skipped.
#elif defined(SOC_PERIPH_CLK_CTRL_SHARED) && SOC_PERIPH_CLK_CTRL_SHARED
        PERIPH_RCC_ATOMIC()
        {
            i2c_ll_enable_controller_clock(_hw, false);
        }
#else
        i2c_ll_enable_controller_clock(_hw, false);
#endif

        const ::i2c_port_t port = static_cast<::i2c_port_t>(_port);
#if defined(SOC_RCC_IS_INDEPENDENT) && SOC_RCC_IS_INDEPENDENT
        i2c_ll_enable_bus_clock(port, false);
#else
        PERIPH_RCC_ATOMIC()
        {
            i2c_ll_enable_bus_clock(port, false);
        }
#endif
    }
    _port = -1;
    _hw   = nullptr;

    portENTER_CRITICAL_SAFE(&_mux);
    resetStateLocked();
    _resp_len                 = 0;
    _resp_pos                 = 0;
    _task_stop                = false;
    _task_running             = false;
    _queued_active            = false;
    _queued_lifecycle_used    = false;
    _queued_be_reload_pending = false;
    _queued_closing           = false;
    _queued_accessor          = nullptr;
    _queued_context           = nullptr;
    _queued_generation        = 0;
    __atomic_store_n(&_queued_broken, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&_queued_dropped_rx, 0, __ATOMIC_RELAXED);
    // Drop the regmap fast-path binding along with the rest of the HW state --
    // a re-init starts from a clean slate; the accessor that owns the binding
    // struct still holds it and re-binds on its next setOnRead/setOnWrite (or
    // simply re-constructs). We only drop OUR pointer to it, never touch the
    // struct itself (we do not own it).
    _isr_binding = nullptr;
    portEXIT_CRITICAL_SAFE(&_mux);
    return bus::CloseOutcome::success();
}

result_t<void> SlaveBus_espidf::beginOperationBackend(bus::OperationContext<i2c::SlaveAccessConfig>& context)
{
    auto& accessor = operationOwner(context);
    if (context.config.tx_mode != slave::QueueMode::Byte || context.config.rx_mode != slave::QueueMode::Byte ||
        _config.tx_underrun != i2c::TxUnderrun::Fill || _config.legacy_wire_frame_window) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
#if M5HAL_DETAIL_I2C_SLAVE_HAS_INTERNAL_PTR_CHECK_
    if (!::esp_ptr_internal(&_queued_bridge)) return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
#endif
    portENTER_CRITICAL_SAFE(&_mux);
    bool legacy_in_use = false;
    for (const auto& transaction : _transactions) legacy_in_use = legacy_in_use || transaction.in_use;
    if (_hw == nullptr || _queued_active || legacy_in_use || _open != nullptr || _current != nullptr ||
        _isr_binding != nullptr || __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0 ||
        !_queued_bridge.workerBegin(context.runtime.generation, _config.tx_fill_byte)) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    _queued_accessor          = static_cast<SlaveAccessor*>(&accessor);
    _queued_context           = &context;
    _queued_generation        = context.runtime.generation;
    _queued_copied_unique     = 0;
    _queued_popped_confirmed  = 0;
    _queued_active            = true;
    _queued_lifecycle_used    = true;
    _queued_be_reload_pending = true;
    _queued_closing           = false;
    _queued_tx_ledger.reset();
    __atomic_store_n(&_queued_dropped_rx, 0, __ATOMIC_RELAXED);
    ::i2c_ll_disable_intr_mask(_hw, I2C_LL_INTR_MASK);
    ::i2c_ll_clear_intr_mask(_hw, I2C_LL_INTR_MASK);
    ::i2c_ll_txfifo_rst(_hw);
    ::i2c_ll_rxfifo_rst(_hw);
    portEXIT_CRITICAL_SAFE(&_mux);
    processQueuedWorker();

    portENTER_CRITICAL_SAFE(&_mux);
    const bool ready = _queued_active && _queued_accessor == &accessor && _queued_context == &context &&
                       !_queued_be_reload_pending && __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) == 0;
    portEXIT_CRITICAL_SAFE(&_mux);
    if (ready) return {};
    (void)_queued_bridge.workerRequestClose(context.runtime.generation);
    const auto aborted = stopQueuedProducerAndQuiesce(context.runtime.generation);
    portENTER_CRITICAL_SAFE(&_mux);
    _queued_active            = false;
    _queued_be_reload_pending = false;
    _queued_closing           = false;
    _queued_accessor          = nullptr;
    _queued_context           = nullptr;
    _queued_generation        = 0;
    portEXIT_CRITICAL_SAFE(&_mux);
    const bool reset = _queued_bridge.workerReset();
    if (!reset) return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    if (!aborted.has_value()) return m5::stl::make_unexpected(aborted.error());
    return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
}

result_t<void> SlaveBus_espidf::endOperationBackend(bus::OperationContext<i2c::SlaveAccessConfig>& context)
{
    auto& accessor            = operationOwner(context);
    const uint32_t generation = context.runtime.generation;
    uint32_t dropped_rx       = 0;
    bool hard_stopped         = false;
    portENTER_CRITICAL_SAFE(&_mux);
    if (!_queued_active || _queued_accessor != &accessor || _queued_context != &context || _hw == nullptr) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    ::i2c_ll_set_slave_addr(_hw, 0x3FFu, true);
    ::i2c_ll_update(_hw);
    _queued_closing = true;
    portEXIT_CRITICAL_SAFE(&_mux);
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
    if (_test_run_worker_during_queued_end) {
        _test_run_worker_during_queued_end = false;
        processQueuedWorker();
    }
#endif
    while (::i2c_ll_is_bus_busy(_hw)) {
        if (bus::remainingTimeout(context.runtime, runtime::millis()) == 0) {
            hard_stopped = true;
            break;
        }
        ::vTaskDelay(1);
    }
    portENTER_CRITICAL_SAFE(&_mux);
    ::i2c_ll_disable_intr_mask(_hw, I2C_LL_INTR_MASK);
    if (hard_stopped) {
#if defined(SOC_PERIPH_CLK_CTRL_SHARED) && SOC_PERIPH_CLK_CTRL_SHARED
        PERIPH_RCC_ATOMIC()
        {
            i2c_ll_enable_controller_clock(_hw, false);
        }
#else
        i2c_ll_enable_controller_clock(_hw, false);
#endif
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
    }
    uint32_t free = 0;
    if (!hard_stopped) ::i2c_ll_get_txfifo_len(_hw, &free);
    const uint32_t occupancy = free >= SOC_I2C_FIFO_LEN ? 0 : SOC_I2C_FIFO_LEN - free;
    if (!hard_stopped) {
        auto session = _queued_bridge.isrBegin(generation);
        if (session.status() == detail::SlaveQueueBridgeResult::Accepted) {
            uint32_t rx = 0;
            ::i2c_ll_get_rxfifo_cnt(_hw, &rx);
            if (rx != 0) {
                auto prepared = _queued_bridge.prepareRx(session, rx);
                if (prepared.status == detail::SlaveQueueBridgeResult::Accepted) {
                    if (prepared.first.size != 0)
                        ::i2c_ll_read_rxfifo(_hw, prepared.first.data, static_cast<uint32_t>(prepared.first.size));
                    if (prepared.second.size != 0)
                        ::i2c_ll_read_rxfifo(_hw, prepared.second.data, static_cast<uint32_t>(prepared.second.size));
                    if (_queued_bridge.commitRx(session) != detail::SlaveQueueBridgeResult::Accepted)
                        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
                } else {
                    uint8_t scratch[SOC_I2C_FIFO_LEN];
                    while (rx != 0) {
                        const uint32_t count = std::min<uint32_t>(rx, sizeof(scratch));
                        ::i2c_ll_read_rxfifo(_hw, scratch, count);
                        dropped_rx += count;
                        rx -= count;
                    }
                }
            }
            if (_queued_bridge.boundaryRequired()) (void)_queued_bridge.abortBoundary(session, occupancy);
        } else {
            __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        }
        ::i2c_ll_txfifo_rst(_hw);
        ::i2c_ll_rxfifo_rst(_hw);
    }
    _queued_be_reload_pending = false;
    (void)_queued_bridge.workerRequestClose(generation);
    portEXIT_CRITICAL_SAFE(&_mux);
    if (dropped_rx != 0) {
        auto& queued_accessor = static_cast<SlaveAccessor&>(accessor);
        queued_accessor.backendRecordDroppedBytes(dropped_rx);
        queued_accessor.backendEvents().publish(slave::SlaveEvent::Overflow, queued_accessor.readable(),
                                                queued_accessor.writable(), generation);
    }
    if (hard_stopped) restorePins();
    notifyTaskFromTask();
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
    processQueuedWorker();
#endif
    bool forced      = false;
    bool abort_error = false;
    for (;;) {
        if (_queued_bridge.workerTryQuiesce(generation)) break;
        if (bus::remainingTimeout(context.runtime, runtime::millis()) == 0) {
            forced      = true;
            abort_error = !stopQueuedProducerAndQuiesce(generation).has_value();
            break;
        }
        notifyTaskFromTask();
        ::vTaskDelay(1);
    }
    portENTER_CRITICAL_SAFE(&_mux);
    _queued_active     = false;
    _queued_closing    = false;
    _queued_accessor   = nullptr;
    _queued_context    = nullptr;
    _queued_generation = 0;
    __atomic_store_n(&_queued_broken,
                     (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0 || _queued_bridge.broken()) ? 1u : 0u,
                     __ATOMIC_RELEASE);
    portEXIT_CRITICAL_SAFE(&_mux);
    const bool reset = _queued_bridge.workerReset();
    if (!reset) return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    const bool broken = __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0;
    if (abort_error) return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    if (hard_stopped || forced) return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    if (broken) return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    return {};
}

void SlaveBus_espidf::notifyOperationActivity(bus::IAccessor* owner)
{
    portENTER_CRITICAL_SAFE(&_mux);
    const bool notify = _queued_active && _queued_accessor == owner;
    if (notify && _hw != nullptr && !::i2c_ll_is_bus_busy(_hw)) {
        // Replace the mandatory fill preload when real bytes arrive before
        // the next transaction, without claiming the fill was clocked.
        ::i2c_ll_set_slave_addr(_hw, 0x3FFu, true);
        ::i2c_ll_update(_hw);
        _queued_be_reload_pending = true;
    }
    portEXIT_CRITICAL_SAFE(&_mux);
    if (notify) notifyTaskFromTask();
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
    if (notify) {
        processQueuedWorker();
        processQueuedWorker();
    }
#endif
}

void SlaveBus_espidf::processQueuedWorker()
{
    SlaveAccessor* accessor                                = nullptr;
    bus::OperationContext<i2c::SlaveAccessConfig>* context = nullptr;
    uint32_t generation                                    = 0;
    portENTER_CRITICAL_SAFE(&_mux);
    if (_queued_active) {
        accessor   = _queued_accessor;
        context    = _queued_context;
        generation = _queued_generation;
    }
    portEXIT_CRITICAL_SAFE(&_mux);
    if (accessor == nullptr || context == nullptr) return;

    auto worker = _queued_bridge.workerBeginSession(generation);
    if (worker.status() != detail::SlaveQueueBridgeResult::Accepted) return;
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
    if (_test_fail_next_queued_begin) {
        _test_fail_next_queued_begin = false;
        _test_held_queued_worker = new QueuedBridge::WorkerSession(static_cast<QueuedBridge::WorkerSession&&>(worker));
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        return;
    }
#endif

    auto publish = [&](slave::SlaveEvent events) {
        accessor->backendEvents().publish(events, accessor->readable(), accessor->writable(), generation);
    };
    const uint32_t dropped_rx = __atomic_exchange_n(&_queued_dropped_rx, 0, __ATOMIC_ACQ_REL);
    if (dropped_rx != 0) {
        accessor->backendRecordDroppedBytes(dropped_rx);
        publish(slave::SlaveEvent::Overflow);
    }
    if (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0 || _queued_bridge.broken()) {
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        publish(slave::SlaveEvent::BusBroken);
    }
    auto confirm = [&](const detail::SlaveTxTotals& totals, bool confirm_bridge_real) -> bool {
        if (totals.real != 0) {
            if (confirm_bridge_real && _queued_bridge.workerConfirmTxReal(worker, static_cast<uint32_t>(totals.real)) !=
                                           detail::SlaveQueueBridgeResult::Accepted) {
                return false;
            }
            auto popped = accessor->backendTxQueue().popBytes(totals.real);
            if (!popped.has_value()) {
                return false;
            }
            _queued_popped_confirmed += totals.real;
            publish(slave::SlaveEvent::TxSpace);
        }
        if (totals.fill != 0) {
            accessor->backendTxQueue().recordUnderrun(static_cast<uint32_t>(totals.fill));
            publish(slave::SlaveEvent::Underrun);
        }
        return true;
    };

    for (size_t iteration = 0; iteration < kQueuedEventCapacity; ++iteration) {
        detail::SlaveQueueRawEvent event;
        uint8_t rx_bytes[kQueuedRxCapacity];
        const auto peeked = _queued_bridge.workerPeekStep(worker, event, rx_bytes, sizeof(rx_bytes));
        if (peeked == detail::SlaveQueueWorkerStep::Empty) break;
        if (peeked != detail::SlaveQueueWorkerStep::Ready) {
            __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
            publish(slave::SlaveEvent::BusBroken);
            break;
        }

        bool commit   = true;
        bool boundary = false;
        detail::SlaveTxBoundaryResult boundary_result{};
        switch (event.kind) {
            case detail::SlaveQueueRawEventKind::RxPayload: {
                if (accessor->backendRxWritable() < event.count) return;
                auto written = accessor->backendWriteRx({rx_bytes, event.count});
                if (!written.has_value() || *written != event.count) return;
                publish(slave::SlaveEvent::RxAvailable);
                break;
            }
            case detail::SlaveQueueRawEventKind::TxLoadedReal:
                commit = _queued_tx_ledger.load(detail::SlaveTxProvenance::Real, event.count).has_value();
                break;
            case detail::SlaveQueueRawEventKind::TxLoadedFill:
                commit = _queued_tx_ledger.load(detail::SlaveTxProvenance::Fill, event.count).has_value();
                break;
            case detail::SlaveQueueRawEventKind::FifoOccupancy: {
                auto observed = _queued_tx_ledger.observeFifoOccupancy(event.value);
                commit        = observed.has_value() && confirm(*observed, true);
                break;
            }
            case detail::SlaveQueueRawEventKind::StopBoundary:
            case detail::SlaveQueueRawEventKind::TxEmptyBoundary:
            case detail::SlaveQueueRawEventKind::AbortBoundary: {
                const auto evidence = event.kind == detail::SlaveQueueRawEventKind::TxEmptyBoundary
                                          ? detail::SlaveTxBoundaryEvidence::ShifterDrained
                                          : detail::SlaveTxBoundaryEvidence::ShifterAmbiguous;
                auto stopped        = _queued_tx_ledger.confirmBoundaryAndDiscard(event.value, evidence);
                if (stopped.has_value()) {
                    boundary_result = *stopped;
                    boundary        = true;
                } else {
                    commit = false;
                }
                break;
            }
        }
        if (!commit || _queued_bridge.workerCommitStep(worker) != detail::SlaveQueueBridgeResult::Accepted) {
            __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
            publish(slave::SlaveEvent::BusBroken);
            break;
        }
        if (boundary) {
            if (_queued_bridge.workerResolveTxBoundary(worker, static_cast<uint32_t>(boundary_result.confirmed.real),
                                                       static_cast<uint32_t>(boundary_result.unclocked.real)) !=
                    detail::SlaveQueueBridgeResult::Accepted ||
                !confirm(boundary_result.confirmed, false)) {
                __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
                publish(slave::SlaveEvent::BusBroken);
                break;
            }
        }
    }

    if (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) == 0 &&
        _queued_bridge.workerState() == detail::SlaveQueueBridgeState::Active) {
        const size_t writable = _queued_bridge.workerTxWritable();
        const size_t borrowed = _queued_copied_unique - _queued_popped_confirmed;
        if (writable != 0) {
            auto bytes = accessor->backendTxQueue().peekBytes(borrowed + writable);
            if (bytes.has_value()) {
                const size_t available  = bytes->first.size + bytes->second.size;
                const size_t copy_count = available > borrowed ? std::min(writable, available - borrowed) : 0;
                uint8_t local[kQueuedTxCapacity];
                for (size_t i = 0; i < copy_count; ++i) {
                    const size_t source = borrowed + i;
                    local[i]            = source < bytes->first.size
                                              ? static_cast<const uint8_t*>(bytes->first.data)[source]
                                              : static_cast<const uint8_t*>(bytes->second.data)[source - bytes->first.size];
                }
                if (copy_count != 0 && _queued_bridge.workerStageTx(worker, local, copy_count) ==
                                           detail::SlaveQueueBridgeResult::Accepted) {
                    _queued_copied_unique += copy_count;
                }
            }
        }
    }
    portENTER_CRITICAL_SAFE(&_mux);
    ++_queued_worker_runs;
    portEXIT_CRITICAL_SAFE(&_mux);
    if (_queued_bridge.broken()) {
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        publish(slave::SlaveEvent::BusBroken);
    }
    // The worker lease ends before the task-side ISR lease below. Together with
    // _mux, bridge sessions make the hardware FIFO a sole serialized consumer.
    resumeQueuedHardwareFromTask();
}

void SlaveBus_espidf::resumeQueuedHardwareFromTask()
{
    portENTER_CRITICAL_SAFE(&_mux);
    if (!_queued_active || _queued_closing || !_queued_be_reload_pending || _hw == nullptr ||
        __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0 || _queued_bridge.txPaused()) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return;
    }
    if (::i2c_ll_is_bus_busy(_hw)) {
        // An address may have won just before the task-side fence. Its STOP
        // ISR establishes the real boundary and requests the same reload.
        portEXIT_CRITICAL_SAFE(&_mux);
        return;
    }
    auto session = _queued_bridge.isrBegin(_queued_generation);
    if (session.status() != detail::SlaveQueueBridgeResult::Accepted) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return;
    }
    if (_queued_bridge.boundaryRequired()) {
        uint32_t free = 0;
        ::i2c_ll_get_txfifo_len(_hw, &free);
        const uint32_t occupancy = free >= SOC_I2C_FIFO_LEN ? 0 : SOC_I2C_FIFO_LEN - free;
        if (_queued_bridge.abortBoundary(session, occupancy) != detail::SlaveQueueBridgeResult::Accepted) {
            __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        } else {
            ::i2c_ll_disable_intr_mask(_hw, I2C_LL_INTR_MASK);
            ::i2c_ll_txfifo_rst(_hw);
        }
        portEXIT_CRITICAL_SAFE(&_mux);
        notifyTaskFromTask();
        return;
    }
    ::i2c_ll_txfifo_rst(_hw);
    uint8_t local[SOC_I2C_FIFO_LEN];
    auto prepared = _queued_bridge.prepareTx(session, local, sizeof(local));
    if (prepared.status != detail::SlaveQueueBridgeResult::Accepted) {
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        portEXIT_CRITICAL_SAFE(&_mux);
        return;
    }
    ::i2c_ll_write_txfifo(_hw, local, static_cast<uint32_t>(prepared.count));
    if (_queued_bridge.commitTxLoaded(session) != detail::SlaveQueueBridgeResult::Accepted) {
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        portEXIT_CRITICAL_SAFE(&_mux);
        return;
    }
    _queued_be_reload_pending = false;
    ::i2c_ll_set_slave_addr(_hw, _config.address, false);
    ::i2c_ll_enable_intr_mask(
        _hw, impl_espidf_slave::kBeRxWmIntr | I2C_TRANS_COMPLETE_INT_ENA_M | impl_espidf_slave::kBeTxWmIntr);
    ::i2c_ll_update(_hw);
    portEXIT_CRITICAL_SAFE(&_mux);
}

bool SlaveBus_espidf::bindIsrRegMap(IsrRegMapBinding* binding)
{
    if (binding == nullptr) {
        return false;
    }
    portENTER_CRITICAL_SAFE(&_mux);
    if (_queued_active) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return false;
    }
    // Last-bind-wins: just take the slot. The superseded binding (if any) is
    // left untouched -- its owner still believes it is bound until it calls
    // unbindIsrRegMap, which is then a no-op (ownership check there).
    // Per-transaction wire state resets on every (re)bind; `pointer` itself is
    // NOT touched here -- the caller's initial/persisted value is respected.
    binding->pointer_received = false;
    binding->write_offset     = 0;
    binding->tx_offset        = 0;
    _isr_binding              = binding;
    // Re-compose the TX FIFO from the now-bound register map so a read arriving
    // before the next write/STOP sees regmap data instead of whatever the
    // plain-stream priming (init(), or a prior binding's hooks) had queued.
    rebuildTxRegMapLocked();
    portEXIT_CRITICAL_SAFE(&_mux);
    return true;
}

void SlaveBus_espidf::unbindIsrRegMap(IsrRegMapBinding* binding)
{
    portENTER_CRITICAL_SAFE(&_mux);
    // Ownership check: only clear the slot if `binding` is still the one bound
    // -- a caller whose binding was already superseded by a later bind (see
    // above) must not rip out the newer one.
    if (_isr_binding == binding) {
        _isr_binding = nullptr;
    }
    portEXIT_CRITICAL_SAFE(&_mux);
}

// Drain RX-FIFO bytes into the current transaction's rx ring as raw stream
// bytes (no register interpretation). Used only while no regmap fast path is
// bound -- see drainRxRegMapLocked for that path. No back-pressure: see the
// header contract.
void SlaveBus_espidf::drainRxLocked(uint32_t count)
{
    if (count == 0) {
        return;
    }
    if (_current == nullptr) {
        _current = allocateTransactionLocked();
        if (_current == nullptr) {
            // No transaction slot to hold the bytes: drain to scratch to keep the
            // HW sane and surface the loss.
            uint8_t scratch[SOC_I2C_FIFO_LEN];
            while (count) {
                uint8_t c = (count > SOC_I2C_FIFO_LEN) ? SOC_I2C_FIFO_LEN : static_cast<uint8_t>(count);
                ::i2c_ll_read_rxfifo(_hw, scratch, c);
                _rx_overflow_count += c;
                count -= c;
            }
            return;
        }
    }
    uint8_t buf[SOC_I2C_FIFO_LEN];
    while (count) {
        const size_t backlog   = _current->rx_size - _current->rx_read;
        const size_t freespace = (backlog < kRxArrayCapacity) ? (kRxArrayCapacity - backlog) : 0;
        uint8_t c              = (count > SOC_I2C_FIFO_LEN) ? SOC_I2C_FIFO_LEN : static_cast<uint8_t>(count);
        if (freespace == 0) {
            // Ring full and no later hold to lift it: drop the tail (BE has no
            // back-pressure primitive), surfaced via rxOverflowCount().
            ::i2c_ll_read_rxfifo(_hw, buf, c);
            _rx_overflow_count += c;
            count -= c;
            continue;
        }
        if (c > freespace) {
            c = static_cast<uint8_t>(freespace);
        }
        ::i2c_ll_read_rxfifo(_hw, buf, c);
        for (uint8_t i = 0; i < c; ++i) {
            _current->rx[(_current->rx_size++) & (kRxArrayCapacity - 1)] = buf[i];
        }
        count -= c;
    }
}

// Same contract as the LL flavor's snapshotResponseLocked (see its comment):
// pulls the open transaction's unread tx bytes into _resp, only once the
// current snapshot is fully sent. Must run with _mux held.
void SlaveBus_espidf::snapshotResponseLocked()
{
    if (_resp_pos < _resp_len) {
        return;
    }
    _resp_len        = 0;
    _resp_pos        = 0;
    Transaction* txn = _open;
    if (txn == nullptr || txn != _current) {
        return;
    }
    while (txn->tx_read < txn->tx_size && _resp_len < kTxCapacity) {
        _resp[_resp_len++] = txn->tx[(txn->tx_read++) & (kTxCapacity - 1)];
    }
}

// Fill the TX FIFO from the response snapshot. Unlike the LL flavor (which
// only needs one fill byte per underrun -- the master stays held under the
// stretch until the next call), BE has no hold: an exhausted _resp must top
// the WHOLE free space with tx_fill_byte, or the FIFO stays below the water
// mark and the level-type TX interrupt spins even with the bus idle. Topping
// fully also means a write phase's FIFO -- already topped from the last
// STOP -- never drops back below threshold on its own, so no direction check
// (is_read) is needed to keep the interrupt from storming during writes.
// Must run with _mux held.
void SlaveBus_espidf::fillTxFromRespLocked()
{
    uint32_t freelen = 0;
    ::i2c_ll_get_txfifo_len(_hw, &freelen);
    if (freelen > SOC_I2C_FIFO_LEN) {
        freelen = SOC_I2C_FIFO_LEN;
    }
    if (freelen == 0) {
        return;
    }
    uint8_t buf[SOC_I2C_FIFO_LEN];
    uint32_t n = 0;
    while (n < freelen && _resp_pos < _resp_len) {
        buf[n++] = _resp[_resp_pos++];
    }
    while (n < freelen) {
        buf[n++] = _config.tx_fill_byte;
    }
    ::i2c_ll_write_txfifo(_hw, buf, static_cast<uint8_t>(n));
}

// Drain RX-FIFO bytes directly into the bound register map: the first byte of
// each transaction sets _isr_binding->pointer, subsequent bytes store via
// regMapWriteByte (auto-increment, firing onWrite). No back-pressure and no
// Transaction.rx[] involvement -- see the header contract. Must run with
// _mux held, and only while _isr_binding != nullptr.
void SlaveBus_espidf::drainRxRegMapLocked(uint32_t count)
{
    IsrRegMapBinding* binding = _isr_binding;
    uint8_t buf[SOC_I2C_FIFO_LEN];
    while (count) {
        uint8_t c = (count > SOC_I2C_FIFO_LEN) ? SOC_I2C_FIFO_LEN : static_cast<uint8_t>(count);
        ::i2c_ll_read_rxfifo(_hw, buf, c);
        for (uint8_t i = 0; i < c; ++i) {
            if (!binding->pointer_received) {
                binding->pointer_received = true;
                binding->pointer          = buf[i];
            } else {
                regMapWriteByte(binding->reg_file, static_cast<uint8_t>(binding->pointer + binding->write_offset),
                                buf[i], binding->on_write, binding->on_write_ctx);
                ++binding->write_offset;
            }
        }
        count -= c;
    }
}

// Fill the TX FIFO's free space from the bound register map starting at
// _isr_binding->pointer + tx_offset (auto-increment, 8-bit wrap via
// regMapReadByte). Unlike fillTxFromRespLocked there is no underrun case --
// the reply IS the register map, so there is always a next byte. Must run
// with _mux held, and only while _isr_binding != nullptr.
void SlaveBus_espidf::fillTxRegMapLocked()
{
    IsrRegMapBinding* binding = _isr_binding;
    uint32_t space            = 0;
    ::i2c_ll_get_txfifo_len(_hw, &space);
    if (space == 0) {
        return;
    }
    if (space > SOC_I2C_FIFO_LEN) {
        space = SOC_I2C_FIFO_LEN;
    }
    uint8_t buf[SOC_I2C_FIFO_LEN];
    for (uint32_t i = 0; i < space; ++i) {
        buf[i] = regMapReadByte(binding->reg_file, static_cast<uint8_t>(binding->pointer + binding->tx_offset + i),
                                binding->on_read, binding->on_read_ctx);
    }
    ::i2c_ll_write_txfifo(_hw, buf, static_cast<uint8_t>(space));
    binding->tx_offset += space;
}

// Discard whatever the TX FIFO holds and refill it from the register map at
// the current pointer (offset 0). A no-op before init() (_hw == nullptr) --
// bindIsrRegMap can run before the bus is initialized in test code. Must run
// with _mux held, and only while _isr_binding != nullptr.
void SlaveBus_espidf::rebuildTxRegMapLocked()
{
    if (_hw == nullptr || _isr_binding == nullptr) {
        return;
    }
    ::i2c_ll_txfifo_rst(_hw);
    _isr_binding->tx_offset = 0;
    fillTxRegMapLocked();
}

void SlaveBus_espidf::isrThunk(void* arg)
{
    static_cast<SlaveBus_espidf*>(arg)->handleIsr();
}

bool SlaveBus_espidf::handleQueuedBeIsrLocked(::i2c_dev_t* hw, uint32_t ints, uint32_t rx, bool& task_woken)
{
    if (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0) {
        uint8_t scratch[SOC_I2C_FIFO_LEN];
        uint32_t remaining = rx;
        while (remaining != 0) {
            const uint32_t chunk = std::min<uint32_t>(remaining, sizeof(scratch));
            ::i2c_ll_read_rxfifo(hw, scratch, chunk);
            remaining -= chunk;
        }
        if (rx != 0) __atomic_fetch_add(&_queued_dropped_rx, rx, __ATOMIC_RELAXED);
        if (ints & I2C_TRANS_COMPLETE_INT_ENA_M) {
            ::i2c_ll_disable_intr_mask(hw, I2C_LL_INTR_MASK);
            ::i2c_ll_txfifo_rst(hw);
            ::i2c_ll_rxfifo_rst(hw);
        } else {
            // Keep RX/STOP enabled until the accepted write reaches STOP so
            // every subsequently clocked byte is drained and counted exactly.
            ::i2c_ll_disable_intr_mask(hw, impl_espidf_slave::kBeTxWmIntr);
        }
        notifyTaskFromISR(task_woken);
        return false;
    }
    auto session = _queued_bridge.isrBegin(_queued_generation);
    if (session.status() != detail::SlaveQueueBridgeResult::Accepted) return false;
    bool activity      = false;
    auto break_backend = [&]() __attribute__((always_inline))
    {
        __atomic_store_n(&_queued_broken, 1, __ATOMIC_RELEASE);
        ::i2c_ll_set_slave_addr(hw, 0x3FFu, true);
        ::i2c_ll_disable_intr_mask(hw, impl_espidf_slave::kBeTxWmIntr);
        ::i2c_ll_update(hw);
    };
    auto drain_rx = [&](uint32_t count) __attribute__((always_inline))
    {
        if (count == 0) return true;
        auto prepared = _queued_bridge.prepareRx(session, count);
        if (prepared.status != detail::SlaveQueueBridgeResult::Accepted) {
            uint8_t scratch[SOC_I2C_FIFO_LEN];
            uint32_t remaining = count;
            while (remaining != 0) {
                const uint32_t chunk = std::min<uint32_t>(remaining, sizeof(scratch));
                ::i2c_ll_read_rxfifo(hw, scratch, chunk);
                remaining -= chunk;
            }
            __atomic_fetch_add(&_queued_dropped_rx, count, __ATOMIC_RELAXED);
            break_backend();
            return false;
        }
        if (prepared.first.size != 0)
            ::i2c_ll_read_rxfifo(hw, prepared.first.data, static_cast<uint32_t>(prepared.first.size));
        if (prepared.second.size != 0)
            ::i2c_ll_read_rxfifo(hw, prepared.second.data, static_cast<uint32_t>(prepared.second.size));
        if (_queued_bridge.commitRx(session) != detail::SlaveQueueBridgeResult::Accepted) {
            break_backend();
            return false;
        }
        activity = true;
        return true;
    };
    auto refill_tx = [&]() __attribute__((always_inline))
    {
        uint32_t free = 0;
        ::i2c_ll_get_txfifo_len(hw, &free);
        const uint32_t occupancy = free >= SOC_I2C_FIFO_LEN ? 0 : SOC_I2C_FIFO_LEN - free;
        if (_queued_bridge.observeFifo(session, occupancy) != detail::SlaveQueueBridgeResult::Accepted) {
            break_backend();
            return false;
        }
        if (free != 0) {
            uint8_t local[SOC_I2C_FIFO_LEN];
            auto prepared = _queued_bridge.prepareTx(session, local, std::min<size_t>(free, sizeof(local)));
            if (prepared.status != detail::SlaveQueueBridgeResult::Accepted) {
                break_backend();
                return false;
            }
            ::i2c_ll_write_txfifo(hw, local, static_cast<uint32_t>(prepared.count));
            if (_queued_bridge.commitTxLoaded(session) != detail::SlaveQueueBridgeResult::Accepted) {
                break_backend();
                return false;
            }
        }
        activity = true;
        return true;
    };
    if ((ints & impl_espidf_slave::kBeRxWmIntr) && rx) {
        (void)drain_rx(rx);
        rx = 0;
    }
    if (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) == 0 && (ints & impl_espidf_slave::kBeTxWmIntr))
        (void)refill_tx();
    if (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) == 0 && (ints & I2C_TRANS_COMPLETE_INT_ENA_M)) {
        if (rx) (void)drain_rx(rx);
        if (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) == 0) {
            uint32_t free = 0;
            ::i2c_ll_get_txfifo_len(hw, &free);
            const uint32_t occupancy = free >= SOC_I2C_FIFO_LEN ? 0 : SOC_I2C_FIFO_LEN - free;
            if (_queued_bridge.stopBoundary(session, occupancy) != detail::SlaveQueueBridgeResult::Accepted) {
                break_backend();
            } else {
                ::i2c_ll_set_slave_addr(hw, 0x3FFu, true);
                ::i2c_ll_disable_intr_mask(hw, I2C_LL_INTR_MASK);
                ::i2c_ll_txfifo_rst(hw);
                ::i2c_ll_update(hw);
                _queued_be_reload_pending = true;
                activity                  = true;
            }
        }
    }
    if (__atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0 && (ints & I2C_TRANS_COMPLETE_INT_ENA_M)) {
        ::i2c_ll_disable_intr_mask(hw, I2C_LL_INTR_MASK);
        ::i2c_ll_txfifo_rst(hw);
        ::i2c_ll_rxfifo_rst(hw);
    }
    if (activity || __atomic_load_n(&_queued_broken, __ATOMIC_ACQUIRE) != 0) notifyTaskFromISR(task_woken);
    return activity;
}

void SlaveBus_espidf::handleIsr()
{
    ::i2c_dev_t* hw = _hw;
    if (hw == nullptr) {
        return;
    }
    bool woken = false;

    uint32_t ints = 0;
    ::i2c_ll_get_intr_mask(hw, &ints);
    ::i2c_ll_clear_intr_mask(hw, ints);

    uint32_t rx = 0;
    ::i2c_ll_get_rxfifo_cnt(hw, &rx);

    portENTER_CRITICAL_ISR(&_mux);

    if (_queued_active) {
        (void)handleQueuedBeIsrLocked(hw, ints, rx, woken);
        portEXIT_CRITICAL_ISR(&_mux);
#if defined(M5HAL_TEST_ESPIDF_I2C_SLAVE_HOST_HARNESS)
        processQueuedWorker();
#endif
        if (woken) portYIELD_FROM_ISR();
        return;
    }

    // Snapshot once per pass: bindIsrRegMap/unbindIsrRegMap only run in task
    // context under this same _mux, so this cannot flip mid-pass.
    const bool regmap_fast_path = (_isr_binding != nullptr);

    // RX water-mark: drain written bytes. The regmap fast path (see the class
    // comment) interprets bytes as pointer/register-data directly and
    // re-composes the TX FIFO from the possibly-new pointer right here -- so a
    // repeated-START read that follows sees correct data without waiting on a
    // task. Without a binding, bytes land in the Transaction rx[] ring like the
    // plain streaming path.
    if ((ints & impl_espidf_slave::kBeRxWmIntr) && rx) {
        if (regmap_fast_path) {
            drainRxRegMapLocked(rx);
            rebuildTxRegMapLocked();
        } else {
            drainRxLocked(rx);
        }
        rx = 0;
    }

    // TX water-mark: the FIFO dropped below the threshold. Per the class
    // comment, this only happens while genuinely being read (a write phase's
    // FIFO stays topped from the last STOP and never re-triggers it). Continue
    // filling from wherever the active source (regmap or the stream _resp
    // snapshot) left off.
    if (ints & impl_espidf_slave::kBeTxWmIntr) {
        if (regmap_fast_path) {
            fillTxRegMapLocked();
        } else {
            snapshotResponseLocked();
            fillTxFromRespLocked();
        }
    }

    // STOP: close out the current transaction and pre-arm a fresh one so the
    // TX FIFO can be topped up right here -- BE has no address-match interrupt
    // to defer that to (unlike the LL flavor). Still tracked even under the
    // regmap fast path (Transaction.complete is what SlaveRegMapAccessor::
    // serve()'s fast-path branch waits on) -- only the byte payload bypasses
    // Transaction.rx[]/tx[].
    if (ints & I2C_TRANS_COMPLETE_INT_ENA_M) {
        if (rx) {
            if (regmap_fast_path) {
                drainRxRegMapLocked(rx);
            } else {
                drainRxLocked(rx);
            }
            rx = 0;
        }
        ::i2c_ll_txfifo_rst(hw);
        if (_current != nullptr) {
            _current->complete = true;
        }
        _current         = allocateTransactionLocked();
        _request_pending = false;
        if (regmap_fast_path) {
            // Next transaction's first byte is a new pointer (SPLIT / repeat-
            // read rely on the binding's pointer itself persisting, so it is
            // NOT reset here).
            _isr_binding->pointer_received = false;
            _isr_binding->write_offset     = 0;
            rebuildTxRegMapLocked();
        } else {
            _resp_len = 0;
            _resp_pos = 0;
            snapshotResponseLocked();
            fillTxFromRespLocked();
        }
    }

    // Wake the serve() consumer on this activity (RX drained or STOP) so it
    // drains/opens without waiting out a poll delay.
    notifyConsumerFromISR(woken);

    portEXIT_CRITICAL_ISR(&_mux);

    if (woken) {
        portYIELD_FROM_ISR();
    }
}

#else
// ---------------------------------------------------------------------------
// Classic ESP32 path: existing v2 driver fill-only best-effort backend.
// ---------------------------------------------------------------------------

result_t<void> SlaveBus_espidf::init(const i2c::SlaveBusConfig& cfg)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (cfg.pin_scl < 0 || cfg.pin_sda < 0 || cfg.address_is_10bit || cfg.address > 0x7Fu) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if !defined(SOC_I2C_SUPPORT_SLAVE) || !SOC_I2C_SUPPORT_SLAVE
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
#endif
#if !defined(SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE) || !SOC_I2C_SLAVE_CAN_GET_STRETCH_CAUSE
    if (cfg.tx_underrun == i2c::TxUnderrun::Stretch) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#endif
    // The callback driver cannot provide the ordered read-completion facts
    // required by the exact queue contract. Keep it legacy-only and reject the
    // default lifecycle before creating a slave device that would already ACK.
    if (!cfg.legacy_wire_frame_window) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    const auto port_r = impl_espidf_slave::resolveSlaveControllerPort(cfg.controller);
    if (!port_r.has_value()) {
        return m5::stl::make_unexpected(port_r.error());
    }

    if (_handle != nullptr || _task != nullptr) {
        auto reset = resetForInitialization();
        if (!reset.has_value()) {
            return reset;
        }
    }
    _config = cfg;

    {
        portENTER_CRITICAL_SAFE(&_mux);
        resetStateLocked();
        _task_stop    = false;
        _task_running = true;
        portEXIT_CRITICAL_SAFE(&_mux);
    }
    if (::xTaskCreate(&SlaveBus_espidf::taskThunk, "m5hal_i2c_slave", 3072, this, tskIDLE_PRIORITY + 2, &_task) !=
        pdPASS) {
        portENTER_CRITICAL_SAFE(&_mux);
        _task_running = false;
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    ::i2c_slave_config_t native_cfg         = {};
    native_cfg.i2c_port                     = static_cast<decltype(native_cfg.i2c_port)>(port_r.value());
    native_cfg.sda_io_num                   = static_cast<::gpio_num_t>(cfg.pin_sda);
    native_cfg.scl_io_num                   = static_cast<::gpio_num_t>(cfg.pin_scl);
    native_cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
    native_cfg.send_buf_depth               = kDriverTxDepth;
    native_cfg.receive_buf_depth            = kDriverRxDepth;
    native_cfg.slave_addr                   = cfg.address;
    native_cfg.addr_bit_len                 = I2C_ADDR_BIT_LEN_7;
    native_cfg.intr_priority                = 0;
    native_cfg.flags.enable_internal_pullup = 1;

    auto mapped = impl_espidf_slave::mapEspErr(::i2c_new_slave_device(&native_cfg, &_handle));
    if (error::isError(mapped)) {
        auto reset = resetForInitialization();
        if (!reset.has_value()) {
            return reset;
        }
        return m5::stl::make_unexpected(mapped);
    }

    ::i2c_slave_event_callbacks_t callbacks = {};
    callbacks.on_receive                    = &SlaveBus_espidf::onReceive;
    callbacks.on_request                    = &SlaveBus_espidf::onRequest;
    mapped = impl_espidf_slave::mapEspErr(::i2c_slave_register_event_callbacks(_handle, &callbacks, this));
    if (error::isError(mapped)) {
        auto reset = resetForInitialization();
        if (!reset.has_value()) {
            return reset;
        }
        return m5::stl::make_unexpected(mapped);
    }
    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        (void)resetForInitialization();
        return initialized;
    }
    return {};
}

bus::CloseOutcome SlaveBus_espidf::teardownBackend(void)
{
    if (_task != nullptr) {
        portENTER_CRITICAL_SAFE(&_mux);
        _task_stop = true;
        portEXIT_CRITICAL_SAFE(&_mux);
        notifyTaskFromTask();
        if (!impl_espidf_slave::waitTaskStopped(&_mux, _task_running)) {
            return bus::CloseOutcome::partialOrUnknown(error::error_t::TIMEOUT_ERROR);
        }
        _task = nullptr;
    }

    if (_handle != nullptr) {
        auto mapped = impl_espidf_slave::mapEspErr(::i2c_del_slave_device(_handle));
        if (error::isError(mapped)) {
            return bus::CloseOutcome::partialOrUnknown(mapped);
        }
        _handle = nullptr;
    }

    portENTER_CRITICAL_SAFE(&_mux);
    resetStateLocked();
    _task_stop    = false;
    _task_running = false;
    portEXIT_CRITICAL_SAFE(&_mux);
    return bus::CloseOutcome::success();
}

bool SlaveBus_espidf::onReceive(::i2c_slave_dev_handle_t handle, const ::i2c_slave_rx_done_event_data_t* evt_data,
                                void* user_data)
{
    (void)handle;
    auto* self = static_cast<SlaveBus_espidf*>(user_data);
    if (self == nullptr || evt_data == nullptr) {
        return false;
    }
    bool task_woken = false;
    portENTER_CRITICAL_ISR(&self->_mux);
    // ESP-IDF defines length as the size of the callback's received buffer.
    // A non-zero length with a null buffer violates that contract, so it
    // contributes no valid bytes and must not create readable zero-filled data.
    const size_t received_len = evt_data->buffer != nullptr ? static_cast<size_t>(evt_data->length) : 0;
    auto* txn                 = self->allocateTransactionLocked();
    if (txn != nullptr) {
        // Non-LL callback path: the driver delivers the WHOLE transaction post-STOP
        // in one buffer, so there is no mid-transaction draining to make room. Unlike
        // the streaming drainRxLocked() ring, kRxArrayCapacity is a hard
        // per-transaction cap here; a longer transaction is truncated (surfaced via
        // rxOverflowCount). The bytes land at rx[0..copy_len) with rx_read = 0, which
        // is the ring's natural start, so the wrap-aware read() handles this path
        // unchanged.
        const size_t copy_len = std::min(received_len, kRxArrayCapacity);
        if (copy_len > 0) {
            ::memcpy(txn->rx, evt_data->buffer, copy_len);
        }
        txn->rx_size   = copy_len;
        txn->rx_read   = 0;
        txn->complete  = true;
        self->_current = txn;
        self->_rx_overflow_count += received_len - copy_len;
    } else {
        self->_rx_overflow_count += received_len;
    }
    // Wake the serve() consumer parked in waitForActivity() so a completed
    // transaction is drained promptly instead of waiting out its safety timeout.
    self->notifyConsumerFromISR(task_woken);
    portEXIT_CRITICAL_ISR(&self->_mux);
    return task_woken;
}

bool SlaveBus_espidf::onRequest(::i2c_slave_dev_handle_t handle, const ::i2c_slave_request_event_data_t* evt_data,
                                void* user_data)
{
    (void)handle;
    (void)evt_data;
    auto* self = static_cast<SlaveBus_espidf*>(user_data);
    if (self == nullptr) {
        return false;
    }
    bool task_woken = false;
    portENTER_CRITICAL_ISR(&self->_mux);
    if (self->_current == nullptr) {
        self->_current = self->newestTransactionLocked();
        if (self->_current == nullptr) {
            self->_current = self->allocateTransactionLocked();
        }
    }
    self->_request_pending = true;
    self->_request_tick    = ::xTaskGetTickCountFromISR();
    self->notifyTaskFromISR(task_woken);
    // Also wake the serve() consumer (a separate wait path from the responder
    // task notified above) so a read request is answered without waiting out
    // its safety timeout.
    self->notifyConsumerFromISR(task_woken);
    portEXIT_CRITICAL_ISR(&self->_mux);
    return task_woken;
}

#endif  // M5HAL_ESPIDF_I2C_SLAVE_LL / M5HAL_ESPIDF_I2C_SLAVE_LL_BE / v2 driver

// ===========================================================================
// Shared legacy wire-frame-window state machine (HW independent)
// ===========================================================================

result_t<void> SlaveBus_espidf::tryOpenWireFrame(bus::IAccessor* owner)
{
    if (owner == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    bool should_notify = false;
    portENTER_CRITICAL_SAFE(&_mux);
#if M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_SLAVE_LL_BE
    if (!_config.legacy_wire_frame_window || _queued_lifecycle_used) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
#endif
    if (_open != nullptr) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto* txn = oldestOpenableTransactionLocked();
    if (txn == nullptr) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    txn->opened   = true;
    _open         = txn;
    _open_owner   = owner;
    should_notify = _request_pending;
    portEXIT_CRITICAL_SAFE(&_mux);
    if (should_notify) {
        notifyTaskFromTask();
    }
    return {};
}

result_t<void> SlaveBus_espidf::closeWireFrame(bus::IAccessor* owner)
{
    portENTER_CRITICAL_SAFE(&_mux);
    if (!isOpenOwnerLocked(owner)) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    discardTransactionLocked(*_open);
    _open       = nullptr;
    _open_owner = nullptr;
    portEXIT_CRITICAL_SAFE(&_mux);
    return {};
}

result_t<size_t> SlaveBus_espidf::read(bus::IAccessor* owner, data::DataSpan dst)
{
    if (dst.data == nullptr && dst.size != 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if M5HAL_ESPIDF_I2C_SLAVE_LL
    // RX back-pressure lift state (LL stretch path only; the v2 / non-LL backend has
    // no clock stretch, so read() there is the plain ring drain below).
    bool release_stretch          = false;
    ::i2c_dev_t* hw_release       = nullptr;
    uint32_t enable_after_release = 0;
#endif
    portENTER_CRITICAL_SAFE(&_mux);
    if (!isOpenOwnerLocked(owner)) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const size_t take = std::min(dst.size, readableBytesOf(*_open));
    if (take > 0) {
        // rx[] is a power-of-two ring; the readable span may wrap past the end.
        const size_t start = _open->rx_read & (kRxArrayCapacity - 1);
        const size_t first = std::min(take, kRxArrayCapacity - start);
        ::memcpy(dst.data, _open->rx + start, first);
        if (take > first) {
            ::memcpy(static_cast<uint8_t*>(dst.data) + first, _open->rx, take - first);
        }
        _open->rx_read += take;
    }
#if M5HAL_ESPIDF_I2C_SLAVE_LL
    // Back-pressure lift (write-direction): the ISR is holding the master under an
    // RX_FULL stretch because THIS transaction's ring filled, and we just freed ring
    // space. Drain the HW FIFO into that freed space HERE, BEFORE releasing -- we hold
    // _mux and the RX interrupts (STRETCH + RXFIFO_WM) are masked by the hold, so the
    // ISR cannot touch the FIFO concurrently. Draining first is REQUIRED for the
    // zero-loss invariant: clearing the RX_FULL stretch while the FIFO is still full
    // would let the master clock a byte into a FIFO with no free slot. If the freed
    // ring fills again from the FIFO, stay held -- serve()'s next read() lifts again.
    // Guard on _open == _current: the hold is on the ring the ISR fills (_current), so
    // only a read() that drained THAT ring (the open transaction IS the one in flight)
    // frees the space the drain needs. The HW pokes (clear_stretch + re-enable masked)
    // run after the _mux release, matching write()'s release pattern; the masked
    // interrupts stay disabled until then, so no ISR races this window, and the FIFO
    // now has room so clearing the stretch is safe.
    if (_hold_kind == HoldKind::rx_full && _open == _current && _hw != nullptr &&
        (_open->rx_size - _open->rx_read) < kRxCapacity) {
        uint32_t fifo = 0;
        ::i2c_ll_get_rxfifo_cnt(_hw, &fifo);
        if (fifo != 0) {
            (void)drainRxLocked(fifo, true);  // fill the freed ring from the still-full FIFO
        }
        // Release ONLY if the ring now has room for the master to clock into: if the
        // drain refilled the ring exactly, clearing the stretch would re-expose the
        // no-free-slot race, so stay held -- serve()'s next read() drains more and
        // lifts then (the FIFO is empty now, so that pass releases with room).
        if ((_open->rx_size - _open->rx_read) < kRxCapacity) {
            const uint32_t masked = clearHoldLocked();
            hw_release            = _hw;
            enable_after_release  = masked & _baseline_intrs;
            release_stretch       = true;
        }
    }
#endif  // M5HAL_ESPIDF_I2C_SLAVE_LL
    portEXIT_CRITICAL_SAFE(&_mux);
#if M5HAL_ESPIDF_I2C_SLAVE_LL
    if (release_stretch) {
        ::i2c_ll_slave_clear_stretch(hw_release);
        if (enable_after_release != 0) {
            ::i2c_ll_enable_intr_mask(hw_release, enable_after_release);
        }
    }
#endif
    return take;
}

result_t<size_t> SlaveBus_espidf::write(bus::IAccessor* owner, data::ConstDataSpan src)
{
    if (src.data == nullptr && src.size != 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    bool should_notify = false;
#if M5HAL_ESPIDF_I2C_SLAVE_LL
    // Read-direction stretch release state (LL stretch path only; the v2 / non-LL
    // backend has no clock stretch, so write() there just queues the reply into the
    // tx ring and wakes the responder task to feed the driver).
    bool release_stretch          = false;
    ::i2c_dev_t* hw_release       = nullptr;
    uint32_t enable_after_release = 0;
#endif
    size_t take = 0;
    portENTER_CRITICAL_SAFE(&_mux);
    if (!isOpenOwnerLocked(owner)) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    // tx[] is a power-of-two ring: free space tracks the UNSENT occupancy
    // (tx_size - tx_read), so room opens as the master consumes the reply and the
    // app can stream a reply larger than kTxCapacity across one read. The notify
    // wakes the responder task to refill/resume a held TX_EMPTY stretch.
    const size_t occupancy = _open->tx_size - _open->tx_read;
    const size_t free      = (occupancy < kTxCapacity) ? (kTxCapacity - occupancy) : 0;
    take                   = std::min(src.size, free);
    if (take > 0) {
        const size_t start = _open->tx_size & (kTxCapacity - 1);
        const size_t first = std::min(take, kTxCapacity - start);
        ::memcpy(_open->tx + start, src.data, first);
        if (take > first) {
            ::memcpy(_open->tx, static_cast<const uint8_t*>(src.data) + first, take - first);
        }
        _open->tx_size += take;
#if M5HAL_ESPIDF_I2C_SLAVE_LL
        // _open == _current gates the release (same stale-reply guard as
        // snapshotResponseLocked()): a held read stretch belongs to the WIRE's
        // transaction, so bytes queued onto a lingering completed transaction
        // (serve() still finishing the preceding write's exchange) must NOT
        // trip this release -- snapshotResponseLocked() would yield nothing and
        // the fill-byte underrun branch would hand the master a bogus leading
        // fill byte. Keep the hold and notify instead; the accessor's write()
        // to the wire's own transaction releases it with the real reply.
        if ((_hold_kind == HoldKind::address_read || _hold_kind == HoldKind::tx_empty) && _hw != nullptr &&
            _open == _current) {
            snapshotResponseLocked();
            fillTxFromRespLocked();
            const uint32_t masked = clearHoldLocked();
            hw_release            = _hw;
            enable_after_release  = masked & _baseline_intrs;
            release_stretch       = true;
        } else {
            should_notify = _request_pending;
        }
#else
        should_notify = _request_pending;
#endif
    }
    portEXIT_CRITICAL_SAFE(&_mux);
#if M5HAL_ESPIDF_I2C_SLAVE_LL
    if (release_stretch) {
        ::i2c_ll_slave_clear_stretch(hw_release);
#if M5HAL_DETAIL_I2C_SLAVE_TX_WATERMARK_
        // Arm the proactive TX water-mark top-up: this write() just started a reply
        // flowing, so keep the FIFO topped up before it empties (smooths the read).
        ::i2c_ll_enable_intr_mask(hw_release, I2C_TXFIFO_WM_INT_ENA_M);
#endif
        if (enable_after_release != 0) {
            ::i2c_ll_enable_intr_mask(hw_release, enable_after_release);
        }
    } else if (should_notify) {
        notifyTaskFromTask();
    }
#else
    if (should_notify) {
        notifyTaskFromTask();
    }
#endif
    return take;
}

result_t<size_t> SlaveBus_espidf::readableBytes(bus::IAccessor* owner)
{
    portENTER_CRITICAL_SAFE(&_mux);
    if (!isOpenOwnerLocked(owner)) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const size_t result = readableBytesOf(*_open);
    portEXIT_CRITICAL_SAFE(&_mux);
    return result;
}

result_t<bool> SlaveBus_espidf::wireFrameComplete(bus::IAccessor* owner)
{
    portENTER_CRITICAL_SAFE(&_mux);
    if (!isOpenOwnerLocked(owner)) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const bool result = _open->complete;
    portEXIT_CRITICAL_SAFE(&_mux);
    return result;
}

void SlaveBus_espidf::taskThunk(void* arg)
{
    static_cast<SlaveBus_espidf*>(arg)->requestTaskLoop();
    ::vTaskDelete(nullptr);
}

void SlaveBus_espidf::resetStateLocked()
{
    for (auto& txn : _transactions) {
        txn = Transaction{};
    }
    _current            = nullptr;
    _open               = nullptr;
    _open_owner         = nullptr;
    _request_pending    = false;
    _pending_commit_len = 0;
    _pending_commit_txn = nullptr;
    _pending_commit_seq = 0;
    _next_seq           = 1;
    _rx_overflow_count  = 0;
    _request_tick       = 0;
#if M5HAL_ESPIDF_I2C_SLAVE_LL
    _hold_kind      = HoldKind::none;
    _masked_intrs   = 0;
    _baseline_intrs = 0;
#endif
}

SlaveBus_espidf::Transaction* SlaveBus_espidf::allocateTransactionLocked()
{
    for (auto& txn : _transactions) {
        if (!txn.in_use) {
            txn        = Transaction{};
            txn.in_use = true;
            txn.seq    = _next_seq++;
            return &txn;
        }
    }
    Transaction* oldest = nullptr;
    for (auto& txn : _transactions) {
        if (&txn == _open) {
            continue;
        }
        if (oldest == nullptr || txn.seq < oldest->seq) {
            oldest = &txn;
        }
    }
    if (oldest != nullptr) {
        // Recycling a full transaction table discards any RX backlog that the
        // consumer did not open/drain. Surface the actual number of lost bytes,
        // not one overflow event. rx_read normally cannot exceed rx_size (read()
        // advances it by min(requested, readable)), but keep this subtraction
        // defensive so a corrupted transaction can never underflow the counter.
        if (oldest->rx_read < oldest->rx_size) {
            _rx_overflow_count += oldest->rx_size - oldest->rx_read;
        }
        *oldest        = Transaction{};
        oldest->in_use = true;
        oldest->seq    = _next_seq++;
    }
    return oldest;
}

SlaveBus_espidf::Transaction* SlaveBus_espidf::newestTransactionLocked()
{
    Transaction* best = nullptr;
    for (auto& txn : _transactions) {
        if (!txn.in_use) {
            continue;
        }
        if (best == nullptr || txn.seq > best->seq) {
            best = &txn;
        }
    }
    return best;
}

SlaveBus_espidf::Transaction* SlaveBus_espidf::oldestOpenableTransactionLocked()
{
    Transaction* best = nullptr;
    for (auto& txn : _transactions) {
        if (!txn.in_use || txn.opened) {
            continue;
        }
        if (best == nullptr || txn.seq < best->seq) {
            best = &txn;
        }
    }
    return best;
}

void SlaveBus_espidf::discardTransactionLocked(Transaction& txn)
{
#if M5HAL_ESPIDF_I2C_SLAVE_LL
    if (&txn == _current && _hold_kind == HoldKind::rx_full) {
        const uint32_t masked = clearHoldLocked();
        if (_hw != nullptr) {
            ::i2c_ll_slave_clear_stretch(_hw);
            enableMaskedInterrupts(masked & _baseline_intrs);
        }
    }
#endif
    if (&txn == _current) {
        _current = nullptr;
    }
    txn = Transaction{};
}

bool SlaveBus_espidf::isOpenOwnerLocked(bus::IAccessor* owner) const
{
    return owner != nullptr && _open != nullptr && _open_owner == owner;
}

void SlaveBus_espidf::notifyTaskFromISR(bool& task_woken)
{
    if (_task != nullptr) {
        ::BaseType_t higher_priority = pdFALSE;
        ::vTaskNotifyGiveFromISR(_task, &higher_priority);
        if (higher_priority == pdTRUE) {
            task_woken = true;
        }
    }
}

void SlaveBus_espidf::notifyConsumerFromISR(bool& task_woken)
{
    // Give the dedicated consumer semaphore (created on the first waitForActivity).
    // Binary, so repeated gives latch to one pending wake and serve() never busy-spins;
    // a give before the consumer parks is held and consumed by its next take.
    if (_consumer_sem != nullptr) {
        ::BaseType_t higher_priority = pdFALSE;
        ::xSemaphoreGiveFromISR(_consumer_sem, &higher_priority);
        if (higher_priority == pdTRUE) {
            task_woken = true;
        }
    }
}

void SlaveBus_espidf::notifyTaskFromTask()
{
    auto* task = _task;
    if (task != nullptr) {
        ::xTaskNotifyGive(task);
    }
}

result_t<bool> SlaveBus_espidf::waitForActivity(bus::IAccessor* owner, uint32_t timeout_ms)
{
    (void)owner;
    // Park on the dedicated consumer semaphore (not this task's direct notification, so
    // an app's own xTaskNotify never collides). Create it on first use, in task context;
    // only one consumer task drives serve(), so this single-writer init needs no lock
    // beyond publishing the handle under _mux for the ISR's read. A give that arrived
    // before we park is latched (the take returns immediately); the safety timeout
    // re-checks completion if a wake is ever missed.
    if (_consumer_sem == nullptr) {
        ::SemaphoreHandle_t sem = ::xSemaphoreCreateBinaryStatic(&_consumer_sem_buf);
        portENTER_CRITICAL_SAFE(&_mux);
        _consumer_sem = sem;
        portEXIT_CRITICAL_SAFE(&_mux);
    }
    // pdTRUE = woken by a consumer-semaphore give (an RX/TX/STOP ISR); pdFALSE = the
    // safety timeout elapsed with no notification. Report it so a custom serve loop can
    // tell an activity wake from a timeout (the caller still re-checks bus state).
    return ::xSemaphoreTake(_consumer_sem, impl_espidf_slave::ticks(timeout_ms)) == pdTRUE;
}

void SlaveBus_espidf::requestTaskLoop()
{
    for (;;) {
        ::TickType_t wait_ticks = portMAX_DELAY;
        portENTER_CRITICAL_SAFE(&_mux);
        const bool stop = _task_stop;
#if M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_SLAVE_LL_BE
        const bool queued = _queued_active;
#endif
        // Only arm the finite stretch-budget wait when an actual TX read-stretch hold
        // is in flight. _request_pending stays true from a read's address-match until
        // its STOP, but once the hold is released (fill-byte fallback or write(), so
        // _hold_kind returns to none) there is nothing left to time out -- gating on
        // _request_pending alone made this task busy-spin at max priority with
        // wait_ticks=0 until STOP, starving the serve() consumer (and, if STOP never
        // came because the bus was wedged, tripping the task watchdog). An RX_FULL
        // back-pressure hold is consumer-driven (read() lifts it), never this task, so
        // it must NOT arm the wait either. The v2 (non-LL) path keeps the prior gate.
        if (!stop && _request_pending && _config.tx_underrun == i2c::TxUnderrun::Stretch
#if M5HAL_ESPIDF_I2C_SLAVE_LL
            && (_hold_kind == HoldKind::address_read || _hold_kind == HoldKind::tx_empty)
#endif
        ) {
            wait_ticks = requestWaitTicksLocked(::xTaskGetTickCount());
        }
        portEXIT_CRITICAL_SAFE(&_mux);
        if (stop) {
            break;
        }

#if M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_SLAVE_LL_BE
        if (queued) {
            // Drain work that may already have been queued before this loop
            // observed queued mode. A notification arriving between this pass
            // and the take remains latched by FreeRTOS for the next pass.
            processQueuedWorker();
            (void)::ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
#endif

        (void)::ulTaskNotifyTake(pdTRUE, wait_ticks);

#if M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_SLAVE_LL_BE
        // beginOperation() can switch this task from the legacy wire-frame
        // window while it is already parked above. Re-check after every wake:
        // otherwise the first queued write notification could be consumed by
        // the legacy wait and the task would park again without staging it.
        portENTER_CRITICAL_SAFE(&_mux);
        const bool queued_after_wake = _queued_active;
        const bool stop_after_wake   = _task_stop;
        portEXIT_CRITICAL_SAFE(&_mux);
        if (stop_after_wake) continue;
        if (queued_after_wake) {
            processQueuedWorker();
            continue;
        }
#endif

#if M5HAL_ESPIDF_I2C_SLAVE_LL

        // LL path: a read stretch is held by the ISR. Compose the reply from the
        // open transaction's tx queue, then release the stretch.
        //
        // stretch-hold-until-write: under the `stretch` policy, if the app has
        // not yet queued reply bytes (open tx empty) and the stretch budget has
        // not expired, we keep the stretch HELD and loop back to wait -- the
        // master stays clock-stretched while a polling Stream accessor runs its
        // begin->read->write->end sequence. The hold ends when write() queues
        // data (it notifies this task) or the budget expires (fall back to a
        // single fill byte so the bus does not deadlock). Under the `fill`
        // policy we fill immediately as before (best-effort, no waiting).
        bool release_stretch          = false;
        ::i2c_dev_t* hw_release       = nullptr;
        uint32_t enable_after_release = 0;
        portENTER_CRITICAL_SAFE(&_mux);
        if (!_task_stop && (_hold_kind == HoldKind::address_read || _hold_kind == HoldKind::tx_empty) &&
            _hw != nullptr) {
            // _open == _current mirrors snapshotResponseLocked()'s stale-reply guard:
            // unread tx bytes on a lingering completed transaction must keep the hold
            // (waiting for serve() to open the wire's transaction and compose afresh),
            // not trip a premature snapshot whose empty _resp pays out a fill byte.
            const bool have_data = (_open != nullptr && _open == _current && _open->tx_read < _open->tx_size);
            const bool hold      = (_config.tx_underrun == i2c::TxUnderrun::Stretch) && !have_data &&
                              (requestWaitTicksLocked(::xTaskGetTickCount()) != 0);
            if (!hold) {
                // Data ready, fill policy, or budget expired: snapshot the reply
                // into the backend buffer, fill the FIFO from it (queued data, else
                // one fill byte), and release the stretch.
                snapshotResponseLocked();
                fillTxFromRespLocked();
                const uint32_t masked = clearHoldLocked();
                release_stretch       = true;
                hw_release            = _hw;  // snapshot under _mux; do not re-read _hw after unlock
                enable_after_release  = masked & _baseline_intrs;
            }
            // else: keep the stretch held; loop back and wait for write()/timeout.
        }
        portEXIT_CRITICAL_SAFE(&_mux);
        if (release_stretch) {
            ::i2c_ll_slave_clear_stretch(hw_release);
#if M5HAL_DETAIL_I2C_SLAVE_TX_WATERMARK_
            // Arm the proactive TX water-mark top-up (the responder task just started a
            // reply flowing); keep the FIFO topped up before it empties.
            ::i2c_ll_enable_intr_mask(hw_release, I2C_TXFIFO_WM_INT_ENA_M);
#endif
            if (enable_after_release != 0) {
                ::i2c_ll_enable_intr_mask(hw_release, enable_after_release);
            }
        }
#elif M5HAL_ESPIDF_I2C_SLAVE_LL_BE
        // BE has no clock stretch to hold, so there is nothing to release here:
        // the ISR fills/streams TX directly (see handleIsr()'s TX water-mark
        // and STOP handling). This task exists only to satisfy the shared
        // wake/stop machinery (waitForActivity, teardown); its loop body is a
        // deliberate no-op for this flavor.
#else
        for (;;) {
            uint8_t local[kTxCapacity] = {};
            size_t len                 = 0;
            bool generated_fill        = false;
            if (!collectPendingWrite(local, len, generated_fill)) {
                break;
            }
            if (len == 0) {
                continue;
            }
            uint32_t written = 0;
            auto* handle     = _handle;
            if (handle != nullptr) {
                (void)::i2c_slave_write(handle, local, static_cast<uint32_t>(len), &written, 0);
            }
            if (written == 0) {
                break;
            }
            if (!generated_fill && written > 0) {
                markWriteCommitted(static_cast<size_t>(written));
            }
        }
#endif
    }

    portENTER_CRITICAL_SAFE(&_mux);
    _task_running = false;
    portEXIT_CRITICAL_SAFE(&_mux);
}

::TickType_t SlaveBus_espidf::requestWaitTicksLocked(::TickType_t now_tick) const
{
    const ::TickType_t budget  = impl_espidf_slave::ticks(_config.stretch_timeout_ms);
    const ::TickType_t elapsed = now_tick - _request_tick;
    return (elapsed >= budget) ? 0 : (budget - elapsed);
}

bool SlaveBus_espidf::collectPendingWrite(uint8_t* dst, size_t& len, bool& generated_fill)
{
    bool should_write = false;
    portENTER_CRITICAL_SAFE(&_mux);
    if (_task_stop || !_request_pending) {
        portEXIT_CRITICAL_SAFE(&_mux);
        return false;
    }

    auto* txn = _open;
    if (txn != nullptr && txn == _current && txn->opened && txn->tx_read < txn->tx_size) {
        len                 = std::min(txn->tx_size - txn->tx_read, kTxCapacity);
        _pending_commit_len = len;
        _pending_commit_txn = txn;
        _pending_commit_seq = txn->seq;
        const size_t start  = txn->tx_read & (kTxCapacity - 1);
        const size_t first  = std::min(len, kTxCapacity - start);
        ::memcpy(dst, txn->tx + start, first);
        if (first < len) {
            ::memcpy(dst + first, txn->tx, len - first);
        }
        should_write = true;
    } else if (_config.tx_underrun == i2c::TxUnderrun::Fill || requestWaitTicksLocked(::xTaskGetTickCount()) == 0) {
        dst[0]              = _config.tx_fill_byte;
        len                 = 1;
        _request_pending    = false;
        _pending_commit_len = 0;
        _pending_commit_txn = nullptr;
        _pending_commit_seq = 0;
        generated_fill      = true;
        should_write        = true;
    }
    portEXIT_CRITICAL_SAFE(&_mux);
    return should_write;
}

void SlaveBus_espidf::markWriteCommitted(size_t len)
{
    portENTER_CRITICAL_SAFE(&_mux);
    auto* txn = _pending_commit_txn;
    if (txn != nullptr && txn->in_use && txn->opened && txn->seq == _pending_commit_seq) {
        const size_t remaining = (txn->tx_read < txn->tx_size) ? (txn->tx_size - txn->tx_read) : 0;
        txn->tx_read += std::min(len, std::min(remaining, _pending_commit_len));
        if (txn == _current && txn->tx_read >= txn->tx_size) {
            _request_pending = false;
        }
    }
    _pending_commit_len = 0;
    _pending_commit_txn = nullptr;
    _pending_commit_seq = 0;
    portEXIT_CRITICAL_SAFE(&_mux);
}

size_t SlaveBus_espidf::readableBytesOf(const Transaction& txn)
{
    return (txn.rx_read < txn.rx_size) ? (txn.rx_size - txn.rx_read) : 0;
}

}  // namespace m5::hal::v2::i2c

#endif

#endif
