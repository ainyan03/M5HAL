// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_SLAVE_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_SLAVE_INL

#include "slave.hpp"

#if defined(ESP_PLATFORM) && (M5HAL_ESPIDF_I2C_SLAVE_LL || M5HAL_ESPIDF_I2C_HAS_SLAVE_V2)

#include <algorithm>
#include <esp_err.h>
#include <soc/soc_caps.h>
#include <string.h>

#include "../../../freertos/hal/runtime/time.hpp"
#include <freertos/task.h>

#if M5HAL_ESPIDF_I2C_SLAVE_LL
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <hal/i2c_ll.h>
#include <soc/i2c_periph.h>
#if __has_include(<esp_private/periph_ctrl.h>)
#include <esp_private/periph_ctrl.h>
#else
#include <driver/periph_ctrl.h>
#endif

#if defined(M5HAL_I2C_SLAVE_GPIO_MARKERS)
// Non-perturbing GPIO markers for logic-analyzer correlation (IRAM-safe gpio_ll).
// Off by default; build the slave with -DM5HAL_I2C_SLAVE_GPIO_MARKERS and wire the
// pins to the analyzer alongside SCL/SDA to see, per 32-byte "息継ぎ" gap, WHO is
// pausing the bus:
//   TXFILL  (pin 6)  HIGH while the slave loads the TX FIFO  -> a read-side gap that
//                    lines up with this pulse is the slave refilling (TX_EMPTY).
//   RXDRAIN (pin 7)  HIGH while the slave reads the RX FIFO   -> a write-side gap on
//                    this pulse is the slave draining; a gap with NO pulse is the
//                    master pausing (its own HW FIFO refill), not the slave.
//   STRETCH (pin 13) HIGH while the stretch-cause ISR services a hold/refill.
#ifndef M5HAL_I2C_SLAVE_MARK_TXFILL
#define M5HAL_I2C_SLAVE_MARK_TXFILL 6
#endif
#ifndef M5HAL_I2C_SLAVE_MARK_RXDRAIN
#define M5HAL_I2C_SLAVE_MARK_RXDRAIN 7
#endif
#ifndef M5HAL_I2C_SLAVE_MARK_STRETCH
#define M5HAL_I2C_SLAVE_MARK_STRETCH 13
#endif
#include <hal/gpio_ll.h>
#define M5HAL_MARK_HI(pin) ::gpio_ll_set_level(&GPIO, (::gpio_num_t)(pin), 1)
#define M5HAL_MARK_LO(pin) ::gpio_ll_set_level(&GPIO, (::gpio_num_t)(pin), 0)
#else
#define M5HAL_MARK_HI(pin) ((void)0)
#define M5HAL_MARK_LO(pin) ((void)0)
#endif
#endif

// The proactive TX water-mark top-up (refills the TX FIFO at FIFO/2 before it empties,
// smoothing the read-side 32-byte "息継ぎ" that the reactive TX_EMPTY stretch otherwise
// causes) can be turned off for A/B diagnosis with -DM5HAL_I2C_SLAVE_NO_TX_WATERMARK.
// Default = on. With it off the TX FIFO is only refilled reactively on TX_EMPTY.
#if defined(M5HAL_I2C_SLAVE_NO_TX_WATERMARK)
#define M5HAL_I2C_SLAVE_TX_WM 0
#else
#define M5HAL_I2C_SLAVE_TX_WM 1
#endif

namespace m5::hal::v2::i2c {

namespace {
namespace impl_espidf_slave {

#if !M5HAL_ESPIDF_I2C_SLAVE_LL
error::error_t mapEspErr(::esp_err_t err)
{
    switch (err) {
        case ESP_OK:
            return error::error_t::OK;
        case ESP_ERR_INVALID_ARG:
        case ESP_ERR_INVALID_STATE:
            return error::error_t::INVALID_ARGUMENT;
        case ESP_ERR_TIMEOUT:
            return error::error_t::TIMEOUT_ERROR;
        case ESP_ERR_NO_MEM:
            return error::error_t::OUT_OF_RESOURCE;
        case ESP_ERR_NOT_FOUND:
            return error::error_t::I2C_NO_ACK;
        case ESP_ERR_NOT_SUPPORTED:
            return error::error_t::UNSUPPORTED;
        default:
            return error::error_t::I2C_BUS_ERROR;
    }
}
#endif

::TickType_t ticks(uint32_t timeout_ms)
{
    return ::m5::hal::v2::detail::timeoutMsToTicks(timeout_ms);
}

}  // namespace impl_espidf_slave
}  // namespace

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
    if (cfg.pin_scl < 0 || cfg.pin_sda < 0 || cfg.address_is_10bit || cfg.address > 0x7Fu) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if !defined(SOC_I2C_SUPPORT_SLAVE) || !SOC_I2C_SUPPORT_SLAVE
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
#endif

    if (_hw != nullptr || _task != nullptr || _intr != nullptr) {
        (void)release();
    }
    _config  = cfg;
    _pin_scl = cfg.pin_scl;
    _pin_sda = cfg.pin_sda;

    {
        portENTER_CRITICAL_SAFE(&_mux);
        resetStateLocked();
        _hold_kind      = HoldKind::none;
        _masked_intrs   = 0;
        _baseline_intrs = 0;
        _resp_len       = 0;
        _resp_pos       = 0;
        _task_stop      = false;
        _task_running   = true;
        portEXIT_CRITICAL_SAFE(&_mux);
    }
    if (::xTaskCreate(&SlaveBus_espidf::taskThunk, "m5hal_i2c_slave", 3072, this, configMAX_PRIORITIES - 1, &_task) !=
        pdPASS) {
        portENTER_CRITICAL_SAFE(&_mux);
        _task_running = false;
        portEXIT_CRITICAL_SAFE(&_mux);
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    const ::i2c_port_t port = I2C_NUM_0;
    ::i2c_dev_t* const hw   = I2C_LL_GET_HW(port);
    _hw                     = hw;

    // periph_module_enable() は RCC リファクタで新世代SoC(C61/C5/P4等)では非機能
    // (deprecated "not functional")。ESP-IDF 公式 I2C ドライバ(esp_driver_i2c/
    // i2c_common.c)と同じ RCC API で I2C0 のバスクロック有効化+レジスタリセットを
    // 行う(v5.5/v6.0 両対応・全SoC portable)。
    // SOC_RCC_IS_INDEPENDENT が真のSoC(c5/c6/c61/h2)では i2c_ll_* を直接呼べる
    // (RCC_ATOMIC 不要・空展開相当)。RCC 共有のSoC(s2/s3/c3/p4)では
    // i2c_ll_enable_bus_clock が __DECLARE_RCC_ATOMIC_ENV を要求するため
    // PERIPH_RCC_ATOMIC() ブロック内で呼ぶ。
    // 注意: ここでは敢えて `::` を付けない。RCC 共有SoCの i2c_ll.h では
    // i2c_ll_enable_bus_clock / i2c_ll_reset_register が関数形式マクロ
    // (`do {(void)__DECLARE_RCC_ATOMIC_ENV; ...} while(0)` 展開)であり、
    // `::` を前置すると `:: do{...}` となり "expected id-expression" で壊れる。
    // 無修飾でもグローバルの実関数/マクロに解決される(衝突する近傍名は無い)。
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
    ::i2c_ll_slave_enable_auto_start(hw, true);

    ::i2c_ll_set_slave_addr(hw, cfg.address, false);
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
    ::i2c_ll_enable_fifo_mode(hw, true);
    // REQUIRED: do NOT store the matched address byte in the RX FIFO. With its
    // power-on default the slave address lands in RX and is mistaken for data.
    hw->fifo_conf.fifo_addr_cfg_en = 0;

    // Enable SCL stretch with a large protect count covering worst-case ISR/task
    // latency, then clear any stretch left from a prior session.
    ::i2c_ll_slave_enable_scl_stretch(hw, true);
    ::i2c_ll_slave_set_stretch_protect_num(hw, 0x3ff);
    ::i2c_ll_slave_clear_stretch(hw);

    if (::esp_intr_alloc(i2c_periph_signal[port].irq, M5HAL_I2C_SLAVE_ISR_INTR_FLAGS, &SlaveBus_espidf::isrThunk, this,
                         &_intr) != ESP_OK) {
        (void)release();
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    _baseline_intrs = I2C_RXFIFO_WM_INT_ENA_M | I2C_TRANS_COMPLETE_INT_ENA_M | I2C_SLAVE_STRETCH_INT_ENA_M;
    ::i2c_ll_enable_intr_mask(hw, _baseline_intrs);
    ::i2c_ll_update(hw);
#if defined(M5HAL_I2C_SLAVE_GPIO_MARKERS)
    {
        const int mpins[3] = {M5HAL_I2C_SLAVE_MARK_TXFILL, M5HAL_I2C_SLAVE_MARK_RXDRAIN, M5HAL_I2C_SLAVE_MARK_STRETCH};
        for (int i = 0; i < 3; ++i) {
            ::gpio_set_direction((::gpio_num_t)mpins[i], GPIO_MODE_OUTPUT);
            ::gpio_set_level((::gpio_num_t)mpins[i], 0);
        }
    }
#endif
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

result_t<void> SlaveBus_espidf::release(void)
{
    if (_intr != nullptr) {
        if (_hw != nullptr) {
            ::i2c_ll_disable_intr_mask(_hw, I2C_LL_INTR_MASK);
            ::i2c_ll_clear_intr_mask(_hw, I2C_LL_INTR_MASK);
        }
        (void)::esp_intr_free(_intr);
        _intr = nullptr;
    }

    if (_task != nullptr) {
        portENTER_CRITICAL_SAFE(&_mux);
        _task_stop = true;
        portEXIT_CRITICAL_SAFE(&_mux);
        notifyTaskFromTask();
        for (uint32_t i = 0; i < 100; ++i) {
            bool running = false;
            portENTER_CRITICAL_SAFE(&_mux);
            running = _task_running;
            portEXIT_CRITICAL_SAFE(&_mux);
            if (!running) {
                break;
            }
            ::vTaskDelay(pdMS_TO_TICKS(1));
        }
        _task = nullptr;
    }

    _hw = nullptr;
    restorePins();

    portENTER_CRITICAL_SAFE(&_mux);
    resetStateLocked();
    _hold_kind      = HoldKind::none;
    _masked_intrs   = 0;
    _baseline_intrs = 0;
    _task_stop      = false;
    _task_running   = false;
    portEXIT_CRITICAL_SAFE(&_mux);
    return {};
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
                count -= c;
            }
            ++_rx_overflow_count;
            return false;
        }
    }
    uint8_t buf[SOC_I2C_FIFO_LEN];
    M5HAL_MARK_HI(M5HAL_I2C_SLAVE_MARK_RXDRAIN);
    while (count) {
        // rx[] is a power-of-two ring: the cap bounds the UNREAD backlog
        // (rx_size - rx_read), not the per-transaction total. While read() drains
        // (freeing space), this ISR path can store far more than kRxCapacity bytes
        // across one transaction.
        const size_t freespace = kRxCapacity - (_current->rx_size - _current->rx_read);
        if (freespace == 0) {
            if (can_hold) {
                // Ring full mid-transaction: leave the remaining `count` bytes in the
                // HW FIFO so the master stays held under the RX_FULL stretch (the
                // caller masks the hold). read() resumes us once it frees ring space.
                // Zero bytes are dropped -- this is the write-direction back-pressure.
                M5HAL_MARK_LO(M5HAL_I2C_SLAVE_MARK_RXDRAIN);
                return true;
            }
            // STOP / no later lift: drain and drop the tail that no longer fits
            // (only reached if the consumer never caught up; surfaced via rxOverflow).
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
            _current->rx[(_current->rx_size++) & (kRxCapacity - 1)] = buf[i];
        }
        count -= c;
    }
    M5HAL_MARK_LO(M5HAL_I2C_SLAVE_MARK_RXDRAIN);
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
    if (txn != nullptr) {
        while (txn->tx_read < txn->tx_size && _resp_len < kTxCapacity) {
            _resp[_resp_len++] = txn->tx[(txn->tx_read++) & (kTxCapacity - 1)];
        }
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
    M5HAL_MARK_HI(M5HAL_I2C_SLAVE_MARK_TXFILL);
    ::i2c_ll_write_txfifo(_hw, buf, static_cast<uint8_t>(n));
    M5HAL_MARK_LO(M5HAL_I2C_SLAVE_MARK_TXFILL);
}

void SlaveBus_espidf::enterTxHoldFromIsrLocked(bool& task_woken, bool address_read)
{
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
    // removes the 32-byte read-side "息継ぎ" (each FIFO-empty -> stretch -> refill). If
    // no reply bytes are queued yet, stop the water mark re-firing (it asserts while
    // the FIFO stays below the threshold); the FIFO then empties into the TX_EMPTY
    // stretch, which holds until the app streams more. Enabled on read-stretch release
    // and disabled at STOP / when the reply is exhausted (so it never fires on writes).
#if M5HAL_I2C_SLAVE_TX_WM
    if ((ints & I2C_TXFIFO_WM_INT_ENA_M) && is_read) {
        snapshotResponseLocked();
        if (_resp_pos < _resp_len) {
            fillTxFromRespLocked();
        } else {
            ::i2c_ll_disable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
        }
    }
#endif

    // Stretch: the HW is holding SCL low waiting for us.
    if (ints & I2C_SLAVE_STRETCH_INT_ENA_M) {
        M5HAL_MARK_HI(M5HAL_I2C_SLAVE_MARK_STRETCH);
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
#if M5HAL_I2C_SLAVE_TX_WM
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
        M5HAL_MARK_LO(M5HAL_I2C_SLAVE_MARK_STRETCH);
    }

    // STOP: mark the transaction complete and discard the TX bytes the master did
    // not read (the window's Tx auto-vanish).
    if (ints & I2C_TRANS_COMPLETE_INT_ENA_M) {
        if (rx) {
            // Transaction ending: there is no later read() to lift a hold, so drain
            // all and drop any tail that no longer fits (back-pressure normally keeps
            // the ring from being full here; a drop means the consumer never caught up).
            drainRxLocked(rx, false);
            rx = 0;
        }
        if (is_read) {
            ::i2c_ll_disable_intr_mask(hw, I2C_TXFIFO_WM_INT_ENA_M);
            ::i2c_ll_txfifo_rst(hw);
        }
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

    // Wake the serve() consumer on this activity (RX drained, hold entered, reply
    // room, or STOP) so it drains/fills without waiting out a poll delay -- the key
    // to keeping the RX ring from being full at the master's STOP at high speed.
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

    if (_handle != nullptr || _task != nullptr) {
        (void)release();
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
    native_cfg.i2c_port                     = I2C_NUM_0;
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
        (void)release();
        return m5::stl::make_unexpected(mapped);
    }

    ::i2c_slave_event_callbacks_t callbacks = {};
    callbacks.on_receive                    = &SlaveBus_espidf::onReceive;
    callbacks.on_request                    = &SlaveBus_espidf::onRequest;
    mapped = impl_espidf_slave::mapEspErr(::i2c_slave_register_event_callbacks(_handle, &callbacks, this));
    if (error::isError(mapped)) {
        (void)release();
        return m5::stl::make_unexpected(mapped);
    }
    return {};
}

result_t<void> SlaveBus_espidf::release(void)
{
    if (_task != nullptr) {
        portENTER_CRITICAL_SAFE(&_mux);
        _task_stop = true;
        portEXIT_CRITICAL_SAFE(&_mux);
        notifyTaskFromTask();
        for (uint32_t i = 0; i < 100; ++i) {
            bool running = false;
            portENTER_CRITICAL_SAFE(&_mux);
            running = _task_running;
            portEXIT_CRITICAL_SAFE(&_mux);
            if (!running) {
                break;
            }
            ::vTaskDelay(pdMS_TO_TICKS(1));
        }
        _task = nullptr;
    }

    if (_handle != nullptr) {
        auto mapped = impl_espidf_slave::mapEspErr(::i2c_del_slave_device(_handle));
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        _handle = nullptr;
    }

    portENTER_CRITICAL_SAFE(&_mux);
    resetStateLocked();
    _task_stop    = false;
    _task_running = false;
    portEXIT_CRITICAL_SAFE(&_mux);
    return {};
}

bool SlaveBus_espidf::onReceive(::i2c_slave_dev_handle_t handle, const ::i2c_slave_rx_done_event_data_t* evt_data,
                                void* user_data)
{
    (void)handle;
    auto* self = static_cast<SlaveBus_espidf*>(user_data);
    if (self == nullptr || evt_data == nullptr) {
        return false;
    }
    portENTER_CRITICAL_ISR(&self->_mux);
    auto* txn = self->allocateTransactionLocked();
    if (txn != nullptr) {
        // Non-LL callback path: the driver delivers the WHOLE transaction post-STOP
        // in one buffer, so there is no mid-transaction draining to make room. Unlike
        // the streaming drainRxLocked() ring, kRxCapacity is a hard per-transaction
        // cap here; a longer transaction is truncated (surfaced via rxOverflowCount).
        // The bytes land at rx[0..copy_len) with rx_read = 0, which is the ring's
        // natural start, so the wrap-aware read() handles this path unchanged.
        const size_t copy_len = std::min(static_cast<size_t>(evt_data->length), kRxCapacity);
        if (copy_len > 0 && evt_data->buffer != nullptr) {
            ::memcpy(txn->rx, evt_data->buffer, copy_len);
        }
        txn->rx_size   = copy_len;
        txn->rx_read   = 0;
        txn->complete  = true;
        self->_current = txn;
        if (copy_len < static_cast<size_t>(evt_data->length)) {
            ++self->_rx_overflow_count;
        }
    }
    portEXIT_CRITICAL_ISR(&self->_mux);
    return false;
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
    portEXIT_CRITICAL_ISR(&self->_mux);
    return task_woken;
}

#endif  // M5HAL_ESPIDF_I2C_SLAVE_LL

// ===========================================================================
// Shared transaction-window state machine (HW independent)
// ===========================================================================

result_t<void> SlaveBus_espidf::beginTransaction(bus::IAccessor* owner, uint32_t timeout_ms)
{
    (void)timeout_ms;
    if (owner == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    bool should_notify = false;
    portENTER_CRITICAL_SAFE(&_mux);
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

result_t<void> SlaveBus_espidf::endTransaction(bus::IAccessor* owner)
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
        const size_t start = _open->rx_read & (kRxCapacity - 1);
        const size_t first = std::min(take, kRxCapacity - start);
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
        if ((_hold_kind == HoldKind::address_read || _hold_kind == HoldKind::tx_empty) && _hw != nullptr) {
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
#if M5HAL_I2C_SLAVE_TX_WM
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

result_t<bool> SlaveBus_espidf::transactionComplete(bus::IAccessor* owner)
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
    _pending_fill       = false;
    _pending_commit_len = 0;
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

        (void)::ulTaskNotifyTake(pdTRUE, wait_ticks);

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
            const bool have_data = (_open != nullptr && _open->tx_read < _open->tx_size);
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
#if M5HAL_I2C_SLAVE_TX_WM
            // Arm the proactive TX water-mark top-up (the responder task just started a
            // reply flowing); keep the FIFO topped up before it empties.
            ::i2c_ll_enable_intr_mask(hw_release, I2C_TXFIFO_WM_INT_ENA_M);
#endif
            if (enable_after_release != 0) {
                ::i2c_ll_enable_intr_mask(hw_release, enable_after_release);
            }
        }
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

    auto* txn = _current;
    if (txn != nullptr && txn->opened && txn->tx_read < txn->tx_size) {
        len                 = std::min(txn->tx_size - txn->tx_read, kTxCapacity);
        _pending_commit_len = len;
        _pending_fill       = false;
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
        _pending_fill       = true;
        generated_fill      = true;
        should_write        = true;
    }
    portEXIT_CRITICAL_SAFE(&_mux);
    return should_write;
}

void SlaveBus_espidf::markWriteCommitted(size_t len)
{
    portENTER_CRITICAL_SAFE(&_mux);
    if (_current != nullptr && _current->opened) {
        const size_t remaining = (_current->tx_read < _current->tx_size) ? (_current->tx_size - _current->tx_read) : 0;
        _current->tx_read += std::min(len, remaining);
        if (_current->tx_read >= _current->tx_size) {
            _request_pending = false;
        }
    }
    _pending_commit_len = 0;
    _pending_fill       = false;
    portEXIT_CRITICAL_SAFE(&_mux);
}

size_t SlaveBus_espidf::readableBytesOf(const Transaction& txn)
{
    return (txn.rx_read < txn.rx_size) ? (txn.rx_size - txn.rx_read) : 0;
}

}  // namespace m5::hal::v2::i2c

#endif

#endif
