// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_INL

#include "i2s.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#include "../../detail/esp_err_map.hpp"

#include <driver/gpio.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>

#include "../../../freertos/hal/runtime/time.hpp"

#include <cstdlib>

namespace m5::hal::v2::i2s {

namespace {
namespace impl_espidf {

error::error_t mapEspErr(::esp_err_t err)
{
    return ::m5::variants::frameworks::espidf::detail::mapEspErrCommon(err, error::error_t::IO_ERROR);
}

// Choose DMA descriptor count and frame count so that
//   dma_desc_num * dma_frame_num * frame_bytes  ≈  target_bytes.
// Constraints: dma_desc_num in [2..128], dma_frame_num in [16..4092].
// We pin dma_frame_num = 120 (480 B / 3.75 ms at 16-bit stereo) and vary
// dma_desc_num to approach the target. Finer descriptors give the driver
// more top-up granularity (a partially filled tail descriptor is smaller)
// and make the on_sent in-flight tracking proportionally smoother; the
// 128-descriptor cap bounds how fine the split can go for large buffers.
void calcDmaParams(size_t target_bytes, size_t frame_bytes, uint32_t& out_desc_num, uint32_t& out_frame_num,
                   size_t& out_capacity)
{
    constexpr uint32_t kFrameNum   = 120;
    constexpr uint32_t kMinDescNum = 2;
    constexpr uint32_t kMaxDescNum = 128;
    const size_t bytes_per_desc    = kFrameNum * frame_bytes;
    uint32_t desc_num              = static_cast<uint32_t>((target_bytes + bytes_per_desc - 1) / bytes_per_desc);
    if (desc_num < kMinDescNum) {
        desc_num = kMinDescNum;
    }
    if (desc_num > kMaxDescNum) {
        desc_num = kMaxDescNum;
    }
    out_desc_num  = desc_num;
    out_frame_num = kFrameNum;
    out_capacity  = static_cast<size_t>(desc_num) * kFrameNum * frame_bytes;
}

bool sameAccessConfig(const i2s::AccessConfig& a, const i2s::AccessConfig& b)
{
    return a.sample_rate_hz == b.sample_rate_hz && a.bits_per_sample == b.bits_per_sample && a.channels == b.channels;
}

::TickType_t toTicks(uint32_t timeout_ms)
{
    return ::m5::hal::v2::detail::timeoutMsToTicks(timeout_ms);
}

// Swap the two 16-bit halves of every 32-bit word while copying dst <- src
// (memcpy-like). dst == src does the swap in place; otherwise dst and src must
// not overlap. Both must be >= 2-byte aligned; len need not be a multiple of 4 —
// a trailing 2-byte tail (len % 4 == 2) is copied verbatim (no pair to swap).
// This undoes the I2S HW v1 L/R-slot transpose (see Bus_espidf::_swap16).
//
// The shape — two INDEPENDENT 16-bit loads, then 16-bit stores to the SWAPPED
// offsets (the swap is the addressing, no ALU op in the load->store chain) — is
// deliberate for the Xtensa LX6/LX7 of HW v1 (classic ESP32 / ESP32-S2): an
// l32i + funnel-shift would stall on the 2-cycle load-use latency (the shift
// waits on the load), the two l16ui do not. At -O2/-Os/-O3 GCC compiles this C
// to exactly that (a zero-overhead `loop` + l16ui x2 + s16i x2 — verified by
// disassembly, identical to hand asm), so it stays readable C and portable
// (RISC-V SoCs compile it too, though _swap16 is never set there — one impl, no
// #ifdef). The function attribute pins -O3 LOCALLY: noinline + noclone keep it a
// single standalone copy so optimize() reliably applies even when the caller or
// the whole file is built at -Os / -O0 (i.e. the codegen does not depend on the
// downstream optimization level).
#if defined(__GNUC__) && !defined(__clang__)
__attribute__((noinline, noclone, optimize("-O3")))
#endif
void swapHalfwords(void* dst, const void* src, size_t len)
{
    uint16_t* d       = static_cast<uint16_t*>(dst);
    const uint16_t* s = static_cast<const uint16_t*>(src);
    size_t words      = len >> 2;
    while (words-- != 0u) {
        const uint16_t a = s[0];  // load both halves before storing -> in-place safe
        const uint16_t b = s[1];
        d[0]             = b;  // store the two halves swapped
        d[1]             = a;
        d += 2;
        s += 2;
    }
    if ((len & 2u) != 0u) {
        *d = *s;  // odd trailing halfword: nothing to pair with
    }
}

}  // namespace impl_espidf
}  // namespace

// ---------------------------------------------------------------------------
// static callback
// ---------------------------------------------------------------------------
bool Bus_espidf::onSentCallback(::i2s_chan_handle_t /*handle*/, ::i2s_event_data_t* event, void* user_ctx)
{
    if (event == nullptr || user_ctx == nullptr) {
        return false;
    }
    auto* self = static_cast<Bus_espidf*>(user_ctx);
    // Clamped subtraction (CAS loop, ISR-safe): silence buffers played during
    // an underrun also land here, but must not push in-flight below zero.
    size_t cur = self->_dma_in_flight.load(std::memory_order_relaxed);
    while (cur != 0) {
        const size_t dec  = (event->size < cur) ? event->size : cur;
        const size_t next = cur - dec;
        if (self->_dma_in_flight.compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
            break;
        }
    }
    return false;
}

bool Bus_espidf::onRecvCallback(::i2s_chan_handle_t /*handle*/, ::i2s_event_data_t* event, void* user_ctx)
{
    if (event == nullptr || user_ctx == nullptr) {
        return false;
    }
    auto* self = static_cast<Bus_espidf*>(user_ctx);
    // Clamped addition (CAS loop, ISR-safe): the on_recv mirror of on_sent.
    // Each captured DMA block bumps the available count, capped at the RX DMA
    // capacity so a reader that falls behind (the driver overwrites the oldest
    // descriptor) cannot make readableBytes() over-report what the DMA holds.
    const size_t cap = self->_dma_rx_capacity;
    size_t cur       = self->_dma_rx_available.load(std::memory_order_relaxed);
    for (;;) {
        size_t next = cur + event->size;
        if (cap != 0 && next > cap) {
            next = cap;
        }
        if (next == cur) {
            break;
        }
        if (self->_dma_rx_available.compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
            break;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
Bus_espidf::~Bus_espidf()
{
    (void)teardownBackend();
}

result_t<void> Bus_espidf::init(const IBusConfig& config)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    // Standard I2S always needs both clocks and at least one data direction.
    // Reject incomplete wiring here so init/acquire cannot succeed only to
    // fail later when the first I/O lazily creates the channel.
    if (config.pin_bclk < 0 || config.pin_ws < 0 || (config.pin_dout < 0 && config.pin_din < 0)) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    auto reset = resetForInitialization();
    if (!reset.has_value()) {
        return reset;
    }
    _config          = config;
    _configured      = false;
    _channel_enabled = false;
    _dma_in_flight.store(0, std::memory_order_relaxed);
    _dma_rx_available.store(0, std::memory_order_relaxed);
    _dma_capacity    = 0;
    _dma_rx_capacity = 0;
    _dma_tx_frames   = 0;
    _dma_rx_frames   = 0;
    return markInitializationSucceeded(false);
}

// ---------------------------------------------------------------------------
// close
// ---------------------------------------------------------------------------
bus::CloseOutcome Bus_espidf::closeBackend(void)
{
    return teardownBackend();
}

bus::CloseOutcome Bus_espidf::teardownBackend(void)
{
    _active_tx_owner      = nullptr;
    _active_rx_owner      = nullptr;
    _operation_configured = false;
    if (_tx_handle == nullptr && _rx_handle == nullptr) {
        if (_controller >= 0) {
            detail_espidf_i2s_controller::release(_controller);
            _controller = -1;
        }
        return bus::CloseOutcome::success();
    }
    // Disable each non-null handle WITHOUT consulting _channel_enabled: in a
    // partial-enable failure (e.g. TX enabled, then RX enable failed)
    // _channel_enabled is still false, yet the TX channel is enabled and
    // i2s_del_channel rejects an enabled channel. Disabling unconditionally is
    // safe — i2s_channel_disable on an already-disabled channel returns
    // ESP_ERR_INVALID_STATE, which is the one harmless error here.
    error::error_t first_error = error::error_t::OK;
    if (_tx_handle != nullptr) {
        const esp_err_t disabled = ::i2s_channel_disable(_tx_handle);
        if (disabled != ESP_OK && disabled != ESP_ERR_INVALID_STATE) {
            first_error = impl_espidf::mapEspErr(disabled);
        } else {
            const esp_err_t deleted = ::i2s_del_channel(_tx_handle);
            if (deleted == ESP_OK) {
                _tx_handle = nullptr;
            } else {
                first_error = impl_espidf::mapEspErr(deleted);
            }
        }
    }
    if (_rx_handle != nullptr) {
        const esp_err_t disabled = ::i2s_channel_disable(_rx_handle);
        if (disabled != ESP_OK && disabled != ESP_ERR_INVALID_STATE) {
            if (!error::isError(first_error)) {
                first_error = impl_espidf::mapEspErr(disabled);
            }
        } else {
            const esp_err_t deleted = ::i2s_del_channel(_rx_handle);
            if (deleted == ESP_OK) {
                _rx_handle = nullptr;
            } else if (!error::isError(first_error)) {
                first_error = impl_espidf::mapEspErr(deleted);
            }
        }
    }
    if (_tx_handle != nullptr || _rx_handle != nullptr) {
        return bus::CloseOutcome::partialOrUnknown(first_error);
    }
    // Unbind the pins ourselves: unlike spi_bus_free (gpio_reset_pin) and the
    // I2C deinit (gpio_output_disable), i2s_del_channel only revokes the
    // esp_gpio_reserve bookkeeping — the GPIO matrix routing and the
    // peripheral-controlled output enable survive the delete, so every output
    // pin keeps actively driving its frozen level (measured on ESP32 and
    // ESP32-S3 with IDF 5.5: a deleted channel's BCLK/WS/DOUT ignores external
    // pulls until something re-takes the pin). gpio_reset_pin returns them to
    // the chip-default high-Z-with-pullup state, matching what the other bus
    // kinds' vendor drivers leave behind. _config still holds the pins this
    // channel was created with (re-init overwrites it only after this runs).
    const int pins[] = {_config.pin_bclk, _config.pin_ws, _config.pin_dout, _config.pin_din, _config.pin_mclk};
    for (int pin : pins) {
        if (pin >= 0) {
            ::gpio_reset_pin(static_cast<gpio_num_t>(pin));
        }
    }
    _channel_enabled = false;
    _configured      = false;
    _dma_in_flight.store(0, std::memory_order_relaxed);
    _dma_rx_available.store(0, std::memory_order_relaxed);
    _dma_capacity    = 0;
    _dma_rx_capacity = 0;
    _dma_tx_frames   = 0;
    _dma_rx_frames   = 0;
    detail_espidf_i2s_controller::release(_controller);
    _controller = -1;
    return bus::CloseOutcome::success();
}

result_t<void> Bus_espidf::resetForInitialization(void)
{
    const auto outcome = teardownBackend();
    if (outcome.disposition == bus::CloseDisposition::Success) {
        return {};
    }
    quarantineLifecycleAfterPartialTeardown();
    return m5::stl::make_unexpected(outcome.error_code);
}

result_t<void> Bus_espidf::failAfterSetup(error::error_t cause)
{
    auto reset = resetForInitialization();
    if (!reset.has_value()) {
        return reset;
    }
    return m5::stl::make_unexpected(cause);
}

result_t<void> Bus_espidf::beginOperationBackend(bus::OperationContext<i2s::AccessConfig>& context)
{
    auto* accessor = operationOwner(context);
    if (accessor == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    const bool tx = context.runtime.mode == bus::OperationMode::Tx;
    const bool rx = context.runtime.mode == bus::OperationMode::Rx;
    if ((!tx && !rx) || (tx && _config.pin_dout < 0) || (rx && _config.pin_din < 0)) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    auto locked = _operation_mutex.lock(bus::remainingTimeout(context.runtime, runtime::millis()));
    if (!locked.has_value()) {
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock unlocker{_operation_mutex};

    bus::IAccessor* opposite = (tx ? _active_rx_owner : _active_tx_owner).load(std::memory_order_acquire);
    if (opposite != nullptr && _operation_configured &&
        !impl_espidf::sameAccessConfig(_active_operation_cfg, context.config)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto ensured = ensureChannel(context.config);
    if (!ensured.has_value()) {
        return ensured;
    }
    if (tx) {
        _active_tx_owner.store(accessor, std::memory_order_release);
    } else {
        _active_rx_owner.store(accessor, std::memory_order_release);
    }
    _active_operation_cfg = context.config;
    _operation_configured = true;
    return {};
}

result_t<void> Bus_espidf::endOperationBackend(bus::OperationContext<i2s::AccessConfig>& context)
{
    auto* accessor = operationOwner(context);
    auto& active   = context.runtime.mode == bus::OperationMode::Tx ? _active_tx_owner : _active_rx_owner;
    auto locked    = _operation_mutex.lock(bus::remainingTimeout(context.runtime, runtime::millis()));
    if (!locked.has_value()) {
        bus::IAccessor* expected = accessor;
        (void)active.compare_exchange_strong(expected, nullptr, std::memory_order_acq_rel);
        return m5::stl::make_unexpected(locked.error());
    }
    runtime::ScopedUnlock unlocker{_operation_mutex};

    if (active.load(std::memory_order_acquire) != accessor) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    active.store(nullptr, std::memory_order_release);
    if (_active_tx_owner.load(std::memory_order_acquire) == nullptr &&
        _active_rx_owner.load(std::memory_order_acquire) == nullptr) {
        _operation_configured = false;
    }
    return {};
}

// ---------------------------------------------------------------------------
// updateDerivedState
// ---------------------------------------------------------------------------
void Bus_espidf::updateDerivedState(const i2s::AccessConfig& cfg, size_t& out_frame_bytes)
{
    // Classic ESP32 (I2S HW v1) cannot duplicate a mono sample into both slots
    // in hardware: with slot_mode MONO the sample effectively updates a
    // fixed-slot mono amplifier at fs/2 (measured on Core2/NS4168: correct
    // 440 Hz fundamental plus fs/2 and fs/2±440 spurs — half-rate imaging).
    // M5Unified avoids the same trap on this silicon by duplicating each
    // sample into both 16-bit halves of its 32-bit mixing frames and only
    // uses the hardware mono registers (tx_mono/tx_chan_equal) on HW v2.
    // So on HW v1 we run stereo slots and duplicate in write()/read()
    // (_expand_mono); HW v2 and later keep the native mono slot mode.
#if SOC_I2S_HW_VERSION_1
    _expand_mono = (cfg.channels == 1);
#else
    _expand_mono = false;
#endif
    // IDF 5.5 on HW v2 reports one active DMA slot for MONO+BOTH, yet RX exposes
    // an adjacent L/R-shaped pair (duplicates when the source drives identical
    // mono slots). HW v1 already captures stereo-shaped frames for its software
    // mono path. In both cases read() keeps one physical left sample per frame.
    _collapse_mono_rx  = (cfg.channels == 1);
    const size_t slots = _expand_mono ? 2u : cfg.channels;
    out_frame_bytes    = static_cast<size_t>(cfg.bits_per_sample / 8) * slots;

    // 16-bit stereo on HW v1 transposes the two 16-bit halves of each 32-bit
    // FIFO word (= the L/R slots); undo it in the DMA buffer (see _swap16 doc
    // in the header). Mono is duplicated (L==R, no-op) and 24/32-bit fill a
    // whole word.
#if SOC_I2S_HW_VERSION_1
    _swap16 = (cfg.bits_per_sample == 16) && (slots == 2) && !_expand_mono;
#else
    _swap16 = false;
#endif
}

// ---------------------------------------------------------------------------
// ensureChannel
// ---------------------------------------------------------------------------
result_t<void> Bus_espidf::ensureChannel(const i2s::AccessConfig& cfg)
{
    // Only 16-bit samples are supported.
    if (cfg.bits_per_sample != 16) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    // channels must be 1 or 2.
    if (cfg.channels != 1 && cfg.channels != 2) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    if (!_configured || !impl_espidf::sameAccessConfig(_applied_cfg, cfg)) {
        // need_reconfig (cheap check): take the setup mutex so the create /
        // reconfigure section is single-threaded. A full-duplex bus runs the TX
        // and RX paths under independent channel locks, so both can reach here
        // at once; without this gate they could double-create or destroy each
        // other's live handle. The lock is held only across setup (never nested
        // with the channel locks), and an RAII guard releases it on every
        // return path below.
        auto setup_locked = _setup_mutex.lock(types::TIMEOUT_FOREVER);
        if (!setup_locked.has_value()) {
            return m5::stl::make_unexpected(setup_locked.error());
        }
        runtime::ScopedUnlock unlocker{_setup_mutex};

        // Re-evaluate under the lock (double-checked): another thread may have
        // configured the channel between the cheap check and acquiring the lock.
        const bool need_reconfig = !_configured || !impl_espidf::sameAccessConfig(_applied_cfg, cfg);
        if (!need_reconfig) {
            return {};
        }

        // Full-duplex bus: both TX and RX channels exist. Cannot
        // destroy/recreate — use the IDF reconfig API instead (disable →
        // reconfig clock/slot → re-enable, no channel tear-down).
        //
        // Limitation: i2s_channel_disable stops DMA immediately without
        // draining in-flight buffers (IDF has no drain API). The Slave
        // peer may capture a partial frame at the transition boundary.
        // ESP32-S3 Slaves re-sync within 1-2 frames; ESP32 classic
        // Slaves may need ~40 frames (observed rotl≈41-55/127 in stress
        // tests). Steady-state data after re-sync is always clean. A
        // protocol-level pre-notification would be needed to eliminate
        // the transient entirely.
        if (_configured && _tx_handle != nullptr && _rx_handle != nullptr) {
            // Recompute _expand_mono / _swap16 for the new cfg BEFORE building
            // the slot/clock config below: without this call the slot mode and
            // the write()/read() byte-swap path would keep using the PREVIOUS
            // cfg's derived state against the newly reconfigured hardware.
            size_t reconfig_frame_bytes = 0;
            updateDerivedState(cfg, reconfig_frame_bytes);

            // This path never tears down the channel, but the driver DOES
            // reallocate every descriptor's buffer when the new slot mode
            // changes the frame size (i2s_std_set_slot re-derives the buffer
            // size from the active slot count; measured on ESP32-S3: the DMA
            // heap swings by half the capacity on a mono<->stereo switch,
            // stays put on a rate-only change). The descriptor geometry
            // (desc_num * frame_num) is kept, so capacity tracks
            // frames * frame_bytes. Reallocation also discards whatever was
            // in flight, so the counts drop with it; on a rate-only change
            // buffers and counts survive alike and both stay untouched.
            // Frame size is constant on HW v1 (_expand_mono pins stereo
            // slots), so there this whole block is a no-op.
            const size_t new_tx_capacity = _dma_tx_frames * reconfig_frame_bytes;
            const size_t new_rx_capacity = _dma_rx_frames * reconfig_frame_bytes;
            if (new_tx_capacity != _dma_capacity) {
                _dma_capacity = new_tx_capacity;
                _dma_in_flight.store(0, std::memory_order_relaxed);
            }
            if (new_rx_capacity != _dma_rx_capacity) {
                _dma_rx_capacity = new_rx_capacity;
                _dma_rx_available.store(0, std::memory_order_relaxed);
            }

            const ::i2s_slot_mode_t slot_mode_fdx =
                (cfg.channels == 1 && !_expand_mono) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;
            ::i2s_std_clk_config_t clk_cfg   = I2S_STD_CLK_DEFAULT_CONFIG(cfg.sample_rate_hz);
            ::i2s_std_slot_config_t slot_cfg = {};
            slot_cfg.data_bit_width          = static_cast<i2s_data_bit_width_t>(cfg.bits_per_sample);
            slot_cfg.slot_bit_width          = I2S_SLOT_BIT_WIDTH_16BIT;
            slot_cfg.slot_mode               = slot_mode_fdx;
            slot_cfg.slot_mask               = I2S_STD_SLOT_BOTH;
            slot_cfg.ws_width                = 16;
            slot_cfg.ws_pol                  = false;
            slot_cfg.bit_shift               = true;
#if SOC_I2S_HW_VERSION_1
            slot_cfg.msb_right = false;
#else
            slot_cfg.left_align    = true;
            slot_cfg.big_endian    = false;
            slot_cfg.bit_order_lsb = false;
#endif
            esp_err_t rc = ::i2s_channel_disable(_tx_handle);
            if (rc == ESP_ERR_INVALID_STATE) {
                rc = ESP_OK;
            }
            if (rc == ESP_OK) {
                rc = ::i2s_channel_disable(_rx_handle);
                if (rc == ESP_ERR_INVALID_STATE) {
                    rc = ESP_OK;
                }
            }
            if (rc != ESP_OK) {
                quarantineLifecycleAfterPartialTeardown();
                return m5::stl::make_unexpected(impl_espidf::mapEspErr(rc));
            }
            rc = ::i2s_channel_reconfig_std_clock(_tx_handle, &clk_cfg);
            if (rc == ESP_OK) {
                rc = ::i2s_channel_reconfig_std_slot(_tx_handle, &slot_cfg);
            }
            if (rc == ESP_OK) {
                rc = ::i2s_channel_reconfig_std_clock(_rx_handle, &clk_cfg);
            }
            if (rc == ESP_OK) {
                rc = ::i2s_channel_reconfig_std_slot(_rx_handle, &slot_cfg);
            }
            const esp_err_t tx_enabled = ::i2s_channel_enable(_tx_handle);
            const esp_err_t rx_enabled = ::i2s_channel_enable(_rx_handle);
            if (rc == ESP_OK && tx_enabled != ESP_OK) {
                rc = tx_enabled;
            }
            if (rc == ESP_OK && rx_enabled != ESP_OK) {
                rc = rx_enabled;
            }
            if (rc != ESP_OK) {
                quarantineLifecycleAfterPartialTeardown();
                return m5::stl::make_unexpected(impl_espidf::mapEspErr(rc));
            }
            _applied_cfg = cfg;
            return {};
        }

        // Single-channel bus: disable → destroy → recreate below.
        auto reset = resetForInitialization();
        if (!reset.has_value()) {
            return reset;
        }

        // --- Frame size (bytes per DMA frame = 1 sample across all slots) and
        // the _expand_mono / _swap16 derived state (see updateDerivedState for
        // why HW v1 needs them).
        size_t frame_bytes = 0;
        updateDerivedState(cfg, frame_bytes);

        // --- Which channels to open is pin-driven: DOUT enables TX, DIN enables RX.
        // At least one must be wired, or there is nothing to drive.
        const bool want_tx = (_config.pin_dout >= 0);
        const bool want_rx = (_config.pin_din >= 0);
        if (!want_tx && !want_rx) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }

        // --- DMA sizing (independent per direction; only the wired ones are used)
        uint32_t tx_desc_num  = 0;
        uint32_t tx_frame_num = 0;
        uint32_t rx_desc_num  = 0;
        uint32_t rx_frame_num = 0;
        if (want_tx) {
            impl_espidf::calcDmaParams(_config.tx_buffer_size, frame_bytes, tx_desc_num, tx_frame_num, _dma_capacity);
        }
        if (want_rx) {
            impl_espidf::calcDmaParams(_config.rx_buffer_size, frame_bytes, rx_desc_num, rx_frame_num,
                                       _dma_rx_capacity);
        }

        // --- Channel allocation. Role sets the clock direction (master generates
        // BCLK / WS, slave consumes a peer's). A full-duplex pair shares one
        // controller via a single i2s_new_channel call (shared BCLK / WS,
        // independent DMA) and ONE chan_cfg, so both channels get the same DMA
        // descriptor geometry. The chan_cfg is sized from the TX sizing when TX is
        // wired, otherwise from the RX sizing. Because both channels then share that
        // geometry, the RX accounting capacity is taken from the chan_cfg actually
        // applied (below), not from rx_buffer_size, so _dma_rx_available clamps to
        // the real RX DMA depth in the full-duplex case.
        const ::i2s_role_t role = (_config.role == IBusConfig::Role::Slave) ? I2S_ROLE_SLAVE : I2S_ROLE_MASTER;
        uint32_t rejected_mask  = 0;
        esp_err_t ret           = ESP_ERR_NOT_FOUND;
        while (_controller < 0) {
            _controller = detail_espidf_i2s_controller::claimStandard(rejected_mask);
            if (_controller < 0) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            const auto port_id            = static_cast<decltype(::i2s_chan_config_t{}.id)>(_controller);
            ::i2s_chan_config_t trial_cfg = I2S_CHANNEL_DEFAULT_CONFIG(port_id, role);
            trial_cfg.dma_desc_num        = want_tx ? tx_desc_num : rx_desc_num;
            trial_cfg.dma_frame_num       = want_tx ? tx_frame_num : rx_frame_num;
            trial_cfg.auto_clear          = true;
            ret = ::i2s_new_channel(&trial_cfg, want_tx ? &_tx_handle : nullptr, want_rx ? &_rx_handle : nullptr);
            if (ret == ESP_OK) {
                break;
            }
            // i2s_new_channel may publish one handle before allocating the
            // paired direction fails. Delete every partial channel before
            // trying another controller; otherwise the handle is overwritten
            // on the next iteration and the driver resource becomes unreachable.
            const int8_t rejected_controller = _controller;
            reset                            = resetForInitialization();
            if (!reset.has_value()) {
                return reset;
            }
            rejected_mask |= uint32_t{1} << rejected_controller;
        }
        const auto port_id           = static_cast<decltype(::i2s_chan_config_t{}.id)>(_controller);
        ::i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(port_id, role);
        chan_cfg.dma_desc_num        = want_tx ? tx_desc_num : rx_desc_num;
        chan_cfg.dma_frame_num       = want_tx ? tx_frame_num : rx_frame_num;
        chan_cfg.auto_clear          = true;  // underrun → silence

        // Both channels inherit chan_cfg's geometry, so the true capacity per
        // direction is that geometry (when RX is the only channel this equals
        // the rx_buffer_size sizing already in _dma_rx_capacity, but when
        // paired with TX it is the TX geometry — recompute either way to stay
        // correct). The frames factor is kept per direction so the full-duplex
        // reconfig path above can re-derive capacity when the frame size
        // changes (the driver keeps the geometry but reallocates the buffers).
        const size_t chan_frames = static_cast<size_t>(chan_cfg.dma_desc_num) * chan_cfg.dma_frame_num;
        if (want_tx) {
            _dma_tx_frames = chan_frames;
        }
        if (want_rx) {
            _dma_rx_frames   = chan_frames;
            _dma_rx_capacity = chan_frames * frame_bytes;
        }

        // --- Slot / clock config (Philips standard; stereo slots when
        // _expand_mono substitutes for the unusable HW v1 mono mode)
        const ::i2s_slot_mode_t slot_mode =
            (cfg.channels == 1 && !_expand_mono) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;

        ::i2s_std_config_t std_cfg = {};
        std_cfg.clk_cfg            = I2S_STD_CLK_DEFAULT_CONFIG(cfg.sample_rate_hz);
        // Clock source note (ESP32 classic, measured on hardware): the default
        // PLL_160M fractional divider is exact for rates whose MCLK divides
        // 160 MHz cleanly (e.g. 32 kHz: 160M / 8.192M = 19 + 17/32) but dithers
        // audibly at e.g. 22.05 kHz. I2S_CLK_SRC_APLL was tried as a fix and
        // REJECTED: on IDF 5.5 it produced ~-6 % actual rate at 32 kHz (30.0 kHz
        // measured via the remote-stream drain rate). Prefer sample rates with an
        // exact PLL_160M divider on this chip.
        // Fully explicit Philips slot configuration (mirrors the field set proven
        // on this hardware family by M5Unified's speaker path). TX mono uses BOTH
        // so one logical sample is emitted on both wire slots. RX uses the same
        // mask because HW v2 returns no data with a single-slot mono mask on the
        // tested S3; read() collapses the stereo-shaped physical capture instead.
        std_cfg.slot_cfg.data_bit_width = static_cast<i2s_data_bit_width_t>(cfg.bits_per_sample);
        std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT;
        std_cfg.slot_cfg.slot_mode      = slot_mode;
        std_cfg.slot_cfg.slot_mask      = I2S_STD_SLOT_BOTH;
        std_cfg.slot_cfg.ws_width       = 16;
        std_cfg.slot_cfg.ws_pol         = false;
        std_cfg.slot_cfg.bit_shift      = true;
#if SOC_I2S_HW_VERSION_1
        std_cfg.slot_cfg.msb_right = false;
#else
        std_cfg.slot_cfg.left_align    = true;
        std_cfg.slot_cfg.big_endian    = false;
        std_cfg.slot_cfg.bit_order_lsb = false;
#endif
        std_cfg.gpio_cfg.bclk = static_cast<gpio_num_t>(_config.pin_bclk);
        std_cfg.gpio_cfg.ws   = static_cast<gpio_num_t>(_config.pin_ws);
        std_cfg.gpio_cfg.dout = static_cast<gpio_num_t>(_config.pin_dout);
        std_cfg.gpio_cfg.din  = (_config.pin_din >= 0) ? static_cast<gpio_num_t>(_config.pin_din) : I2S_GPIO_UNUSED;
        std_cfg.gpio_cfg.mclk = (_config.pin_mclk >= 0) ? static_cast<gpio_num_t>(_config.pin_mclk) : I2S_GPIO_UNUSED;
        std_cfg.gpio_cfg.invert_flags.mclk_inv = false;
        std_cfg.gpio_cfg.invert_flags.bclk_inv = false;
        std_cfg.gpio_cfg.invert_flags.ws_inv   = false;

        // --- Init the standard mode on each wired channel with the shared std_cfg
        // (the TX and RX of a full-duplex pair share BCLK / WS / slot format).
        if (_tx_handle != nullptr) {
            ret = ::i2s_channel_init_std_mode(_tx_handle, &std_cfg);
            if (ret != ESP_OK) {
                return failAfterSetup(impl_espidf::mapEspErr(ret));
            }
        }
        if (_rx_handle != nullptr) {
            ret = ::i2s_channel_init_std_mode(_rx_handle, &std_cfg);
            if (ret != ESP_OK) {
                return failAfterSetup(impl_espidf::mapEspErr(ret));
            }
        }

        // --- Register the on_sent / on_recv callbacks for DMA accounting (each on
        // its own channel; the unused hook stays null).
        if (_tx_handle != nullptr) {
            ::i2s_event_callbacks_t cbs = {};
            cbs.on_sent                 = &Bus_espidf::onSentCallback;
            ret                         = ::i2s_channel_register_event_callback(_tx_handle, &cbs, this);
            if (ret != ESP_OK) {
                return failAfterSetup(impl_espidf::mapEspErr(ret));
            }
        }
        if (_rx_handle != nullptr) {
            ::i2s_event_callbacks_t cbs = {};
            cbs.on_recv                 = &Bus_espidf::onRecvCallback;
            ret                         = ::i2s_channel_register_event_callback(_rx_handle, &cbs, this);
            if (ret != ESP_OK) {
                return failAfterSetup(impl_espidf::mapEspErr(ret));
            }
        }

        // --- Preload silence on TX before enabling. Starting the DMA engine on an
        // empty FIFO occasionally comes up byte-misaligned on the classic ESP32
        // (playback becomes harsh metallic noise until a lucky re-init); a full
        // preload makes the start deterministic and frame-aligned. The preloaded
        // bytes are counted as in-flight so writableBytes stays truthful (the
        // on_sent callback drains them as they play out as silence). RX has no
        // equivalent — its DMA starts empty and fills as samples arrive.
        if (_tx_handle != nullptr) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
            uint8_t zeros[480] = {};
            size_t preloaded   = 0;
            size_t loaded      = 0;
            do {
                loaded        = 0;
                esp_err_t pre = ::i2s_channel_preload_data(_tx_handle, zeros, sizeof(zeros), &loaded);
                if (pre != ESP_OK) {
                    break;
                }
                preloaded += loaded;
            } while (loaded == sizeof(zeros) && preloaded < _dma_capacity);
            _dma_in_flight.store(preloaded, std::memory_order_relaxed);
#else
            // ESP-IDF 5.0 has the channel API but not its preload helper.
            // Start empty there; later releases retain the deterministic
            // silence preload used to avoid classic-ESP32 frame misalignment.
            _dma_in_flight.store(0, std::memory_order_relaxed);
#endif
        }

        // --- Enable each wired channel
        if (_tx_handle != nullptr) {
            ret = ::i2s_channel_enable(_tx_handle);
            if (ret != ESP_OK) {
                return failAfterSetup(impl_espidf::mapEspErr(ret));
            }
        }
        if (_rx_handle != nullptr) {
            ret = ::i2s_channel_enable(_rx_handle);
            if (ret != ESP_OK) {
                return failAfterSetup(impl_espidf::mapEspErr(ret));
            }
        }
        _channel_enabled = true;

        _applied_cfg = cfg;
        _configured  = true;
        // NOTE: _dma_in_flight keeps the TX preload count here. Zeroing it would
        // over-report free space by a full DMA capacity right after enable; the
        // host would then overfeed and the dropped (non-blocking) writes turn
        // into audible gaps until the accounting re-equilibrates. _dma_rx_available
        // starts at zero — RX reports readable bytes only as the DMA captures them.
        return {};
    }  // end setup section (the ScopedUnlock releases _setup_mutex here)

    // Already configured with a matching cfg: nothing to do.
    return {};
}

// ---------------------------------------------------------------------------
// write
// ---------------------------------------------------------------------------
result_t<size_t> Bus_espidf::writeBackend(bus::OperationContext<i2s::AccessConfig>& context, data::Source* src,
                                          size_t len)
{
    const auto& cfg = context.config;

    // Direction guard BEFORE ensureChannel: writing on an RX-only bus (no DOUT
    // wired) must not lazily create a TX channel as a side effect.
    if (_config.pin_dout < 0) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }

    // No TX channel on this bus (RX-only wiring): writing is not supported.
    if (_tx_handle == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }

    const ::TickType_t timeout_ticks = impl_espidf::toTicks(cfg.write_timeout_ms);
    size_t done                      = 0;

    while (src != nullptr && !src->eof() && done < len) {
        auto span = src->peek(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }

        if (_expand_mono) {
            // Duplicate each 16-bit sample into both slots via a bounce
            // buffer. `written`/`done` stay in logical (mono) bytes for the
            // caller; only _dma_in_flight counts physical bytes.
            uint8_t bounce[960];
            size_t avail = span.value().size;
            if (avail > sizeof(bounce) / 2) {
                avail = sizeof(bounce) / 2;
            }
            const size_t samples = avail / 2;
            if (samples == 0) {
                break;  // less than one whole sample available
            }
            for (size_t i = 0; i < samples; ++i) {
                bounce[i * 4 + 0] = span.value().data[i * 2 + 0];
                bounce[i * 4 + 1] = span.value().data[i * 2 + 1];
                bounce[i * 4 + 2] = span.value().data[i * 2 + 0];
                bounce[i * 4 + 3] = span.value().data[i * 2 + 1];
            }
            size_t written_phys = 0;
            esp_err_t mret      = ::i2s_channel_write(_tx_handle, bounce, samples * 4, &written_phys, timeout_ticks);
            // Same hazard as the plain path below: never leave the DMA on a
            // partial frame. Complete the frame with short blocking writes
            // (at most 3 bytes; one frame's space frees every frame period).
            while ((written_phys & 3) != 0 && written_phys < samples * 4) {
                size_t completed = 0;
                (void)::i2s_channel_write(_tx_handle, bounce + written_phys, 4 - (written_phys & 3), &completed, 50);
                if (completed == 0) {
                    break;
                }
                written_phys += completed;
            }
            if (written_phys > 0) {
                _dma_in_flight.fetch_add(written_phys, std::memory_order_relaxed);
            }
            const size_t written_logical = (written_phys / 4) * 2;
            if (written_logical > 0) {
                auto advanced = src->advance(written_logical);
                if (!advanced.has_value()) {
                    return m5::stl::make_unexpected(advanced.error());
                }
                done += written_logical;
            }
            if (mret != ESP_OK && mret != ESP_ERR_TIMEOUT) {
                if (done == 0) {
                    return m5::stl::make_unexpected(impl_espidf::mapEspErr(mret));
                }
                break;
            }
            if (mret == ESP_ERR_TIMEOUT || written_logical == 0) {
                break;
            }
            continue;
        }

        if (_swap16) {
            // HW v1 16-bit stereo: write through a bounce with each frame's L/R
            // 16-bit halves transposed so the silicon's transpose lands the data
            // back in standard order on the wire. Accounting is 1:1 (no mono
            // expansion); only whole 4-byte frames are written so the swap never
            // straddles a frame boundary.
            alignas(4) uint8_t bounce[960];
            size_t avail = span.value().size;
            if (avail > sizeof(bounce)) {
                avail = sizeof(bounce);
            }
            avail &= ~static_cast<size_t>(3);  // whole frames only
            if (avail == 0) {
                break;  // less than one whole frame available
            }
            // Copy into the bounce with each frame's L/R 16-bit halves transposed
            // (one fused pass; leaves the caller's source buffer untouched).
            impl_espidf::swapHalfwords(bounce, span.value().data, avail);
            size_t written = 0;
            esp_err_t sret = ::i2s_channel_write(_tx_handle, bounce, avail, &written, timeout_ticks);
            // Never leave the DMA on a partial frame: a frame-straddling resume
            // would desync the L/R swap from then on. One frame frees every frame
            // period, so the short blocking completion is effectively instant.
            while ((written & 3) != 0 && written < avail) {
                size_t completed = 0;
                (void)::i2s_channel_write(_tx_handle, bounce + written, 4 - (written & 3), &completed, 50);
                if (completed == 0) {
                    break;
                }
                written += completed;
            }
            // Fatal error first (mirror the plain path): on a first-chunk fatal
            // error propagate it rather than returning a short success.
            if (sret != ESP_OK && sret != ESP_ERR_TIMEOUT) {
                if (done == 0) {
                    return m5::stl::make_unexpected(impl_espidf::mapEspErr(sret));
                }
                break;
            }
            if (written > 0) {
                _dma_in_flight.fetch_add(written, std::memory_order_relaxed);
                // Advance the source only by whole frames: if completion above
                // could not finish a frame (DMA wedged), never advance a partial
                // frame's worth — that would desync the L/R swap on resume.
                const size_t whole = written & ~static_cast<size_t>(3);
                if (whole > 0) {
                    auto advanced = src->advance(whole);
                    if (!advanced.has_value()) {
                        return m5::stl::make_unexpected(advanced.error());
                    }
                    done += whole;
                }
            }
            if (sret == ESP_ERR_TIMEOUT || written == 0) {
                break;
            }
            continue;
        }

        size_t written = 0;
        esp_err_t ret  = ::i2s_channel_write(_tx_handle, span.value().data, span.value().size, &written, timeout_ticks);

        // Never leave the stream on a half-sample boundary: with non-blocking
        // writes against a nearly full DMA the driver can accept an odd byte
        // count, and a one-byte phase shift turns all subsequent playback into
        // harsh fs/2-sideband noise until the channel is re-initialised. One
        // sample's worth of space frees every sample period, so a short
        // blocking completion is effectively instant.
        if ((written & 1) != 0 && written < span.value().size) {
            size_t completed = 0;
            (void)::i2s_channel_write(_tx_handle, span.value().data + written, 1, &completed, 50);
            written += completed;
        }

        // Short write due to timeout is normal (design spec: short write OK).
        // Other errors are propagated.
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            if (done == 0) {
                return m5::stl::make_unexpected(impl_espidf::mapEspErr(ret));
            }
            break;
        }

        if (written > 0) {
            _dma_in_flight.fetch_add(written, std::memory_order_relaxed);
            auto advanced = src->advance(written);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
            done += written;
        }

        // A timeout ends the write: looping again would wait another full
        // timeout window per chunk, exceeding the write_timeout_ms contract.
        if (ret == ESP_ERR_TIMEOUT || written == 0) {
            break;
        }
    }

    return done;
}

// ---------------------------------------------------------------------------
// writableBytes
// ---------------------------------------------------------------------------
result_t<size_t> Bus_espidf::writableBytesBackend(bus::OperationContext<i2s::AccessConfig>& context)
{
    (void)context;

    // Direction guard BEFORE ensureChannel: an RX-only bus (no DOUT) has nothing
    // writable, and querying it must not lazily create a TX channel.
    if (_config.pin_dout < 0) {
        return static_cast<size_t>(0);
    }

    // No TX channel on this bus (RX-only wiring): nothing is writable.
    if (_tx_handle == nullptr) {
        return static_cast<size_t>(0);
    }

    const size_t in_flight = _dma_in_flight.load(std::memory_order_relaxed);
    const size_t capacity  = _dma_capacity;
    const size_t free_phys = (in_flight < capacity) ? (capacity - in_flight) : 0;
    return _expand_mono ? free_phys / 2 : free_phys;
}

// ---------------------------------------------------------------------------
// read
// ---------------------------------------------------------------------------
result_t<size_t> Bus_espidf::readBackend(bus::OperationContext<i2s::AccessConfig>& context, data::Sink* dst, size_t len)
{
    const auto& cfg = context.config;

    // Direction guard BEFORE ensureChannel: reading on a TX-only bus (no DIN
    // wired) must not lazily create an RX channel as a side effect.
    if (_config.pin_din < 0) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }

    // No RX channel on this bus (TX-only wiring): reading is not supported.
    if (_rx_handle == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }

    const ::TickType_t timeout_ticks = impl_espidf::toTicks(cfg.read_timeout_ms);
    size_t done                      = 0;

    while (dst != nullptr && !dst->closed() && done < len) {
        // Reserve scratch from the sink; the captured DMA bytes land there.
        auto reserved = dst->reserve(len - done);
        if (!reserved.has_value()) {
            return m5::stl::make_unexpected(reserved.error());
        }
        size_t want = reserved.value().size;
        if (want == 0) {
            break;  // sink is full
        }

        if (_collapse_mono_rx) {
            // Mono capture: the DMA delivers stereo-shaped physical pairs, so
            // read them into a bounce buffer and keep the left slot of each
            // frame as the logical mono sample. With a mono TX the pair is
            // duplicated. `want` (and `done`) stay in logical bytes; only the
            // i2s_channel_read size is physical.
            uint8_t bounce[960];
            size_t logical_cap = want;
            if (logical_cap > sizeof(bounce) / 2) {
                logical_cap = sizeof(bounce) / 2;
            }
            const size_t samples = logical_cap / 2;  // 2 bytes per 16-bit mono sample
            if (samples == 0) {
                break;  // less than one whole sample of room
            }
            size_t read_phys = 0;
            esp_err_t rret   = ::i2s_channel_read(_rx_handle, bounce, samples * 4, &read_phys, timeout_ticks);
            // Never leave the stream on a partial physical frame (4 bytes): a
            // one-sample phase shift would swap the L/R slots from then on.
            while ((read_phys & 3) != 0 && read_phys < samples * 4) {
                size_t completed = 0;
                (void)::i2s_channel_read(_rx_handle, bounce + read_phys, 4 - (read_phys & 3), &completed, 50);
                if (completed == 0) {
                    break;
                }
                read_phys += completed;
            }
            const size_t got_samples = read_phys / 4;
            if (got_samples > 0) {
#if SOC_I2S_HW_VERSION_1
                // HW v1 transposes the two 16-bit halves in every DMA word;
                // raw pair[1] is therefore the physical left slot. Mono TX
                // duplicates L/R so the distinction only shows with an
                // external source whose slots differ.
                constexpr bool kHwVersion1 = true;
#else
                constexpr bool kHwVersion1 = false;
#endif
                const size_t got_logical = detail_espidf_i2s::collapseStereo16RxPairsToMonoLeft(
                    reserved.value().data, bounce, got_samples, kHwVersion1);
                auto committed = dst->commit(got_logical);
                if (!committed.has_value()) {
                    return m5::stl::make_unexpected(committed.error());
                }
                // Clamped subtraction on the physical accounting.
                size_t cur = _dma_rx_available.load(std::memory_order_relaxed);
                while (cur != 0) {
                    const size_t dec  = (read_phys < cur) ? read_phys : cur;
                    const size_t next = cur - dec;
                    if (_dma_rx_available.compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
                        break;
                    }
                }
                done += got_logical;
            }
            if (rret != ESP_OK && rret != ESP_ERR_TIMEOUT) {
                if (done == 0) {
                    return m5::stl::make_unexpected(impl_espidf::mapEspErr(rret));
                }
                break;
            }
            if (rret == ESP_ERR_TIMEOUT || got_samples == 0) {
                break;
            }
            continue;
        }

        if (_swap16) {
            // HW v1 16-bit stereo: capture whole frames, then transpose each
            // frame's L/R 16-bit halves in place to undo the silicon's transpose
            // (the wire carries standard Philips). Round to 4-byte frames and
            // complete any partial frame so the swap never straddles a boundary.
            want &= ~static_cast<size_t>(3);
            if (want == 0) {
                break;  // less than one whole frame of room
            }
            size_t read_bytes = 0;
            esp_err_t sret    = ::i2s_channel_read(_rx_handle, reserved.value().data, want, &read_bytes, timeout_ticks);
            while ((read_bytes & 3) != 0 && read_bytes < want) {
                size_t completed = 0;
                (void)::i2s_channel_read(_rx_handle, reserved.value().data + read_bytes, 4 - (read_bytes & 3),
                                         &completed, 50);
                if (completed == 0) {
                    break;
                }
                read_bytes += completed;
            }
            impl_espidf::swapHalfwords(reserved.value().data, reserved.value().data, read_bytes);
            if (sret != ESP_OK && sret != ESP_ERR_TIMEOUT) {
                if (done == 0) {
                    return m5::stl::make_unexpected(impl_espidf::mapEspErr(sret));
                }
                break;
            }
            if (read_bytes > 0) {
                // Commit only whole frames: swap16Frames leaves a trailing partial
                // frame untouched (completion above could not finish it), so never
                // expose an unswapped half-frame to the caller. The partial bytes
                // were still drained from the DMA, so decrement the full count.
                const size_t whole = read_bytes & ~static_cast<size_t>(3);
                if (whole > 0) {
                    auto committed = dst->commit(whole);
                    if (!committed.has_value()) {
                        return m5::stl::make_unexpected(committed.error());
                    }
                    done += whole;
                }
                size_t cur = _dma_rx_available.load(std::memory_order_relaxed);
                while (cur != 0) {
                    const size_t dec  = (read_bytes < cur) ? read_bytes : cur;
                    const size_t next = cur - dec;
                    if (_dma_rx_available.compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
                        break;
                    }
                }
            }
            if (sret == ESP_ERR_TIMEOUT || read_bytes == 0) {
                break;
            }
            continue;
        }

        // Round the request DOWN to a whole 16-bit sample before reading: when
        // `want` is odd and i2s_channel_read happens to return exactly `want`,
        // the completion guard below (which only fires on a short read) is
        // skipped and an odd byte count would be committed — a one-byte phase
        // shift that turns every later sample into fs/2-sideband noise.
        want &= ~static_cast<size_t>(1);
        if (want == 0) {
            break;  // less than one whole sample of room
        }

        size_t read_bytes = 0;
        esp_err_t ret     = ::i2s_channel_read(_rx_handle, reserved.value().data, want, &read_bytes, timeout_ticks);

        // Never leave the stream on a half-sample boundary (2 bytes): a one-byte
        // phase shift turns subsequent samples into noise (the TX path guards the
        // same hazard). One sample's worth frees every sample period, so a short
        // blocking completion is effectively instant.
        if ((read_bytes & 1) != 0 && read_bytes < want) {
            size_t completed = 0;
            (void)::i2s_channel_read(_rx_handle, reserved.value().data + read_bytes, 1, &completed, 50);
            read_bytes += completed;
        }

        // Short read due to timeout is normal (continuous stream, short read OK).
        // Other errors are propagated.
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            if (done == 0) {
                return m5::stl::make_unexpected(impl_espidf::mapEspErr(ret));
            }
            break;
        }

        if (read_bytes > 0) {
            auto committed = dst->commit(read_bytes);
            if (!committed.has_value()) {
                return m5::stl::make_unexpected(committed.error());
            }
            // Clamped subtraction on the captured-bytes accounting.
            size_t cur = _dma_rx_available.load(std::memory_order_relaxed);
            while (cur != 0) {
                const size_t dec  = (read_bytes < cur) ? read_bytes : cur;
                const size_t next = cur - dec;
                if (_dma_rx_available.compare_exchange_weak(cur, next, std::memory_order_relaxed)) {
                    break;
                }
            }
            done += read_bytes;
        }

        // A timeout ends the read: looping again would wait another full timeout
        // window per chunk, exceeding the read_timeout_ms contract.
        if (ret == ESP_ERR_TIMEOUT || read_bytes == 0) {
            break;
        }
    }

    return done;
}

// ---------------------------------------------------------------------------
// readableBytes
// ---------------------------------------------------------------------------
result_t<size_t> Bus_espidf::readableBytesBackend(bus::OperationContext<i2s::AccessConfig>& context)
{
    (void)context;

    // Direction guard BEFORE ensureChannel: a TX-only bus (no DIN) has nothing
    // readable, and querying it must not lazily create an RX channel.
    if (_config.pin_din < 0) {
        return static_cast<size_t>(0);
    }

    // No RX channel on this bus (TX-only wiring): nothing is readable.
    if (_rx_handle == nullptr) {
        return static_cast<size_t>(0);
    }

    const size_t available = _dma_rx_available.load(std::memory_order_relaxed);
    // Public accounting stays in logical (mono) bytes; the physical capture is
    // stereo-shaped physical pairs in mono mode, so halve it.
    return _collapse_mono_rx ? available / 2 : available;
}

}  // namespace m5::hal::v2::i2s

#endif  // defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_INL
