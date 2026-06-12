// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_INL

#include "i2s.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#include <esp_err.h>
#include <freertos/FreeRTOS.h>

namespace m5::hal::v1::i2s {

namespace {
namespace impl_espidf {

::m5::hal::v1::error::error_t mapEspErr(::esp_err_t err)
{
    switch (err) {
        case ESP_OK:
            return ::m5::hal::v1::error::error_t::OK;
        case ESP_ERR_INVALID_ARG:
        case ESP_ERR_INVALID_STATE:
            return ::m5::hal::v1::error::error_t::INVALID_ARGUMENT;
        case ESP_ERR_TIMEOUT:
            return ::m5::hal::v1::error::error_t::TIMEOUT_ERROR;
        case ESP_ERR_NO_MEM:
            return ::m5::hal::v1::error::error_t::OUT_OF_RESOURCE;
        default:
            return ::m5::hal::v1::error::error_t::IO_ERROR;
    }
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

bool sameAccessConfig(const ::m5::hal::v1::i2s::AccessConfig& a, const ::m5::hal::v1::i2s::AccessConfig& b)
{
    return a.sample_rate_hz == b.sample_rate_hz && a.bits_per_sample == b.bits_per_sample && a.channels == b.channels;
}

::TickType_t toTicks(uint32_t timeout_ms)
{
    return timeout_ms == 0 ? 0 : pdMS_TO_TICKS(timeout_ms);
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

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
::m5::hal::v1::result_t<void> Bus_espidf::init(const BusConfig_espidf& config)
{
    // Re-init: tear down any existing channel first — clearing the handle
    // without `i2s_del_channel` would leak the old channel with its DMA
    // still running.
    destroyChannel();
    _config          = config;
    _configured      = false;
    _channel_enabled = false;
    _dma_in_flight.store(0, std::memory_order_relaxed);
    _dma_capacity = 0;
    return {};
}

// ---------------------------------------------------------------------------
// release
// ---------------------------------------------------------------------------
::m5::hal::v1::result_t<void> Bus_espidf::release(void)
{
    destroyChannel();
    return {};
}

void Bus_espidf::destroyChannel(void)
{
    if (_tx_handle != nullptr) {
        if (_channel_enabled) {
            ::i2s_channel_disable(_tx_handle);
            _channel_enabled = false;
        }
        ::i2s_del_channel(_tx_handle);
        _tx_handle  = nullptr;
        _configured = false;
        _dma_in_flight.store(0, std::memory_order_relaxed);
        _dma_capacity = 0;
    }
}

// ---------------------------------------------------------------------------
// ensureChannel
// ---------------------------------------------------------------------------
::m5::hal::v1::result_t<void> Bus_espidf::ensureChannel(const ::m5::hal::v1::i2s::AccessConfig& cfg)
{
    // Only 16-bit samples are supported.
    if (cfg.bits_per_sample != 16) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    // channels must be 1 or 2.
    if (cfg.channels != 1 && cfg.channels != 2) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    const bool need_reconfig = !_configured || !impl_espidf::sameAccessConfig(_applied_cfg, cfg);
    if (!need_reconfig) {
        return {};
    }

    // Disable → destroy existing channel before reconfiguring.
    destroyChannel();

    // --- Frame size (bytes per DMA frame = 1 sample across all slots).
    // Classic ESP32 (I2S HW v1) cannot duplicate a mono sample into both
    // slots in hardware: with slot_mode MONO the sample effectively updates
    // a fixed-slot mono amplifier at fs/2 (measured on Core2/NS4168: correct
    // 440 Hz fundamental plus fs/2 and fs/2±440 spurs — half-rate imaging).
    // M5Unified avoids the same trap on this silicon by duplicating each
    // sample into both 16-bit halves of its 32-bit mixing frames and only
    // uses the hardware mono registers (tx_mono/tx_chan_equal) on HW v2.
    // So on HW v1 we run stereo slots and duplicate in write()
    // (_expand_mono); HW v2 and later keep the native mono slot mode.
#if SOC_I2S_HW_VERSION_1
    _expand_mono = (cfg.channels == 1);
#else
    _expand_mono = false;
#endif
    const size_t slots       = _expand_mono ? 2u : cfg.channels;
    const size_t frame_bytes = static_cast<size_t>(cfg.bits_per_sample / 8) * slots;

    // --- DMA sizing
    uint32_t dma_desc_num  = 0;
    uint32_t dma_frame_num = 0;
    impl_espidf::calcDmaParams(_config.tx_buffer_size, frame_bytes, dma_desc_num, dma_frame_num, _dma_capacity);

    // --- Channel allocation
    ::i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num        = dma_desc_num;
    chan_cfg.dma_frame_num       = dma_frame_num;
    chan_cfg.auto_clear          = true;  // underrun → silence

    esp_err_t ret = ::i2s_new_channel(&chan_cfg, &_tx_handle, nullptr);
    if (ret != ESP_OK) {
        _tx_handle = nullptr;
        return m5::stl::make_unexpected(impl_espidf::mapEspErr(ret));
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
    // on this hardware family by M5Unified's speaker path). slot_mask BOTH
    // matches M5Unified; on HW v2 with slot_mode MONO the driver maps this to
    // its hardware sample-duplication path (HW v1 mono never reaches here —
    // see _expand_mono above).
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

    ret = ::i2s_channel_init_std_mode(_tx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ::i2s_del_channel(_tx_handle);
        _tx_handle = nullptr;
        return m5::stl::make_unexpected(impl_espidf::mapEspErr(ret));
    }

    // --- Register the on_sent callback for DMA consumption tracking
    ::i2s_event_callbacks_t cbs = {};
    cbs.on_sent                 = &Bus_espidf::onSentCallback;
    ret                         = ::i2s_channel_register_event_callback(_tx_handle, &cbs, this);
    if (ret != ESP_OK) {
        ::i2s_del_channel(_tx_handle);
        _tx_handle = nullptr;
        return m5::stl::make_unexpected(impl_espidf::mapEspErr(ret));
    }

    // --- Preload silence before enabling. Starting the DMA engine on an
    // empty FIFO occasionally comes up byte-misaligned on the classic ESP32
    // (playback becomes harsh metallic noise until a lucky re-init); a full
    // preload makes the start deterministic and frame-aligned. The preloaded
    // bytes are counted as in-flight so writableBytes stays truthful (the
    // on_sent callback drains them as they play out as silence).
    {
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
    }

    // --- Enable
    ret = ::i2s_channel_enable(_tx_handle);
    if (ret != ESP_OK) {
        ::i2s_del_channel(_tx_handle);
        _tx_handle = nullptr;
        return m5::stl::make_unexpected(impl_espidf::mapEspErr(ret));
    }
    _channel_enabled = true;

    _applied_cfg = cfg;
    _configured  = true;
    // NOTE: _dma_in_flight keeps the preload count here. Zeroing it would
    // over-report free space by a full DMA capacity right after enable; the
    // host would then overfeed and the dropped (non-blocking) writes turn
    // into audible gaps until the accounting re-equilibrates.
    return {};
}

// ---------------------------------------------------------------------------
// write
// ---------------------------------------------------------------------------
::m5::hal::v1::result_t<size_t> Bus_espidf::write(::m5::hal::v1::bus::IAccessor* owner,
                                                  const ::m5::hal::v1::i2s::AccessConfig& cfg,
                                                  ::m5::hal::v1::data::Source* tx, size_t len)
{
    (void)owner;

    auto ensure = ensureChannel(cfg);
    if (!ensure.has_value()) {
        return m5::stl::make_unexpected(ensure.error());
    }

    const ::TickType_t timeout_ticks = impl_espidf::toTicks(cfg.write_timeout_ms);
    size_t done                      = 0;

    while (tx != nullptr && !tx->eof() && done < len) {
        auto span = tx->peek(len - done);
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
                auto advanced = tx->advance(written_logical);
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
            auto advanced = tx->advance(written);
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
::m5::hal::v1::result_t<size_t> Bus_espidf::writableBytes(::m5::hal::v1::bus::IAccessor* owner,
                                                          const ::m5::hal::v1::i2s::AccessConfig& cfg)
{
    (void)owner;

    auto ensure = ensureChannel(cfg);
    if (!ensure.has_value()) {
        return m5::stl::make_unexpected(ensure.error());
    }

    const size_t in_flight = _dma_in_flight.load(std::memory_order_relaxed);
    const size_t capacity  = _dma_capacity;
    const size_t free_phys = (in_flight < capacity) ? (capacity - in_flight) : 0;
    return _expand_mono ? free_phys / 2 : free_phys;
}

}  // namespace m5::hal::v1::i2s

#endif  // defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_INL
