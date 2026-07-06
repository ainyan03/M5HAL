// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/i2s/i2s.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#include <atomic>
#include <driver/i2s_std.h>

namespace m5::hal::v2::i2s {

// This variant needs no fields beyond the abstract kind config; the
// empty derivation still gives `init` a variant-owned type, so a
// sibling variant's config cannot be passed by accident (the same
// typed-init guarantee as variants with extra fields).
struct BusConfig_espidf : public i2s::IBusConfig {
    using IBusConfig::IBusConfig;
};

// ESP-IDF gen5 (driver/i2s_std.h) concrete I2S bus (TX and/or RX).
// The channels are created lazily on the first write / read. On each call the
// AccessConfig is compared to the previous one; if it changed the channels are
// disabled, reconfigured, then re-enabled. Which channels exist is pin-driven:
// pin_dout >= 0 creates a TX channel, pin_din >= 0 creates an RX channel, both
// create a full-duplex pair on one controller (shared BCLK / WS, independent
// DMA). Role (master / slave) sets the clock direction. Underruns are silent
// (ESP-IDF auto_clear = true). TX DMA bytes consumed are tracked via on_sent
// for writableBytes(); RX DMA bytes captured are tracked via on_recv for
// readableBytes().
class Bus_espidf : public i2s::IBus {
public:
    ~Bus_espidf() override
    {
        (void)release();
    }

    result_t<void> init(const BusConfig_espidf& config);
    result_t<void> release(void) override;

    result_t<size_t> write(bus::IAccessor* owner, const i2s::AccessConfig& cfg, data::Source* src, size_t len) override;

    result_t<size_t> writableBytes(bus::IAccessor* owner, const i2s::AccessConfig& cfg) override;

    result_t<size_t> read(bus::IAccessor* owner, const i2s::AccessConfig& cfg, data::Sink* dst, size_t len) override;

    result_t<size_t> readableBytes(bus::IAccessor* owner, const i2s::AccessConfig& cfg) override;

private:
    // Lazily open (or reconfigure) the I2S channels to match cfg. Creates a TX
    // channel when pin_dout >= 0 and an RX channel when pin_din >= 0.
    result_t<void> ensureChannel(const i2s::AccessConfig& cfg);

    // Close and delete both channels if they exist.
    void destroyChannel(void);

    // Static event callbacks registered with i2s_channel_register_event_callback.
    static bool onSentCallback(::i2s_chan_handle_t handle, ::i2s_event_data_t* event, void* user_ctx);
    static bool onRecvCallback(::i2s_chan_handle_t handle, ::i2s_event_data_t* event, void* user_ctx);

    ::i2s_chan_handle_t _tx_handle = nullptr;
    ::i2s_chan_handle_t _rx_handle = nullptr;
    bool _channel_enabled          = false;

    // Serializes channel create / reconfigure. ensureChannel() runs while the
    // caller holds ONE channel lock (TX or RX); in a full-duplex bus the TX and
    // RX paths can therefore enter ensureChannel() concurrently. This leaf mutex
    // makes the create/reconfigure section single-threaded (with a re-check of
    // _configured inside) so the two directions cannot double-create or destroy
    // each other's handle. Taken alone, never nested with the channel locks.
    runtime::Mutex _setup_mutex;

    // Bytes of OUR data currently in the TX DMA pipeline. write() adds; the
    // on_sent callback subtracts with a clamp at zero. The clamp is what
    // makes the accounting correct under auto_clear: silence buffers played
    // during an underrun also fire on_sent, but with in-flight already at
    // zero they subtract nothing (a submitted/consumed counter pair would
    // instead drift and pin writableBytes at full capacity forever).
    std::atomic<size_t> _dma_in_flight{0};

    // Bytes captured by the RX DMA and not yet drained by read(). The on_recv
    // callback adds (clamped at capacity so a slow reader cannot over-report);
    // read() subtracts what it drains. The mirror of _dma_in_flight on the
    // read side: it bounds readableBytes() to what the DMA actually holds.
    std::atomic<size_t> _dma_rx_available{0};

    // Effective TX DMA buffer capacity computed from dma_desc_num *
    // dma_frame_num * frame_size, set in ensureChannel.
    size_t _dma_capacity = 0;

    // Effective RX DMA buffer capacity (same derivation as _dma_capacity, sized
    // from rx_buffer_size); bounds _dma_rx_available.
    size_t _dma_rx_capacity = 0;

    // channels==1 on I2S HW v2 (classic ESP32): run stereo slots and let
    // write() duplicate each sample into L/R (see ensureChannel for why the
    // native mono slot mode is unusable there). Public accounting (write
    // return, writableBytes, the remote credit built on them) stays in
    // logical mono bytes; _dma_in_flight alone holds physical bytes.
    bool _expand_mono = false;

    // 16-bit stereo on I2S HW v2 (classic ESP32 / ESP32-S2): the silicon packs
    // two 16-bit samples into a 32-bit FIFO word with the halves transposed
    // (Espressif documents this as "data swapped every two data"), which for
    // stereo swaps the L/R slots. write()/read() undo it in the DMA buffer so the
    // wire stays standard Philips. Only true stereo 16-bit needs it (mono is
    // duplicated into both slots → the transpose is a no-op; 24/32-bit fill a
    // whole FIFO word). Measured on Core2(ESP32)↔CoreS3(ESP32-S3): both link
    // directions show a deterministic adjacent-pair swap without this.
    bool _swap16 = false;

    // Last applied AccessConfig (used to detect when reconfiguration is needed).
    i2s::AccessConfig _applied_cfg;
    bool _configured = false;
};

// Facade backend selection: i2s::Bus::init(BusConfig_espidf) -> Bus_espidf.
template <>
struct BackendFor<BusConfig_espidf> {
    using type = Bus_espidf;
};

}  // namespace m5::hal::v2::i2s

#endif  // defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_HPP
