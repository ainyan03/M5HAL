// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_HPP

#include "../../detail/espidf_version.hpp"
#include "controller_lease.hpp"
#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../../../../hal/v2/i2s/i2s.hpp"

namespace m5::hal::v2::i2s::detail_espidf_i2s {

// Collapse stereo-shaped 16-bit DMA frames to the physical left slot. I2S HW
// v1 transposes the two halfwords in each DMA word; later hardware does not.
// Kept outside the ESP guard so the production primitive is native-testable.
inline size_t collapseStereo16RxPairsToMonoLeft(uint8_t* dst, const uint8_t* src, size_t physical_frame_count,
                                                bool hw_version_1)
{
    const size_t left_offset = hw_version_1 ? 2u : 0u;
    for (size_t i = 0; i < physical_frame_count; ++i) {
        dst[i * 2 + 0] = src[i * 4 + left_offset + 0];
        dst[i * 2 + 1] = src[i * 4 + left_offset + 1];
    }
    return physical_frame_count * 2;
}

}  // namespace m5::hal::v2::i2s::detail_espidf_i2s

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#include <atomic>
#include <driver/i2s_std.h>

namespace m5::hal::v2::i2s {

// ESP-IDF gen5 (driver/i2s_std.h) concrete I2S bus (TX and/or RX).
// The channels are created lazily when an Access begins. The requested
// AccessConfig is applied once for that Access. Which channels exist is pin-driven:
// pin_dout >= 0 creates a TX channel, pin_din >= 0 creates an RX channel, both
// create a full-duplex pair on one controller (shared BCLK / WS, independent
// DMA). Role (master / slave) sets the clock direction. Underruns are silent
// (ESP-IDF auto_clear = true). TX DMA bytes consumed are tracked via on_sent
// for writableBytes(); RX DMA bytes captured are tracked via on_recv for
// readableBytes().
class Bus_espidf : public i2s::IBus {
public:
    ~Bus_espidf() override;

    result_t<void> init(const IBusConfig& config);
    result_t<void> close(void)
    {
        return bus::IBus::close();
    }

    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }
    bus::BusCapabilities capabilities(void) const override
    {
        return bus::detail::BusCapabilitiesBuilder{i2s::IBus::capabilities()}
            .enable(bus::BusFeature::Transmit, _config.pin_dout >= 0)
            .enable(bus::BusFeature::Receive, _config.pin_din >= 0)
            .enable(bus::BusFeature::FullDuplex, _config.pin_dout >= 0 && _config.pin_din >= 0)
            .build();
    }

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<i2s::AccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<i2s::AccessConfig>& context) override;

    result_t<size_t> writeBackend(bus::OperationContext<i2s::AccessConfig>& context, data::Source* src,
                                  size_t len) override;

    result_t<size_t> writableBytesBackend(bus::OperationContext<i2s::AccessConfig>& context) override;

    result_t<size_t> readBackend(bus::OperationContext<i2s::AccessConfig>& context, data::Sink* dst,
                                 size_t len) override;

    result_t<size_t> readableBytesBackend(bus::OperationContext<i2s::AccessConfig>& context) override;

    bus::CloseOutcome closeBackend(void) override;

private:
    // Lazily open (or reconfigure) the I2S channels to match cfg. Creates a TX
    // channel when pin_dout >= 0 and an RX channel when pin_din >= 0.
    result_t<void> ensureChannel(const i2s::AccessConfig& cfg);

    // Close and delete both channels if they exist.
    bus::CloseOutcome teardownBackend(void);
    result_t<void> resetForInitialization(void);
    result_t<void> failAfterSetup(error::error_t cause);

    // Recompute _expand_mono / _swap16 from cfg (see the field comments below
    // for why HW v2 needs them) and report the resulting frame size (bytes per
    // DMA frame) for DMA buffer sizing. Shared by ensureChannel's create path
    // and its in-place full-duplex reconfig path so neither can apply a slot
    // config while working from stale derived state.
    void updateDerivedState(const i2s::AccessConfig& cfg, size_t& out_frame_bytes);

    // Static event callbacks registered with i2s_channel_register_event_callback.
    static bool onSentCallback(::i2s_chan_handle_t handle, ::i2s_event_data_t* event, void* user_ctx);
    static bool onRecvCallback(::i2s_chan_handle_t handle, ::i2s_event_data_t* event, void* user_ctx);

    ::i2s_chan_handle_t _tx_handle = nullptr;
    ::i2s_chan_handle_t _rx_handle = nullptr;
    int8_t _controller             = -1;
    bool _channel_enabled          = false;

    // Serializes channel create / reconfigure. ensureChannel() runs while the
    // caller holds ONE channel lock (TX or RX); in a full-duplex bus the TX and
    // RX paths can therefore enter ensureChannel() concurrently. This leaf mutex
    // makes the create/reconfigure section single-threaded (with a re-check of
    // _configured inside) so the two directions cannot double-create or destroy
    // each other's handle. Taken alone, never nested with the channel locks.
    runtime::Mutex _setup_mutex;

    // Coordinates the independently locked TX/RX Access scopes while applying
    // the one physical channel configuration shared by both directions.
    runtime::Mutex _operation_mutex;
    std::atomic<bus::IAccessor*> _active_tx_owner{nullptr};
    std::atomic<bus::IAccessor*> _active_rx_owner{nullptr};
    i2s::AccessConfig _active_operation_cfg;
    bool _operation_configured = false;

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

    // DMA descriptor geometry (dma_desc_num * dma_frame_num) applied at channel
    // creation, per direction. The IDF keeps this geometry across the
    // full-duplex reconfig path but reallocates every descriptor's buffer when
    // the new slot mode changes the frame size, so the effective capacity is
    // frames * frame_bytes — these hold the constant `frames` factor for that
    // recomputation (frame_bytes is per-AccessConfig).
    size_t _dma_tx_frames = 0;
    size_t _dma_rx_frames = 0;

    // channels==1 on I2S HW v1 (classic ESP32): run stereo slots and let
    // write() duplicate each sample into L/R (see ensureChannel for why the
    // native mono slot mode is unusable there). Public accounting (write
    // return, writableBytes, the remote credit built on them) stays in
    // logical mono bytes; _dma_in_flight alone holds physical bytes.
    bool _expand_mono = false;

    // channels==1 RX normalization. HW v1 captures the stereo-shaped frames
    // used by _expand_mono; HW v2 native MONO+BOTH also exposes adjacent slots
    // (duplicates when the source drives the same mono sample on L/R) on IDF
    // 5.5. read() keeps the physical left sample from each pair so the public
    // buffer remains logical mono without changing TX's native-mono fast path.
    bool _collapse_mono_rx = false;

    // 16-bit stereo on I2S HW v1 (classic ESP32 / ESP32-S2): the silicon packs
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

inline result_t<std::unique_ptr<IBus>> makePortableBackend_espidf(const bus::LocalResourceContext& resources,
                                                                  const IBusConfig& config)
{
    return bus::makePortableBackend<IBus, Bus_espidf, IBusConfig>(resources, config);
}

template <class Policy>
struct NativeProvider_espidf {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, Policy)
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
};

}  // namespace m5::hal::v2::i2s

#endif  // defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_HPP
