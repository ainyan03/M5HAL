#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v1/bus/bus.hpp"
#include "../../../../../hal/v1/i2s/i2s.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#include <atomic>
#include <driver/i2s_std.h>

namespace m5::variants::frameworks::espidf::hal::v1::i2s {

using namespace ::m5::hal::v1;  // resolve unqualified types/bus:: refs

// ESP-IDF gen5 (driver/i2s_std.h) concrete I2S TX bus.
// The channel is created lazily on the first write. On each call the
// AccessConfig is compared to the previous one; if it changed the channel
// is disabled, reconfigured, then re-enabled. Underruns are silent (ESP-IDF
// auto_clear = true). DMA bytes consumed are tracked via the on_sent event
// callback to provide an accurate writableBytes() estimate.
class Bus : public ::m5::hal::v1::i2s::I2SBus {
public:
    ~Bus() override
    {
        (void)release();
    }

    m5::stl::expected<void, ::m5::hal::v1::error::error_t> init(const ::m5::hal::v1::bus::BusConfig& config) override;
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> release(void) override;

    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> write(::m5::hal::v1::bus::Accessor* owner,
                                                                   const ::m5::hal::v1::i2s::I2SAccessConfig& cfg,
                                                                   ::m5::hal::v1::data::Source* tx,
                                                                   size_t len) override;

    m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> writableBytes(
        ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::i2s::I2SAccessConfig& cfg) override;

private:
    // Lazily open (or reconfigure) the I2S channel to match cfg.
    m5::stl::expected<void, ::m5::hal::v1::error::error_t> ensureChannel(
        const ::m5::hal::v1::i2s::I2SAccessConfig& cfg);

    // Close and delete the channel if it exists.
    void destroyChannel(void);

    // Static event callback registered with i2s_channel_register_event_callback.
    static bool onSentCallback(::i2s_chan_handle_t handle, ::i2s_event_data_t* event, void* user_ctx);

    ::i2s_chan_handle_t _tx_handle = nullptr;
    bool _channel_enabled          = false;

    // Bytes of OUR data currently in the DMA pipeline. write() adds; the
    // on_sent callback subtracts with a clamp at zero. The clamp is what
    // makes the accounting correct under auto_clear: silence buffers played
    // during an underrun also fire on_sent, but with in-flight already at
    // zero they subtract nothing (a submitted/consumed counter pair would
    // instead drift and pin writableBytes at full capacity forever).
    std::atomic<size_t> _dma_in_flight{0};

    // Effective DMA buffer capacity computed from dma_desc_num * dma_frame_num
    // * frame_size, set in ensureChannel.
    size_t _dma_capacity = 0;

    // channels==1 on I2S HW v1 (classic ESP32): run stereo slots and let
    // write() duplicate each sample into L/R (see ensureChannel for why the
    // native mono slot mode is unusable there). Public accounting (write
    // return, writableBytes, the remote credit built on them) stays in
    // logical mono bytes; _dma_in_flight alone holds physical bytes.
    bool _expand_mono = false;

    // Last applied AccessConfig (used to detect when reconfiguration is needed).
    ::m5::hal::v1::i2s::I2SAccessConfig _applied_cfg;
    bool _configured = false;
};

}  // namespace m5::variants::frameworks::espidf::hal::v1::i2s

#endif  // defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2S_I2S_HPP
