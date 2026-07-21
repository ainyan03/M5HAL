// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_PDM_PDM_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_PDM_PDM_INL

#include "pdm.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_PDM_HAS_RX_PCM

#include "../i2s/controller_lease.hpp"
#include "../../../freertos/hal/runtime/time.hpp"

#include "../../detail/esp_err_map.hpp"

#include <driver/gpio.h>
#include <esp_err.h>

namespace m5::hal::v2::pdm {
namespace {
error::error_t mapEspErr(esp_err_t err)
{
    // NOT_FOUND: the i2s channel allocator reports a free-channel miss with
    // this code — a resource-exhaustion case for this driver.
    if (err == ESP_ERR_NOT_FOUND) {
        return error::error_t::OUT_OF_RESOURCE;
    }
    return ::m5::variants::frameworks::espidf::detail::mapEspErrCommon(err, error::error_t::IO_ERROR);
}

TickType_t ticks(uint32_t timeout_ms)
{
    return detail::timeoutMsToTicks(timeout_ms);
}
}  // namespace

bool Bus_espidf::onRecvCallback(i2s_chan_handle_t, i2s_event_data_t* event, void* user_ctx)
{
    if (event == nullptr || user_ctx == nullptr) {
        return false;
    }
    auto* self       = static_cast<Bus_espidf*>(user_ctx);
    const size_t cap = self->_dma_capacity;
    size_t current   = self->_dma_available.load(std::memory_order_relaxed);
    for (;;) {
        size_t next = current + event->size;
        if (next > cap) {
            next = cap;
        }
        if (self->_dma_available.compare_exchange_weak(current, next, std::memory_order_relaxed)) {
            break;
        }
    }
    return false;
}

result_t<void> Bus_espidf::init(const IBusConfig& config)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (config.pin_clk < 0 || config.pin_din < 0 || config.rx_buffer_size == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto reset = resetForInitialization();
    if (!reset.has_value()) {
        return reset;
    }
    _config = config;
    return markInitializationSucceeded(false);
}

Bus_espidf::~Bus_espidf()
{
    (void)teardownBackend();
}

bus::CloseOutcome Bus_espidf::closeBackend(void)
{
    return teardownBackend();
}

result_t<void> Bus_espidf::beginOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    if (context.runtime.mode != bus::OperationMode::Rx) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return ensureChannel(context.config);
}

result_t<void> Bus_espidf::endOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    return {};
}

bus::CloseOutcome Bus_espidf::teardownBackend(void)
{
    if (_rx_handle != nullptr) {
        const esp_err_t disabled = i2s_channel_disable(_rx_handle);
        if (disabled != ESP_OK && disabled != ESP_ERR_INVALID_STATE) {
            return bus::CloseOutcome::partialOrUnknown(mapEspErr(disabled));
        }
        const esp_err_t deleted = i2s_del_channel(_rx_handle);
        if (deleted != ESP_OK) {
            return bus::CloseOutcome::partialOrUnknown(mapEspErr(deleted));
        }
        _rx_handle = nullptr;
        if (_config.pin_clk >= 0) {
            (void)gpio_reset_pin(static_cast<gpio_num_t>(_config.pin_clk));
        }
        if (_config.pin_din >= 0) {
            (void)gpio_reset_pin(static_cast<gpio_num_t>(_config.pin_din));
        }
    }
    detail_espidf_i2s_controller::release(_controller);
    _controller   = -1;
    _configured   = false;
    _dma_capacity = 0;
    _dma_available.store(0, std::memory_order_relaxed);
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

result_t<void> Bus_espidf::ensureChannel(const AccessConfig& cfg)
{
    if (cfg.sample_rate_hz == 0 || cfg.bits_per_sample != 16 || cfg.channels != 1) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (_configured && cfg.sample_rate_hz == _applied_cfg.sample_rate_hz) {
        return {};
    }
    auto reset = resetForInitialization();
    if (!reset.has_value()) {
        return reset;
    }

    _controller = detail_espidf_i2s_controller::claimPdm();
    if (_controller < 0) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    constexpr uint32_t kFramesPerDescriptor = 120;
    constexpr uint32_t kMinDescriptors      = 2;
    constexpr uint32_t kMaxDescriptors      = 128;
    constexpr size_t kFrameBytes            = 2;
    const size_t bytes_per_descriptor       = kFramesPerDescriptor * kFrameBytes;
    uint32_t descriptor_count =
        static_cast<uint32_t>((_config.rx_buffer_size + bytes_per_descriptor - 1) / bytes_per_descriptor);
    if (descriptor_count < kMinDescriptors) {
        descriptor_count = kMinDescriptors;
    }
    if (descriptor_count > kMaxDescriptors) {
        descriptor_count = kMaxDescriptors;
    }

    const auto port_id            = static_cast<decltype(i2s_chan_config_t{}.id)>(_controller);
    i2s_chan_config_t channel_cfg = I2S_CHANNEL_DEFAULT_CONFIG(port_id, I2S_ROLE_MASTER);
    channel_cfg.dma_desc_num      = descriptor_count;
    channel_cfg.dma_frame_num     = kFramesPerDescriptor;
    _dma_capacity                 = static_cast<size_t>(descriptor_count) * kFramesPerDescriptor * kFrameBytes;

    esp_err_t err = i2s_new_channel(&channel_cfg, nullptr, &_rx_handle);
    if (err != ESP_OK) {
        return failAfterSetup(mapEspErr(err));
    }

    i2s_pdm_rx_config_t pdm_cfg = {};
    pdm_cfg.clk_cfg             = I2S_PDM_RX_CLK_DEFAULT_CONFIG(cfg.sample_rate_hz);
    pdm_cfg.slot_cfg            = I2S_PDM_RX_SLOT_PCM_FMT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
    pdm_cfg.gpio_cfg.clk        = static_cast<gpio_num_t>(_config.pin_clk);
    pdm_cfg.gpio_cfg.din        = static_cast<gpio_num_t>(_config.pin_din);
    pdm_cfg.gpio_cfg.invert_flags.clk_inv = false;
    err                                   = i2s_channel_init_pdm_rx_mode(_rx_handle, &pdm_cfg);
    if (err != ESP_OK) {
        return failAfterSetup(mapEspErr(err));
    }

    i2s_event_callbacks_t callbacks = {};
    callbacks.on_recv               = &Bus_espidf::onRecvCallback;
    err                             = i2s_channel_register_event_callback(_rx_handle, &callbacks, this);
    if (err == ESP_OK) {
        err = i2s_channel_enable(_rx_handle);
    }
    if (err != ESP_OK) {
        return failAfterSetup(mapEspErr(err));
    }
    _applied_cfg = cfg;
    _configured  = true;
    return {};
}

result_t<size_t> Bus_espidf::readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len)
{
    const auto& cfg = context.config;
    if (len != 0 && dst == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (_rx_handle == nullptr || !_configured) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    size_t done = 0;
    while (done < len && dst != nullptr && !dst->closed()) {
        auto span = dst->reserve(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        const size_t request = span->size & ~size_t{1};
        if (request == 0) {
            break;
        }
        size_t captured = 0;
        esp_err_t err   = i2s_channel_read(_rx_handle, span->data, request, &captured, ticks(cfg.read_timeout_ms));
        if (captured > request || (captured & 1) != 0) {
            return m5::stl::make_unexpected(error::error_t::IO_ERROR);
        }
        if (captured != 0) {
            auto committed = dst->commit(captured);
            if (!committed.has_value()) {
                return m5::stl::make_unexpected(committed.error());
            }
            size_t current = _dma_available.load(std::memory_order_relaxed);
            while (!_dma_available.compare_exchange_weak(current, (captured < current) ? current - captured : 0,
                                                         std::memory_order_relaxed)) {
            }
            done += captured;
        }
        if (err == ESP_ERR_TIMEOUT || captured == 0) {
            break;
        }
        if (err != ESP_OK) {
            return (done != 0) ? result_t<size_t>{done} : m5::stl::make_unexpected(mapEspErr(err));
        }
    }
    return done;
}

result_t<size_t> Bus_espidf::readableBytesBackend(bus::OperationContext<AccessConfig>& context)
{
    (void)context;
    if (_rx_handle == nullptr || !_configured) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return _dma_available.load(std::memory_order_relaxed) & ~size_t{1};
}

}  // namespace m5::hal::v2::pdm

#endif
#endif
