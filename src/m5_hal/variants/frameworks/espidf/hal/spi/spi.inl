// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_INL

#include "spi.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_MASTER

#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace m5::hal::v2::spi {

namespace {
namespace impl_espidf {

struct DataChunk {
    size_t tx = 0;
    size_t rx = 0;
};

bool chunkUsesBothBuffers(const DataChunk& chunk)
{
    return chunk.tx > 0 && chunk.rx > 0;
}

int rxBufferIndexForChunk(const DataChunk& chunk, int tx_idx)
{
    return chunkUsesBothBuffers(chunk) ? (tx_idx ^ 1) : tx_idx;
}

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
        default:
            return error::error_t::IO_ERROR;
    }
}

bool isHalfDuplexMode(spi::spi_data_mode_t mode)
{
    using spi::spi_data_mode_t;
    return mode == spi_data_mode_t::HalfDuplex || mode == spi_data_mode_t::HalfDuplexWithDcPin ||
           mode == spi_data_mode_t::HalfDuplexWithDcBit || mode == spi_data_mode_t::DualOutput ||
           mode == spi_data_mode_t::DualIo || mode == spi_data_mode_t::QuadOutput || mode == spi_data_mode_t::QuadIo ||
           mode == spi_data_mode_t::OctalOutput || mode == spi_data_mode_t::OctalIo;
}

bool isSingleLaneHalfDuplexMode(spi::spi_data_mode_t mode)
{
    using spi::spi_data_mode_t;
    return mode == spi_data_mode_t::HalfDuplex || mode == spi_data_mode_t::HalfDuplexWithDcPin ||
           mode == spi_data_mode_t::HalfDuplexWithDcBit;
}

void setPinLevel(types::gpio_number_t pin, bool level)
{
    if (pin >= 0) {
        (void)::gpio_set_level(static_cast<::gpio_num_t>(pin), level ? 1 : 0);
    }
}

void setPinOutput(types::gpio_number_t pin, bool level)
{
    if (pin >= 0) {
        // INPUT_OUTPUT keeps the driven level readable through GPIO input,
        // matching the espidf GPIO variant's diagnostics convention.
        (void)::gpio_set_direction(static_cast<::gpio_num_t>(pin), GPIO_MODE_INPUT_OUTPUT);
        setPinLevel(pin, level);
    }
}

void setDC(types::gpio_number_t dc_pin, int8_t level)
{
    if (level >= 0 && dc_pin >= 0) {
        setPinLevel(dc_pin, level != 0);
    }
}

uint8_t metaByte(uint32_t value, uint8_t index, uint8_t bytes)
{
    const uint8_t shift = static_cast<uint8_t>((bytes - 1u - index) * 8u);
    return static_cast<uint8_t>(value >> shift);
}

size_t minSize(size_t a, size_t b)
{
    return a < b ? a : b;
}

result_t<DataChunk> prepareChunk(data::Source* src, size_t& tx_remaining, data::Sink* dst, size_t& rx_remaining,
                                 uint8_t* dma_buf, size_t max_len, bool sequential)
{
    data::ConstDataSpan tx_span{};
    const bool can_tx = src != nullptr && tx_remaining > 0 && !src->eof();
    const bool can_rx = dst != nullptr && rx_remaining > 0 && !dst->closed();
    if (can_tx) {
        auto peeked = src->peek(minSize(tx_remaining, max_len));
        if (!peeked.has_value()) {
            return m5::stl::make_unexpected(peeked.error());
        }
        tx_span = peeked.value().first(minSize(tx_remaining, max_len));
    }

    size_t tx_chunk = 0;
    size_t rx_chunk = 0;
    if (can_tx && can_rx && !sequential) {
        const size_t common = minSize(tx_span.size, minSize(rx_remaining, max_len));
        tx_chunk            = common;
        rx_chunk            = common;
    } else if (can_tx) {
        tx_chunk = minSize(tx_span.size, max_len);
    } else if (can_rx) {
        rx_chunk = minSize(rx_remaining, max_len);
    }

    if (tx_chunk > 0) {
        std::memcpy(dma_buf, tx_span.data, tx_chunk);
        auto advanced = src->advance(tx_chunk);
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        tx_remaining -= tx_chunk;
    }
    if (rx_chunk > 0) {
        rx_remaining -= rx_chunk;
    }
    return DataChunk{tx_chunk, rx_chunk};
}

result_t<void> completeChunk(data::Sink* dst, uint8_t* dma_buf, const DataChunk& chunk, bus::TransferTotals& totals)
{
    if (chunk.tx > 0) {
        totals.tx += chunk.tx;
    }
    if (chunk.rx > 0) {
        auto reserved = dst->reserve(chunk.rx);
        if (!reserved.has_value()) {
            return m5::stl::make_unexpected(reserved.error());
        }
        if (reserved.value().size < chunk.rx) {
            return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
        }
        std::memcpy(reserved.value().data, dma_buf, chunk.rx);
        auto committed = dst->commit(chunk.rx);
        if (!committed.has_value()) {
            return m5::stl::make_unexpected(committed.error());
        }
        totals.rx += chunk.rx;
    }
    return {};
}

result_t<void> startChunk(::spi_device_handle_t device, ::spi_transaction_t& trans, uint8_t* tx_buf, uint8_t* rx_buf,
                          const DataChunk& chunk)
{
    std::memset(&trans, 0, sizeof(trans));
    trans.tx_buffer = chunk.tx > 0 ? tx_buf : nullptr;
    trans.rx_buffer = chunk.rx > 0 ? rx_buf : nullptr;
    trans.length    = (chunk.tx > 0 ? chunk.tx : chunk.rx) * 8u;
    trans.rxlength  = chunk.rx * 8u;

    auto mapped = mapEspErr(::spi_device_polling_start(device, &trans, portMAX_DELAY));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return {};
}

result_t<void> endChunk(::spi_device_handle_t device)
{
    auto mapped = mapEspErr(::spi_device_polling_end(device, portMAX_DELAY));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return {};
}

}  // namespace impl_espidf
}  // namespace

void Bus_espidf::workerEntry(void* arg)
{
    static_cast<Bus_espidf*>(arg)->workerLoop();
}

void Bus_espidf::workerLoop(void)
{
    while (true) {
        (void)::ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int front_tx_idx = 0;
        impl_espidf::DataChunk front{_worker_front_tx_len, _worker_front_rx_len};
        int front_rx_idx = impl_espidf::rxBufferIndexForChunk(front, front_tx_idx);
        auto worker_err  = error::error_t::OK;

        while (_in_flight.load(std::memory_order_relaxed)) {
            auto prepare_error = error::error_t::OK;
            impl_espidf::DataChunk next{};
            const int next_tx_idx      = front_tx_idx ^ 1;
            const bool front_uses_both = impl_espidf::chunkUsesBothBuffers(front);
            if (!front_uses_both) {
                auto prepared =
                    impl_espidf::prepareChunk(_worker_src, _worker_tx_remaining, _worker_dst, _worker_rx_remaining,
                                              _dma_buf[next_tx_idx], kMaxDmaChunk, _worker_half_duplex);
                if (prepared.has_value()) {
                    next = prepared.value();
                } else {
                    prepare_error = prepared.error();
                }
            }

            auto ended = impl_espidf::endChunk(_device);
            if (!ended.has_value() && worker_err == error::error_t::OK) {
                worker_err = ended.error();
            }

            if (worker_err == error::error_t::OK) {
                auto completed =
                    impl_espidf::completeChunk(_worker_dst, _dma_buf[front_rx_idx], front, _transfer_totals);
                if (!completed.has_value()) {
                    worker_err = completed.error();
                }
            }

            if (front_uses_both && worker_err == error::error_t::OK) {
                auto prepared =
                    impl_espidf::prepareChunk(_worker_src, _worker_tx_remaining, _worker_dst, _worker_rx_remaining,
                                              _dma_buf[next_tx_idx], kMaxDmaChunk, _worker_half_duplex);
                if (prepared.has_value()) {
                    next = prepared.value();
                } else {
                    prepare_error = prepared.error();
                }
            }

            if (prepare_error != error::error_t::OK && worker_err == error::error_t::OK) {
                worker_err = prepare_error;
            }

            if (worker_err != error::error_t::OK) {
                _in_flight.store(false, std::memory_order_relaxed);
                break;
            }

            if (next.tx == 0 && next.rx == 0) {
                _in_flight.store(false, std::memory_order_relaxed);
                break;
            }

            const int next_rx_idx = impl_espidf::rxBufferIndexForChunk(next, next_tx_idx);
            auto started          = impl_espidf::startChunk(_device, _dma_trans[next_tx_idx], _dma_buf[next_tx_idx],
                                                            _dma_buf[next_rx_idx], next);
            if (!started.has_value()) {
                worker_err = started.error();
                _in_flight.store(false, std::memory_order_relaxed);
                break;
            }

            front        = next;
            front_tx_idx = next_tx_idx;
            front_rx_idx = next_rx_idx;
            _worker_remaining =
                _worker_tx_remaining > _worker_rx_remaining ? _worker_tx_remaining : _worker_rx_remaining;
        }

        _worker_src          = nullptr;
        _worker_dst          = nullptr;
        _worker_remaining    = 0;
        _worker_tx_remaining = 0;
        _worker_rx_remaining = 0;
        _worker_front_tx_len = 0;
        _worker_front_rx_len = 0;
        _worker_half_duplex  = false;
        // Publish every worker-side state update through one release store.
        // _worker_active remains true until the consumer observes this terminal
        // status, so an inactive+ASYNC_RUNNING state only denotes the synchronous
        // front chunk and can never be mistaken for worker completion.
        _worker_status.store(worker_err, std::memory_order_release);
    }
}

void Bus_espidf::waitInFlight(void)
{
    const auto worker_status = _worker_status.load(std::memory_order_acquire);
    if (worker_status != error::error_t::ASYNC_RUNNING) {
        _in_flight.store(false, std::memory_order_relaxed);
        _worker_active.store(false, std::memory_order_relaxed);
        _transfer_owner = nullptr;
        return;
    }
    if (_worker_active.load(std::memory_order_relaxed)) {
        while (_worker_status.load(std::memory_order_acquire) == error::error_t::ASYNC_RUNNING) {
            ::taskYIELD();
        }
        _in_flight.store(false, std::memory_order_relaxed);
        _worker_active.store(false, std::memory_order_relaxed);
        _transfer_owner = nullptr;
        return;
    }
    if (!_in_flight.load(std::memory_order_relaxed)) {
        return;
    }

    (void)impl_espidf::endChunk(_device);
    _in_flight.store(false, std::memory_order_relaxed);
    _transfer_owner = nullptr;
    _worker_status.store(error::error_t::OK, std::memory_order_relaxed);
    _worker_active.store(false, std::memory_order_relaxed);
    _worker_src       = nullptr;
    _worker_dst       = nullptr;
    _worker_remaining = 0;
}

void Bus_espidf::stopWorker(void)
{
    if (_worker_task != nullptr) {
        ::vTaskDelete(_worker_task);
        _worker_task = nullptr;
    }
}

void Bus_espidf::freeDmaBuffers(void)
{
    if (_dma_buf[0] != nullptr) {
        ::heap_caps_free(_dma_buf[0]);
        _dma_buf[0] = nullptr;
    }
    if (_dma_buf[1] != nullptr) {
        ::heap_caps_free(_dma_buf[1]);
        _dma_buf[1] = nullptr;
    }
}

Bus_espidf::~Bus_espidf()
{
    auto released = release();
    if (!released.has_value()) {
        // release() deliberately preserves the worker on driver teardown
        // failure so an explicit caller does not lose a still-adopted backend.
        // Destruction cannot preserve it: the task holds `this`, so stop it to
        // avoid use-after-free. DMA remains allocated while a device may still
        // be registered because the driver can retain transaction references.
        stopWorker();
        if (_device == nullptr) {
            freeDmaBuffers();
        }
    }
}

error::error_t Bus_espidf::attach(::spi_host_device_t host, int8_t claimed_controller)
{
    if (!detail_espidf_spi::attachedControllerMatches(static_cast<int>(host), claimed_controller,
                                                      hardwareControllerCountForSPI(), static_cast<int>(SPI2_HOST))) {
        return error::error_t::INVALID_ARGUMENT;
    }
    // Re-entry (attach called again while device/worker/DMA buffers from a
    // prior attach() or init() are still held) tears down first so nothing
    // is silently overwritten and orphaned.
    auto released = release();
    if (!released.has_value()) {
        return released.error();
    }
    _host       = host;
    _owns_bus   = false;
    _dma_buf[0] = static_cast<uint8_t*>(::heap_caps_aligned_alloc(4, kMaxDmaChunk, MALLOC_CAP_DMA));
    _dma_buf[1] = static_cast<uint8_t*>(::heap_caps_aligned_alloc(4, kMaxDmaChunk, MALLOC_CAP_DMA));
    if (_dma_buf[0] == nullptr || _dma_buf[1] == nullptr) {
        if (_dma_buf[0] != nullptr) {
            ::heap_caps_free(_dma_buf[0]);
            _dma_buf[0] = nullptr;
        }
        if (_dma_buf[1] != nullptr) {
            ::heap_caps_free(_dma_buf[1]);
            _dma_buf[1] = nullptr;
        }
        return error::error_t::OUT_OF_RESOURCE;
    }
    auto created = ::xTaskCreatePinnedToCore(workerEntry, "m5hal-spi", 4096, this, configMAX_PRIORITIES - 2,
                                             &_worker_task, xPortGetCoreID());
    if (created != pdPASS) {
        ::heap_caps_free(_dma_buf[0]);
        _dma_buf[0] = nullptr;
        ::heap_caps_free(_dma_buf[1]);
        _dma_buf[1] = nullptr;
        return error::error_t::OUT_OF_RESOURCE;
    }
    return error::error_t::OK;
}

result_t<void> Bus_espidf::init(const BusConfig_espidf& config)
{
    // Re-entry tears down whatever the previous attach()/init() left behind
    // regardless of ownership form, so the DMA buffers and worker task below
    // are never allocated on top of still-live resources.
    auto released = release();
    if (!released.has_value()) {
        return m5::stl::make_unexpected(released.error());
    }
    _config = config;
    _host   = config.host;

    _dma_buf[0] = static_cast<uint8_t*>(::heap_caps_aligned_alloc(4, kMaxDmaChunk, MALLOC_CAP_DMA));
    _dma_buf[1] = static_cast<uint8_t*>(::heap_caps_aligned_alloc(4, kMaxDmaChunk, MALLOC_CAP_DMA));
    if (_dma_buf[0] == nullptr || _dma_buf[1] == nullptr) {
        (void)release();
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    ::spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num        = static_cast<int>(_config.pin_mosi);
    bus_config.miso_io_num        = static_cast<int>(_config.pin_miso);
    bus_config.sclk_io_num        = static_cast<int>(_config.pin_clk);
    bus_config.quadwp_io_num      = static_cast<int>(_config.pin_d2);
    bus_config.quadhd_io_num      = static_cast<int>(_config.pin_d3);
    bus_config.data4_io_num       = static_cast<int>(_config.pin_d4);
    bus_config.data5_io_num       = static_cast<int>(_config.pin_d5);
    bus_config.data6_io_num       = static_cast<int>(_config.pin_d6);
    bus_config.data7_io_num       = static_cast<int>(_config.pin_d7);
    bus_config.max_transfer_sz    = kMaxDmaChunk;

    auto mapped = impl_espidf::mapEspErr(::spi_bus_initialize(_host, &bus_config, SPI_DMA_CH_AUTO));
    if (error::isError(mapped)) {
        (void)release();
        return m5::stl::make_unexpected(mapped);
    }
    _owns_bus = true;

    if (_config.pin_dc >= 0) {
        impl_espidf::setPinOutput(_config.pin_dc, true);
    }

    auto created = ::xTaskCreatePinnedToCore(workerEntry, "m5hal-spi", 4096, this, configMAX_PRIORITIES - 2,
                                             &_worker_task, xPortGetCoreID());
    if (created != pdPASS) {
        (void)release();
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    return {};
}

result_t<void> Bus_espidf::release(void)
{
    waitInFlight();

    const auto teardown = detail_espidf_spi::releaseDriverBeforeWorker(
        _owns_bus, ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0),
        [this]() {
            auto removed = removeDevice();
            return removed.has_value() ? error::error_t::OK : removed.error();
        },
        [this]() { return impl_espidf::mapEspErr(::spi_bus_free(_host)); }, [this]() { stopWorker(); });
    if (teardown.bus_released) {
        _owns_bus = false;
    }

    if (teardown.worker_stopped) {
        freeDmaBuffers();
        _transaction_active = false;
        _transfer_totals.clear();
        _transfer_owner = nullptr;
    }

    if (error::isError(teardown.error)) {
        return m5::stl::make_unexpected(teardown.error);
    }
    return {};
}

result_t<void> Bus_espidf::removeDevice(void)
{
    waitInFlight();

    if (_device == nullptr) {
        return {};
    }
    auto mapped = impl_espidf::mapEspErr(::spi_bus_remove_device(_device));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    _device           = nullptr;
    _device_freq      = 0;
    _device_mode      = 0;
    _device_order     = 0;
    _device_data_mode = spi::spi_data_mode_t::FullDuplex;
    return {};
}

result_t<bool> Bus_espidf::ensureDevice(const spi::MasterAccessConfig& cfg)
{
    if (_device != nullptr && _device_freq == cfg.freq && _device_mode == cfg.spi_mode &&
        _device_order == cfg.spi_order && _device_data_mode == cfg.spi_data_mode) {
        return false;
    }

    auto removed = removeDevice();
    if (!removed.has_value()) {
        return m5::stl::make_unexpected(removed.error());
    }

    ::spi_device_interface_config_t dev_config = {};
    dev_config.clock_speed_hz                  = static_cast<int>(cfg.freq);
    dev_config.mode                            = cfg.spi_mode & 0x03;
    dev_config.spics_io_num                    = -1;
    dev_config.queue_size                      = 1;
    const bool half_duplex                     = impl_espidf::isHalfDuplexMode(cfg.spi_data_mode);
    // Without this flag ESP-IDF limits the clock to 26.66 MHz.
    dev_config.flags = SPI_DEVICE_NO_DUMMY;
    if (half_duplex) {
        dev_config.flags |= SPI_DEVICE_HALFDUPLEX;
    }
    if (_config.pin_miso < 0 && impl_espidf::isSingleLaneHalfDuplexMode(cfg.spi_data_mode)) {
        // ESP-IDF single-I/O mode routes the MOSI (SPID) signal back as
        // input during the receive phase.
        dev_config.flags |= SPI_DEVICE_3WIRE;
    }
    if (cfg.spi_order != 0) {
        dev_config.flags |= SPI_DEVICE_BIT_LSBFIRST;
    }

    auto mapped = impl_espidf::mapEspErr(::spi_bus_add_device(_host, &dev_config, &_device));
    if (error::isError(mapped)) {
        _device = nullptr;
        return m5::stl::make_unexpected(mapped);
    }

    _device_freq      = cfg.freq;
    _device_mode      = cfg.spi_mode;
    _device_order     = cfg.spi_order;
    _device_data_mode = cfg.spi_data_mode;
    return true;
}

result_t<void> Bus_espidf::beginTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    (void)owner;
    waitInFlight();
    if (cfg.freq == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    auto dev = ensureDevice(cfg);
    if (!dev.has_value()) {
        return m5::stl::make_unexpected(dev.error());
    }
    if (dev.value()) {
        ::spi_transaction_t settle = {};
        settle.length              = 1;
        auto mapped                = impl_espidf::mapEspErr(::spi_device_polling_transmit(_device, &settle));
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
    }

    _transaction_active = true;
    impl_espidf::setPinOutput(cfg.pin_cs, false);
    const auto dc_pin = cfg.pin_dc >= 0 ? cfg.pin_dc : _config.pin_dc;
    impl_espidf::setDC(dc_pin, 1);
    return {};
}

result_t<void> Bus_espidf::endTransaction(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    (void)owner;
    waitInFlight();
    impl_espidf::setPinLevel(cfg.pin_cs, true);
    _transaction_active = false;
    return {};
}

result_t<void> Bus_espidf::transfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg,
                                    const spi::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                    size_t rx_len)
{
    waitInFlight();
    const auto worker_status = _worker_status.load(std::memory_order_acquire);
    if (error::isError(worker_status)) {
        const auto err = worker_status;
        _worker_status.store(error::error_t::OK, std::memory_order_relaxed);
        return m5::stl::make_unexpected(err);
    }
    if (!_transaction_active || cfg.freq == 0 || _device == nullptr || desc.command_bytes > 4 ||
        desc.address_bytes > 4) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (rx_len > 0 && _config.pin_miso < 0 &&
        (!impl_espidf::isSingleLaneHalfDuplexMode(cfg.spi_data_mode) || _config.pin_mosi < 0)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    auto dev = ensureDevice(cfg);
    if (!dev.has_value()) {
        return m5::stl::make_unexpected(dev.error());
    }

    const types::gpio_number_t dc_pin = cfg.pin_dc >= 0 ? cfg.pin_dc : _config.pin_dc;
    if (cfg.pin_dc >= 0 && cfg.pin_dc != _last_acc_dc) {
        impl_espidf::setPinOutput(cfg.pin_dc, true);
        _last_acc_dc = cfg.pin_dc;
    }

    const bool has_phase_dc = desc.command_dc_level >= 0 || desc.address_dc_level >= 0 || desc.data_dc_level >= 0;
    if (!has_phase_dc && desc.dc_level_valid) {
        impl_espidf::setDC(dc_pin, desc.dc_level ? 1 : 0);
    }

    if (desc.command_bytes > 0) {
        impl_espidf::setDC(dc_pin, desc.command_dc_level);
        ::spi_transaction_t trans = {};
        trans.flags               = SPI_TRANS_USE_TXDATA;
        trans.length              = desc.command_bytes * 8u;
        for (uint8_t i = 0; i < desc.command_bytes; ++i) {
            trans.tx_data[i] = impl_espidf::metaByte(desc.command, i, desc.command_bytes);
        }
        auto mapped = impl_espidf::mapEspErr(::spi_device_polling_transmit(_device, &trans));
        impl_espidf::setDC(dc_pin, 1);
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
    }

    if (desc.address_bytes > 0) {
        impl_espidf::setDC(dc_pin, desc.address_dc_level);
        ::spi_transaction_t trans = {};
        trans.flags               = SPI_TRANS_USE_TXDATA;
        trans.length              = desc.address_bytes * 8u;
        for (uint8_t i = 0; i < desc.address_bytes; ++i) {
            trans.tx_data[i] = impl_espidf::metaByte(desc.address, i, desc.address_bytes);
        }
        auto mapped = impl_espidf::mapEspErr(::spi_device_polling_transmit(_device, &trans));
        impl_espidf::setDC(dc_pin, 1);
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
    }

    if (desc.dummy_cycles > 0) {
        ::spi_transaction_t trans = {};
        trans.length              = desc.dummy_cycles;
        auto mapped               = impl_espidf::mapEspErr(::spi_device_polling_transmit(_device, &trans));
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
    }

    if (tx_len == 0 && rx_len == 0) {
        return {};
    }

    impl_espidf::setDC(dc_pin, desc.data_dc_level);
    if (has_phase_dc && desc.data_dc_level < 0) {
        impl_espidf::setDC(dc_pin, 1);
    }

    size_t tx_remaining    = tx_len;
    size_t rx_remaining    = rx_len;
    const bool half_duplex = impl_espidf::isSingleLaneHalfDuplexMode(cfg.spi_data_mode);
    auto first =
        impl_espidf::prepareChunk(src, tx_remaining, dst, rx_remaining, _dma_buf[0], kMaxDmaChunk, half_duplex);
    if (!first.has_value()) {
        return m5::stl::make_unexpected(first.error());
    }
    if (first.value().tx == 0 && first.value().rx == 0) {
        return {};
    }

    uint8_t* rx_buf = _dma_buf[impl_espidf::rxBufferIndexForChunk(first.value(), 0)];
    auto started    = impl_espidf::startChunk(_device, _dma_trans[0], _dma_buf[0], rx_buf, first.value());
    if (!started.has_value()) {
        return m5::stl::make_unexpected(started.error());
    }

    const size_t remaining = tx_remaining > rx_remaining ? tx_remaining : rx_remaining;
    _worker_active.store(false, std::memory_order_relaxed);
    _in_flight.store(true, std::memory_order_relaxed);
    _worker_status.store(error::error_t::ASYNC_RUNNING, std::memory_order_relaxed);
    _transfer_owner = owner;

    if (remaining == 0) {
        auto ended = impl_espidf::endChunk(_device);
        _in_flight.store(false, std::memory_order_relaxed);
        _worker_status.store(error::error_t::OK, std::memory_order_relaxed);
        _transfer_owner = nullptr;
        if (!ended.has_value()) {
            return m5::stl::make_unexpected(ended.error());
        }

        auto completed = impl_espidf::completeChunk(dst, rx_buf, first.value(), _transfer_totals);
        if (!completed.has_value()) {
            return m5::stl::make_unexpected(completed.error());
        }
        return {};
    }

    if (_worker_task == nullptr) {
        // A missing worker cannot service a multi-chunk transfer. Drain the
        // already-started first chunk and fail loudly instead of notifying a
        // null task handle.
        auto ended = impl_espidf::endChunk(_device);
        _in_flight.store(false, std::memory_order_relaxed);
        _worker_status.store(error::error_t::OK, std::memory_order_relaxed);
        _transfer_owner = nullptr;
        if (!ended.has_value()) {
            return m5::stl::make_unexpected(ended.error());
        }
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    _worker_src          = src;
    _worker_dst          = dst;
    _worker_tx_remaining = tx_remaining;
    _worker_rx_remaining = rx_remaining;
    _worker_remaining    = remaining;
    _worker_front_tx_len = first.value().tx;
    _worker_front_rx_len = first.value().rx;
    _worker_half_duplex  = half_duplex;
    _worker_active.store(true, std::memory_order_relaxed);
    ::xTaskNotifyGive(_worker_task);
    return {};
}

result_t<bus::TransferTotals> Bus_espidf::waitTransfer(bus::IAccessor* owner, const spi::MasterAccessConfig& cfg)
{
    (void)cfg;
    auto worker_status = _worker_status.load(std::memory_order_acquire);
    if ((_in_flight.load(std::memory_order_relaxed) || worker_status == error::error_t::ASYNC_RUNNING) &&
        _transfer_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }

    if (_in_flight.load(std::memory_order_relaxed) && !_worker_active.load(std::memory_order_relaxed) &&
        worker_status == error::error_t::ASYNC_RUNNING) {
        auto ended = impl_espidf::endChunk(_device);
        _in_flight.store(false, std::memory_order_relaxed);
        if (!ended.has_value()) {
            _transfer_owner = nullptr;
            return m5::stl::make_unexpected(ended.error());
        }
    }

    if (worker_status == error::error_t::ASYNC_RUNNING && _worker_active.load(std::memory_order_relaxed)) {
        do {
            ::taskYIELD();
            worker_status = _worker_status.load(std::memory_order_acquire);
        } while (worker_status == error::error_t::ASYNC_RUNNING);
        _in_flight.store(false, std::memory_order_relaxed);
    }

    worker_status = _worker_status.load(std::memory_order_acquire);
    if (worker_status != error::error_t::ASYNC_RUNNING) {
        _in_flight.store(false, std::memory_order_relaxed);
        _worker_active.store(false, std::memory_order_relaxed);
    }
    if (error::isError(worker_status)) {
        const auto err = worker_status;
        _worker_status.store(error::error_t::OK, std::memory_order_relaxed);
        _transfer_totals.clear();
        _transfer_owner = nullptr;
        return m5::stl::make_unexpected(err);
    }

    auto totals = _transfer_totals;
    _transfer_totals.clear();
    _transfer_owner = nullptr;
    return totals;
}

bool Bus_espidf::transferBusy(bus::IAccessor* owner)
{
    return _worker_status.load(std::memory_order_acquire) == error::error_t::ASYNC_RUNNING && _transfer_owner == owner;
}

}  // namespace m5::hal::v2::spi

#endif

#endif
