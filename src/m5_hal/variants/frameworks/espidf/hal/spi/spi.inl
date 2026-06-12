#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_SPI_SPI_INL

#include "spi.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_SPI_HAS_MASTER

#include <driver/gpio.h>
#include <esp_err.h>

#include <cstddef>

namespace m5::variants::frameworks::espidf::hal::v1::spi {

namespace {

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

bool isHalfDuplexMode(::m5::hal::v1::spi::spi_data_mode_t mode)
{
    using ::m5::hal::v1::spi::spi_data_mode_t;
    return mode == spi_data_mode_t::spi_halfduplex || mode == spi_data_mode_t::spi_halfduplex_with_dc_pin ||
           mode == spi_data_mode_t::spi_halfduplex_with_dc_bit || mode == spi_data_mode_t::spi_dual_output ||
           mode == spi_data_mode_t::spi_dual_io || mode == spi_data_mode_t::spi_quad_output ||
           mode == spi_data_mode_t::spi_quad_io || mode == spi_data_mode_t::spi_octal_output ||
           mode == spi_data_mode_t::spi_octal_io;
}

bool descNeedsHalfDuplex(const ::m5::hal::v1::spi::TransferDesc& desc)
{
    return desc.command_bytes > 0 || desc.address_bytes > 0 || desc.dummy_cycles > 0;
}

void setPinLevel(::m5::hal::v1::types::gpio_number_t pin, bool level)
{
    if (pin >= 0) {
        (void)::gpio_set_level(static_cast<::gpio_num_t>(pin), level ? 1 : 0);
    }
}

void setPinOutput(::m5::hal::v1::types::gpio_number_t pin, bool level)
{
    if (pin >= 0) {
        (void)::gpio_set_direction(static_cast<::gpio_num_t>(pin), GPIO_MODE_OUTPUT);
        setPinLevel(pin, level);
    }
}

void setDC(const ::m5::hal::v1::spi::SPIBusConfig& bus_cfg, int8_t level)
{
    if (level >= 0) {
        setPinLevel(bus_cfg.pin_dc, level != 0);
    }
}

uint8_t metaByte(uint32_t value, uint8_t remaining)
{
    const uint8_t shift = static_cast<uint8_t>((remaining - 1u) * 8u);
    return static_cast<uint8_t>(value >> shift);
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> transmit(::spi_device_handle_t device, const void* tx_buffer,
                                                                void* rx_buffer, size_t tx_len, size_t rx_len)
{
    ::spi_transaction_t trans = {};
    trans.tx_buffer           = tx_buffer;
    trans.rx_buffer           = rx_buffer;
    trans.length              = tx_len * 8u;
    trans.rxlength            = rx_len * 8u;
    if (tx_len == 0) {
        trans.length = rx_len * 8u;
    }

    auto mapped = mapEspErr(::spi_device_polling_transmit(device, &trans));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> sendMeta(::spi_device_handle_t device, uint32_t value,
                                                                uint8_t bytes)
{
    if (bytes > 4) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    while (bytes > 0) {
        const uint8_t byte = metaByte(value, bytes);
        auto result        = transmit(device, &byte, nullptr, sizeof(byte), 0);
        if (!result.has_value()) {
            return result;
        }
        --bytes;
    }
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> sendDummy(::spi_device_handle_t device, uint8_t cycles)
{
    if (cycles == 0) {
        return {};
    }

    ::spi_transaction_ext_t trans = {};
    trans.base.flags              = SPI_TRANS_VARIABLE_DUMMY;
    trans.dummy_bits              = cycles;
    auto mapped                   = mapEspErr(::spi_device_polling_transmit(device, &trans.base));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> transferChunk(::spi_device_handle_t device,
                                                                     ::m5::hal::v1::data::ConstDataSpan tx_span,
                                                                     ::m5::hal::v1::data::DataSpan rx_span)
{
    const size_t common = (tx_span.size < rx_span.size) ? tx_span.size : rx_span.size;
    if (common > 0) {
        auto result = transmit(device, tx_span.data, rx_span.data, common, common);
        if (!result.has_value()) {
            return result;
        }
    }
    if (tx_span.size > common) {
        auto result = transmit(device, tx_span.data + common, nullptr, tx_span.size - common, 0);
        if (!result.has_value()) {
            return result;
        }
    }
    if (rx_span.size > common) {
        auto result = transmit(device, nullptr, rx_span.data + common, 0, rx_span.size - common);
        if (!result.has_value()) {
            return result;
        }
    }
    return {};
}

}  // namespace

::m5::hal::v1::error::error_t Bus::attach(::spi_host_device_t host)
{
    if (_owns_bus) {
        (void)release();
    }
    _host     = host;
    _owns_bus = false;
    return ::m5::hal::v1::error::error_t::OK;
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::init(const ::m5::hal::v1::bus::BusConfig& config)
{
    if (config.getBusKind() != ::m5::hal::v1::types::bus_kind_t::SPI) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    const auto& spi_config = static_cast<const BusConfig&>(config);
    // Release the previous bus while `_host` still names the OLD host;
    // adopting the new config first would free the wrong bus and leak
    // the old one.
    if (_owns_bus) {
        (void)release();
    }
    _config = spi_config;
    _host   = spi_config.host;

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
    bus_config.max_transfer_sz    = 64 * 1024;

    auto mapped = mapEspErr(::spi_bus_initialize(_host, &bus_config, SPI_DMA_CH_AUTO));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    if (_config.pin_dc >= 0) {
        setPinOutput(_config.pin_dc, true);
    }
    _owns_bus = true;
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::release(void)
{
    auto removed = removeDevice();
    if (!removed.has_value()) {
        return m5::stl::make_unexpected(removed.error());
    }

    if (_owns_bus) {
        auto mapped = mapEspErr(::spi_bus_free(_host));
        _owns_bus   = false;
        if (::m5::hal::v1::error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
    }
    _transaction_active = false;
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::removeDevice(void)
{
    if (_device == nullptr) {
        return {};
    }
    auto mapped         = mapEspErr(::spi_bus_remove_device(_device));
    _device             = nullptr;
    _device_freq        = 0;
    _device_mode        = 0;
    _device_order       = 0;
    _device_half_duplex = false;
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::ensureDevice(
    const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg, bool half_duplex)
{
    if (_device != nullptr && _device_freq == cfg.freq && _device_mode == cfg.spi_mode &&
        _device_order == cfg.spi_order && _device_half_duplex == half_duplex) {
        return {};
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
    if (cfg.spi_order != 0) {
        dev_config.flags |= SPI_DEVICE_BIT_LSBFIRST;
    }
    if (half_duplex) {
        dev_config.flags |= SPI_DEVICE_HALFDUPLEX;
    }

    auto mapped = mapEspErr(::spi_bus_add_device(_host, &dev_config, &_device));
    if (::m5::hal::v1::error::isError(mapped)) {
        _device = nullptr;
        return m5::stl::make_unexpected(mapped);
    }

    _device_freq        = cfg.freq;
    _device_mode        = cfg.spi_mode;
    _device_order       = cfg.spi_order;
    _device_half_duplex = half_duplex;
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::beginTransaction(
    ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg)
{
    (void)owner;
    if (cfg.freq == 0) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    _transaction_active = true;
    setPinOutput(cfg.pin_cs, false);
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::endTransaction(
    ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg)
{
    (void)owner;
    setPinLevel(cfg.pin_cs, true);
    _transaction_active = false;
    return removeDevice();
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::transfer(
    ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::spi::SPIMasterAccessConfig& cfg,
    const ::m5::hal::v1::spi::TransferDesc& desc, ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx)
{
    (void)owner;
    if (!_transaction_active || cfg.freq == 0) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    const bool half_duplex =
        isHalfDuplexMode(cfg.spi_data_mode) || descNeedsHalfDuplex(desc) || tx == nullptr || rx == nullptr;
    auto dev = ensureDevice(cfg, half_duplex);
    if (!dev.has_value()) {
        return m5::stl::make_unexpected(dev.error());
    }

    const bool has_phase_dc = desc.command_dc_level >= 0 || desc.address_dc_level >= 0 || desc.data_dc_level >= 0;
    if (!has_phase_dc && desc.dc_level_valid) {
        setDC(_config, desc.dc_level ? 1 : 0);
    }

    setDC(_config, desc.command_dc_level);
    auto meta = sendMeta(_device, desc.command, desc.command_bytes);
    if (!meta.has_value()) {
        return m5::stl::make_unexpected(meta.error());
    }

    setDC(_config, desc.address_dc_level);
    meta = sendMeta(_device, desc.address, desc.address_bytes);
    if (!meta.has_value()) {
        return m5::stl::make_unexpected(meta.error());
    }

    auto dummy = sendDummy(_device, desc.dummy_cycles);
    if (!dummy.has_value()) {
        return m5::stl::make_unexpected(dummy.error());
    }

    setDC(_config, desc.data_dc_level);

    size_t transferred = 0;
    while ((tx != nullptr && !tx->eof()) || (rx != nullptr && !rx->closed())) {
        ::m5::hal::v1::data::ConstDataSpan tx_span{};
        ::m5::hal::v1::data::DataSpan rx_span{};
        if (tx != nullptr && !tx->eof()) {
            auto peeked = tx->peek(SIZE_MAX);
            if (!peeked.has_value()) {
                return m5::stl::make_unexpected(peeked.error());
            }
            tx_span = peeked.value();
        }
        if (rx != nullptr && !rx->closed()) {
            auto reserved = rx->reserve(SIZE_MAX);
            if (!reserved.has_value()) {
                return m5::stl::make_unexpected(reserved.error());
            }
            rx_span = reserved.value();
        }

        const size_t chunk_len = (tx_span.size > rx_span.size) ? tx_span.size : rx_span.size;
        if (chunk_len == 0) {
            break;
        }
        auto chunk = transferChunk(_device, tx_span, rx_span);
        if (!chunk.has_value()) {
            return m5::stl::make_unexpected(chunk.error());
        }
        if (tx_span.size > 0) {
            auto advanced = tx->advance(tx_span.size);
            if (!advanced.has_value()) {
                return m5::stl::make_unexpected(advanced.error());
            }
        }
        if (rx_span.size > 0) {
            auto committed = rx->commit(rx_span.size);
            if (!committed.has_value()) {
                return m5::stl::make_unexpected(committed.error());
            }
        }
        transferred += chunk_len;
    }

    return transferred;
}

}  // namespace m5::variants::frameworks::espidf::hal::v1::spi

#endif

#endif
