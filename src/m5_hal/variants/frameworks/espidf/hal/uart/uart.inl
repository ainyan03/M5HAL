#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_INL

#include "uart.hpp"

#if defined(ESP_PLATFORM)

#include <freertos/FreeRTOS.h>

namespace m5::variants::frameworks::espidf::hal::v1::uart {

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
        default:
            return ::m5::hal::v1::error::error_t::UNKNOWN_ERROR;
    }
}

::uart_port_t resolvePort(int8_t port_num)
{
    if (port_num < 0) {
        return UART_NUM_0;
    }
    return static_cast<::uart_port_t>(port_num);
}

::uart_word_length_t wordLength(uint8_t data_bits)
{
    switch (data_bits) {
        case 5:
            return UART_DATA_5_BITS;
        case 6:
            return UART_DATA_6_BITS;
        case 7:
            return UART_DATA_7_BITS;
        case 8:
        default:
            return UART_DATA_8_BITS;
    }
}

::uart_stop_bits_t stopBits(uint8_t stop_bits)
{
    return stop_bits == 2 ? UART_STOP_BITS_2 : UART_STOP_BITS_1;
}

::uart_parity_t parity(::m5::hal::v1::uart::parity_t p)
{
    switch (p) {
        case ::m5::hal::v1::uart::parity_t::even:
            return UART_PARITY_EVEN;
        case ::m5::hal::v1::uart::parity_t::odd:
            return UART_PARITY_ODD;
        case ::m5::hal::v1::uart::parity_t::none:
        default:
            return UART_PARITY_DISABLE;
    }
}

bool sameConfig(const ::m5::hal::v1::uart::UARTAccessConfig& lhs, const ::m5::hal::v1::uart::UARTAccessConfig& rhs)
{
    return lhs.baud_rate == rhs.baud_rate && lhs.data_bits == rhs.data_bits && lhs.stop_bits == rhs.stop_bits &&
           lhs.parity == rhs.parity && lhs.invert == rhs.invert;
}

::TickType_t ticks(uint32_t timeout_ms)
{
    return pdMS_TO_TICKS(timeout_ms);
}

}  // namespace

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::init(const ::m5::hal::v1::bus::BusConfig& config)
{
    if (config.getBusKind() != ::m5::hal::v1::types::bus_kind_t::UART) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    const auto& uart_config = static_cast<const BusConfig&>(config);
    _config                 = uart_config;
    _port                   = resolvePort(uart_config.port_num);
    if (_port < UART_NUM_0 || _port >= UART_NUM_MAX) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    const int rx_size = static_cast<int>(_config.rx_buffer_size == 0 ? 256 : _config.rx_buffer_size);
    const int tx_size = static_cast<int>(_config.tx_buffer_size);
    auto mapped       = mapEspErr(::uart_driver_install(_port, rx_size, tx_size, 0, nullptr, 0));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    _installed  = true;
    _configured = false;
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::release(void)
{
    if (_installed) {
        auto mapped = mapEspErr(::uart_driver_delete(_port));
        _installed  = false;
        _configured = false;
        if (::m5::hal::v1::error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
    }
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::applyConfig(
    const ::m5::hal::v1::uart::UARTAccessConfig& cfg)
{
    if (!_installed || cfg.baud_rate == 0 || cfg.data_bits < 5 || cfg.data_bits > 8 ||
        (cfg.stop_bits != 1 && cfg.stop_bits != 2)) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    if (_configured && sameConfig(_applied_cfg, cfg)) {
        return {};
    }

    ::uart_config_t native_cfg = {};
    native_cfg.baud_rate       = static_cast<int>(cfg.baud_rate);
    native_cfg.data_bits       = wordLength(cfg.data_bits);
    native_cfg.parity          = parity(cfg.parity);
    native_cfg.stop_bits       = stopBits(cfg.stop_bits);
    native_cfg.flow_ctrl       = UART_HW_FLOWCTRL_DISABLE;
#if defined(UART_SCLK_DEFAULT)
    native_cfg.source_clk = UART_SCLK_DEFAULT;
#endif

    auto mapped = mapEspErr(::uart_param_config(_port, &native_cfg));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    mapped = mapEspErr(::uart_set_pin(_port, static_cast<int>(_config.pin_tx), static_cast<int>(_config.pin_rx),
                                      static_cast<int>(_config.pin_rts), static_cast<int>(_config.pin_cts)));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    mapped = mapEspErr(::uart_set_line_inverse(_port, cfg.invert ? UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV : 0));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }

    _applied_cfg = cfg;
    _configured  = true;
    return {};
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::write(::m5::hal::v1::bus::Accessor* owner,
                                                                    const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                    ::m5::hal::v1::data::Source* tx, size_t len)
{
    (void)owner;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }

    size_t done = 0;
    while (tx != nullptr && !tx->eof() && done < len) {
        auto span = tx->peek(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        const int written = ::uart_write_bytes(_port, span.value().data, span.value().size);
        if (written < 0) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
        }
        auto advanced = tx->advance(static_cast<size_t>(written));
        if (!advanced.has_value()) {
            return m5::stl::make_unexpected(advanced.error());
        }
        done += static_cast<size_t>(written);
    }

    auto mapped = mapEspErr(::uart_wait_tx_done(_port, ticks(cfg.write_timeout_ms)));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return done;
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::read(::m5::hal::v1::bus::Accessor* owner,
                                                                   const ::m5::hal::v1::uart::UARTAccessConfig& cfg,
                                                                   ::m5::hal::v1::data::Sink* rx, size_t len)
{
    (void)owner;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }

    size_t done = 0;
    while (rx != nullptr && !rx->closed() && done < len) {
        auto span = rx->reserve(len - done);
        if (!span.has_value()) {
            return m5::stl::make_unexpected(span.error());
        }
        if (span.value().size == 0) {
            break;
        }
        const auto timeout = done == 0 ? ticks(cfg.first_byte_timeout_ms) : ticks(cfg.inter_byte_timeout_ms);
        const int read     = ::uart_read_bytes(_port, span.value().data, span.value().size, timeout);
        if (read < 0) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::UNKNOWN_ERROR);
        }
        if (read == 0) {
            break;
        }
        auto committed = rx->commit(static_cast<size_t>(read));
        if (!committed.has_value()) {
            return m5::stl::make_unexpected(committed.error());
        }
        done += static_cast<size_t>(read);
    }
    return done;
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::readableBytes(
    ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::uart::UARTAccessConfig& cfg)
{
    (void)owner;
    auto applied = applyConfig(cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    size_t size = 0;
    auto mapped = mapEspErr(::uart_get_buffered_data_len(_port, &size));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return size;
}

}  // namespace m5::variants::frameworks::espidf::hal::v1::uart

#endif

#endif
