#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN4_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN4_INL

#include "i2c.hpp"

#if defined(ESP_PLATFORM) && !M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4

#include "backend_master_write_buffer.inl"

#include <esp_err.h>

namespace m5::variants::frameworks::espidf::hal::v1::i2c {

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
        case ESP_FAIL:
            return ::m5::hal::v1::error::error_t::I2C_NO_ACK;
        default:
            return ::m5::hal::v1::error::error_t::I2C_BUS_ERROR;
    }
}

bool isValidAddress(const ::m5::hal::v1::i2c::I2CMasterAccessConfig& cfg)
{
    return !cfg.address_is_10bit && cfg.i2c_addr <= 0x007Fu;
}

::TickType_t timeoutTicks(uint32_t timeout_ms)
{
    return pdMS_TO_TICKS(timeout_ms);
}

}  // namespace

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::init(const BusConfig& config)
{
    if (config.pin_scl < 0 || config.pin_sda < 0) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    // Release the previous driver while `_port` still names the OLD port;
    // adopting the new config first would delete the wrong driver and
    // leak the old one.
    if (_installed) {
        (void)release();
    }
    _config = config;
    _port   = config.i2c_port;

    ::i2c_config_t conf   = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = static_cast<int>(_config.pin_sda);
    conf.scl_io_num       = static_cast<int>(_config.pin_scl);
    conf.sda_pullup_en    = true;
    conf.scl_pullup_en    = true;
    conf.master.clk_speed = 100000;
    conf.clk_flags        = 0;

    auto mapped = mapEspErr(::i2c_param_config(_port, &conf));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    mapped = mapEspErr(::i2c_driver_install(_port, I2C_MODE_MASTER, 0, 0, 0));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    _installed = true;
    return {};
}

m5::stl::expected<void, ::m5::hal::v1::error::error_t> Bus::release(void)
{
    if (_installed) {
        auto mapped = mapEspErr(::i2c_driver_delete(_port));
        _installed  = false;
        if (::m5::hal::v1::error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
    }
    return {};
}

m5::stl::expected<size_t, ::m5::hal::v1::error::error_t> Bus::transfer(
    ::m5::hal::v1::bus::Accessor* owner, const ::m5::hal::v1::i2c::I2CMasterAccessConfig& cfg,
    const ::m5::hal::v1::i2c::TransferDesc& desc, ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx)
{
    (void)owner;
    if (!_installed || cfg.freq == 0 || !isValidAddress(cfg)) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    // The gen4 driver has no per-device handle (unlike the gen5 backend, which
    // caches an i2c_master_dev_handle_t and only re-adds it when the
    // address/frequency change). Here the per-target frequency lives only in
    // i2c_config_t, so we re-apply it each transfer. A future optimization could
    // cache the last applied frequency and skip i2c_param_config when unchanged.
    ::i2c_config_t conf   = {};
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = static_cast<int>(_config.pin_sda);
    conf.scl_io_num       = static_cast<int>(_config.pin_scl);
    conf.sda_pullup_en    = true;
    conf.scl_pullup_en    = true;
    conf.master.clk_speed = cfg.freq;
    conf.clk_flags        = 0;
    auto mapped           = mapEspErr(::i2c_param_config(_port, &conf));
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }

    detail::TempWriteBuffer write_bytes;
    auto built = write_bytes.build(::m5::hal::v1::data::ConstDataSpan{desc.prefix, desc.prefix_len}, tx);
    if (!built.has_value()) {
        return m5::stl::make_unexpected(built.error());
    }

    const bool have_tx = !write_bytes.empty();
    const bool have_rx = (rx != nullptr);
    const auto ticks   = timeoutTicks(cfg.timeout_ms);

    if (!have_tx && !have_rx) {
        auto cmd = ::i2c_cmd_link_create();
        if (cmd == nullptr) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::OUT_OF_RESOURCE);
        }
        (void)::i2c_master_start(cmd);
        (void)::i2c_master_write_byte(cmd, static_cast<uint8_t>((cfg.i2c_addr << 1) | I2C_MASTER_WRITE), true);
        (void)::i2c_master_stop(cmd);
        auto err = ::i2c_master_cmd_begin(_port, cmd, ticks);
        ::i2c_cmd_link_delete(cmd);
        mapped = mapEspErr(err);
        if (::m5::hal::v1::error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        return size_t{0};
    }

    // Data phase only: the staged buffer holds prefix + tx, but the
    // prefix is not counted in the return value (matches SPI, S16 D4).
    size_t total    = write_bytes.size() - desc.prefix_len;
    ::esp_err_t err = ESP_OK;

    if (!have_rx) {
        err = ::i2c_master_write_to_device(_port, static_cast<uint8_t>(cfg.i2c_addr), write_bytes.data(),
                                           write_bytes.size(), ticks);
    } else {
        auto rsv = rx->reserve(SIZE_MAX);
        if (!rsv.has_value()) {
            return m5::stl::make_unexpected(rsv.error());
        }
        auto rx_span = rsv.value();
        if (rx_span.size == 0) {
            // Write-only degradation: the tx phase must still hit the
            // wire — returning early here reported "sent" without ever
            // transmitting (S16 D10).
            if (have_tx) {
                mapped = mapEspErr(::i2c_master_write_to_device(_port, static_cast<uint8_t>(cfg.i2c_addr),
                                                                write_bytes.data(), write_bytes.size(), ticks));
                if (::m5::hal::v1::error::isError(mapped)) {
                    return m5::stl::make_unexpected(mapped);
                }
            }
            return total;
        }

        if (have_tx && cfg.use_restart) {
            err = ::i2c_master_write_read_device(_port, static_cast<uint8_t>(cfg.i2c_addr), write_bytes.data(),
                                                 write_bytes.size(), rx_span.data, rx_span.size, ticks);
        } else {
            if (have_tx) {
                err = ::i2c_master_write_to_device(_port, static_cast<uint8_t>(cfg.i2c_addr), write_bytes.data(),
                                                   write_bytes.size(), ticks);
            }
            if (err == ESP_OK) {
                err = ::i2c_master_read_from_device(_port, static_cast<uint8_t>(cfg.i2c_addr), rx_span.data,
                                                    rx_span.size, ticks);
            }
        }

        if (err == ESP_OK) {
            auto com = rx->commit(rx_span.size);
            if (!com.has_value()) {
                return m5::stl::make_unexpected(com.error());
            }
            total += rx_span.size;
        }
    }

    mapped = mapEspErr(err);
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return total;
}

}  // namespace m5::variants::frameworks::espidf::hal::v1::i2c

#endif

#endif
