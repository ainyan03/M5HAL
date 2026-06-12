// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN5_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN5_INL

#include "i2c.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5

#include "backend_master_write_buffer.inl"

#include <esp_err.h>

namespace m5::hal::v1::i2c {

namespace {
namespace impl_espidf_gen5 {

// The shared write-buffer helper stays in the variant namespace
// (backend_master_write_buffer.inl); this TU-local alias keeps the
// `detail::` spelling working in the backend implementation below.
namespace detail = ::m5::variants::frameworks::espidf::hal::v1::i2c::detail;

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
        case ESP_ERR_NOT_FOUND:
            return ::m5::hal::v1::error::error_t::I2C_NO_ACK;
        default:
            return ::m5::hal::v1::error::error_t::I2C_BUS_ERROR;
    }
}

::i2c_addr_bit_len_t addressBitLen(bool address_is_10bit)
{
#if SOC_I2C_SUPPORT_10BIT_ADDR
    return address_is_10bit ? I2C_ADDR_BIT_LEN_10 : I2C_ADDR_BIT_LEN_7;
#else
    // I2C_ADDR_BIT_LEN_10 is gated behind SOC_I2C_SUPPORT_10BIT_ADDR (absent on
    // targets without 10-bit support, and on esp32 IDF patches before it was
    // backported: v5.2.4 / v5.3.3 / v5.4.1). ensureDevice() rejects a 10-bit
    // request up front when the cap is missing, so we only reach 7-bit here.
    (void)address_is_10bit;
    return I2C_ADDR_BIT_LEN_7;
#endif
}

bool isValidAddress(const ::m5::hal::v1::i2c::MasterAccessConfig& cfg)
{
    return cfg.address_is_10bit ? (cfg.i2c_addr <= 0x03FFu) : (cfg.i2c_addr <= 0x007Fu);
}

}  // namespace impl_espidf_gen5
}  // namespace

::m5::hal::v1::error::error_t Bus_espidf::attach(::i2c_master_bus_handle_t bus_handle)
{
    if (bus_handle == nullptr) {
        return ::m5::hal::v1::error::error_t::INVALID_ARGUMENT;
    }
    if (_bus_handle != nullptr) {
        (void)release();
    }
    _bus_handle = bus_handle;
    _owns_bus   = false;
    return ::m5::hal::v1::error::error_t::OK;
}

::m5::hal::v1::result_t<void> Bus_espidf::init(const BusConfig_espidf& config)
{
    _config = config;
    if (_config.pin_scl < 0 || _config.pin_sda < 0) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    if (_bus_handle != nullptr) {
        (void)release();
    }

    ::i2c_master_bus_config_t bus_config    = {};
    bus_config.i2c_port                     = config.i2c_port;
    bus_config.scl_io_num                   = static_cast<::gpio_num_t>(_config.pin_scl);
    bus_config.sda_io_num                   = static_cast<::gpio_num_t>(_config.pin_sda);
    bus_config.clk_source                   = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt            = 7;
    bus_config.flags.enable_internal_pullup = 1;

    auto err    = ::i2c_new_master_bus(&bus_config, &_bus_handle);
    auto mapped = impl_espidf_gen5::mapEspErr(err);
    if (::m5::hal::v1::error::isError(mapped)) {
        _bus_handle = nullptr;
        return m5::stl::make_unexpected(mapped);
    }
    _owns_bus = true;
    return {};
}

::m5::hal::v1::result_t<void> Bus_espidf::release(void)
{
    auto removed = removeDevice();
    if (!removed.has_value()) {
        return m5::stl::make_unexpected(removed.error());
    }

    if (_bus_handle != nullptr && _owns_bus) {
        auto err    = ::i2c_del_master_bus(_bus_handle);
        auto mapped = impl_espidf_gen5::mapEspErr(err);
        _bus_handle = nullptr;
        _owns_bus   = false;
        if (::m5::hal::v1::error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        return {};
    }
    _bus_handle = nullptr;
    _owns_bus   = false;
    return {};
}

::m5::hal::v1::result_t<void> Bus_espidf::removeDevice(void)
{
    if (_dev_handle == nullptr) {
        return {};
    }

    auto mapped           = impl_espidf_gen5::mapEspErr(::i2c_master_bus_rm_device(_dev_handle));
    _dev_handle           = nullptr;
    _dev_addr             = 0;
    _dev_freq             = 0;
    _dev_scl_wait_us      = 0;
    _dev_address_is_10bit = false;
    if (::m5::hal::v1::error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return {};
}

::m5::hal::v1::result_t<void> Bus_espidf::ensureDevice(const ::m5::hal::v1::i2c::MasterAccessConfig& cfg)
{
#if !SOC_I2C_SUPPORT_10BIT_ADDR
    if (cfg.address_is_10bit) {
        // This target's I2C peripheral does not expose 10-bit addressing.
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
#endif
    const uint32_t scl_wait_us = cfg.wire_timeout_ms * 1000u;
    if (_dev_handle != nullptr && _dev_addr == cfg.i2c_addr && _dev_freq == cfg.freq &&
        _dev_scl_wait_us == scl_wait_us && _dev_address_is_10bit == cfg.address_is_10bit) {
        return {};
    }

    auto removed = removeDevice();
    if (!removed.has_value()) {
        return m5::stl::make_unexpected(removed.error());
    }

    ::i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length       = impl_espidf_gen5::addressBitLen(cfg.address_is_10bit);
    dev_config.device_address        = cfg.i2c_addr;
    dev_config.scl_speed_hz          = cfg.freq;
    // i2c_device_config_t::scl_wait_us was added to the bus-device API in IDF
    // v5.2.2 (backported into the v5.2 line); skip it on older patches.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 2)
    dev_config.scl_wait_us = scl_wait_us;
#endif

    auto err    = ::i2c_master_bus_add_device(_bus_handle, &dev_config, &_dev_handle);
    auto mapped = impl_espidf_gen5::mapEspErr(err);
    if (::m5::hal::v1::error::isError(mapped)) {
        _dev_handle = nullptr;
        return m5::stl::make_unexpected(mapped);
    }

    _dev_addr             = cfg.i2c_addr;
    _dev_freq             = cfg.freq;
    _dev_scl_wait_us      = scl_wait_us;
    _dev_address_is_10bit = cfg.address_is_10bit;
    return {};
}

::m5::hal::v1::result_t<size_t> Bus_espidf::transfer(::m5::hal::v1::bus::IAccessor* owner,
                                                     const ::m5::hal::v1::i2c::MasterAccessConfig& cfg,
                                                     const ::m5::hal::v1::i2c::TransferDesc& desc,
                                                     ::m5::hal::v1::data::Source* tx, ::m5::hal::v1::data::Sink* rx)
{
    (void)owner;
    if (_bus_handle == nullptr) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    if (cfg.freq == 0 || !impl_espidf_gen5::isValidAddress(cfg)) {
        return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }

    impl_espidf_gen5::detail::TempWriteBuffer write_bytes;
    auto built = write_bytes.build(::m5::hal::v1::data::ConstDataSpan{desc.prefix, desc.prefix_len}, tx);
    if (!built.has_value()) {
        return m5::stl::make_unexpected(built.error());
    }

    const bool have_tx = !write_bytes.empty();
    const bool have_rx = (rx != nullptr);
    const int timeout  = static_cast<int>(cfg.wire_timeout_ms);

    if (!have_tx && !have_rx) {
        if (cfg.address_is_10bit) {
            return m5::stl::make_unexpected(::m5::hal::v1::error::error_t::INVALID_ARGUMENT);
        }
        auto err    = ::i2c_master_probe(_bus_handle, cfg.i2c_addr, timeout);
        auto mapped = impl_espidf_gen5::mapEspErr(err);
        if (::m5::hal::v1::error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        return size_t{0};
    }

    auto ensured = ensureDevice(cfg);
    if (!ensured.has_value()) {
        return m5::stl::make_unexpected(ensured.error());
    }

    // Data phase only: the staged buffer holds prefix + tx, but the
    // prefix is not counted in the return value (matches SPI).
    size_t total = write_bytes.size() - desc.prefix_len;

    auto finish = [&](::esp_err_t result) -> ::m5::hal::v1::result_t<size_t> {
        auto result_map = impl_espidf_gen5::mapEspErr(result);
        if (::m5::hal::v1::error::isError(result_map)) {
            return m5::stl::make_unexpected(result_map);
        }
        return total;
    };

    if (!have_rx) {
        auto err = ::i2c_master_transmit(_dev_handle, write_bytes.data(), write_bytes.size(), timeout);
        return finish(err);
    }

    auto rsv = rx->reserve(SIZE_MAX);
    if (!rsv.has_value()) {
        return m5::stl::make_unexpected(rsv.error());
    }
    auto rx_span = rsv.value();
    if (rx_span.size == 0) {
        // A sink that reserves 0 bytes degrades this to write-only: the
        // tx phase must still hit the wire — returning early here
        // reported "sent" without ever transmitting.
        if (have_tx) {
            return finish(::i2c_master_transmit(_dev_handle, write_bytes.data(), write_bytes.size(), timeout));
        }
        return finish(ESP_OK);
    }

    ::esp_err_t err = ESP_OK;
    if (have_tx && cfg.use_restart) {
        err = ::i2c_master_transmit_receive(_dev_handle, write_bytes.data(), write_bytes.size(), rx_span.data,
                                            rx_span.size, timeout);
    } else {
        if (have_tx) {
            err = ::i2c_master_transmit(_dev_handle, write_bytes.data(), write_bytes.size(), timeout);
        }
        if (err == ESP_OK) {
            err = ::i2c_master_receive(_dev_handle, rx_span.data, rx_span.size, timeout);
        }
    }
    if (err == ESP_OK) {
        auto com = rx->commit(rx_span.size);
        if (!com.has_value()) {
            return m5::stl::make_unexpected(com.error());
        }
        total += rx_span.size;
    }
    return finish(err);
}

}  // namespace m5::hal::v1::i2c

#endif

#endif
