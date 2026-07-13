// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN4_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN4_INL

#include "i2c.hpp"

#if defined(ESP_PLATFORM) && !M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5 && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN4

#include "backend_master_write_buffer.inl"

#include <esp_err.h>

#include <cstdint>

#include "../../../freertos/hal/runtime/time.hpp"

namespace m5::hal::v2::i2c {

namespace {
namespace impl_espidf_gen4 {

// The shared write-buffer helper stays in the variant namespace
// (backend_master_write_buffer.inl); this TU-local alias keeps the
// `detail::` spelling working in the backend implementation below.
namespace detail = ::m5::variants::frameworks::espidf::hal::v2::i2c::detail;

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
        case ESP_FAIL:
            return error::error_t::I2C_NO_ACK;
        default:
            return error::error_t::I2C_BUS_ERROR;
    }
}

bool isValidAddress(const i2c::MasterAccessConfig& cfg)
{
    return !cfg.address_is_10bit && cfg.i2c_addr <= 0x007Fu;
}

// Whole-command budget for the legacy blocking API. i2c_master_cmd_begin's
// ticks_to_wait bounds the WHOLE queued command chain (driver/i2c.c takes
// cmd_mux and waits each event against the same deadline), so passing
// wire_timeout_ms straight through expires mid-transfer on healthy wires once
// the transfer straddles a tick boundary -- the same tick-quantization race
// measured and fixed on the gen5 driver (see backend_master_gen5.inl's
// transactionTimeoutMs). Budget = expected wire time + the configured
// progress allowance + two tick periods (an N-tick wait guarantees only N-1
// full tick periods of real time).
::TickType_t transactionTimeoutTicks(uint32_t wire_timeout_ms, uint64_t total_bytes, uint32_t freq_hz)
{
    if (wire_timeout_ms == types::TIMEOUT_FOREVER) {
        return portMAX_DELAY;
    }
    // A Sink-driven read may legitimately request SIZE_MAX ("drain the sink");
    // saturate before the bit math below can wrap uint64.
    constexpr uint64_t kMaxCountedBytes = (UINT64_MAX / (9u * 1000u)) - 4u;
    uint64_t total_ms;
    if (total_bytes > kMaxCountedBytes) {
        total_ms = 0xFFFFFFFEu;
    } else {
        // 9 SCL cycles per byte (8 data + ACK); +4 bytes covers the address
        // byte(s) and START/RESTART/STOP framing across both phases.
        const uint64_t wire_ms = ((total_bytes + 4u) * 9u * 1000u) / (freq_hz != 0 ? freq_hz : 1u) + 1u;
        total_ms               = wire_ms + wire_timeout_ms + 2u * portTICK_PERIOD_MS;
        if (total_ms > 0xFFFFFFFEu) {
            total_ms = 0xFFFFFFFEu;  // stay below the TIMEOUT_FOREVER sentinel
        }
    }
    return ::m5::hal::v2::detail::timeoutMsToTicks(static_cast<uint32_t>(total_ms));
}

}  // namespace impl_espidf_gen4
}  // namespace

result_t<void> Bus_espidf::init(const BusConfig_espidf& config)
{
    if (config.pin_scl < 0 || config.pin_sda < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
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

    auto mapped = impl_espidf_gen4::mapEspErr(::i2c_param_config(_port, &conf));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    mapped = impl_espidf_gen4::mapEspErr(::i2c_driver_install(_port, I2C_MODE_MASTER, 0, 0, 0));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    _installed    = true;
    _applied_freq = conf.master.clk_speed;
    return {};
}

result_t<void> Bus_espidf::release(void)
{
    if (_installed) {
        // Transactional release (D1/D9): clear ownership flags only AFTER the
        // ESP-IDF free succeeds. i2c_driver_delete returns an error on bad args /
        // wrong state without having freed the driver, so on error we keep
        // _installed set -- the dtor (or a retried release) can try again, and a
        // swap that aborts on this error keeps a usable old backend instead of a
        // half-torn-down one.
        auto mapped = impl_espidf_gen4::mapEspErr(::i2c_driver_delete(_port));
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        _installed    = false;
        _applied_freq = 0;
    }
    return {};
}

// Unlike the gen5 driver, the legacy `i2c_master_cmd_begin` timeout paths
// (both the queue-receive timeout branch and the I2C_STATUS_TIMEOUT branch)
// call `i2c_hw_fsm_reset()` -- which folds in a clear-bus -- synchronously,
// before returning ESP_ERR_TIMEOUT. So this backend already returns with the
// bus terminated on a timeout; no additional recovery call is needed here.
result_t<void> Bus_espidf::transfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg,
                                    const i2c::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                    size_t rx_len)
{
    (void)owner;
    _transfer_totals.clear();
    if (!_installed || cfg.freq == 0 || !impl_espidf_gen4::isValidAddress(cfg)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    // The gen4 driver has no per-device handle (unlike the gen5 backend, which
    // caches an i2c_master_dev_handle_t and only re-adds it when the
    // address/frequency change). Here the per-target frequency lives only in
    // i2c_config_t; cache the last applied value and re-run i2c_param_config
    // only when it changes (pins change only via init, which resets the cache).
    bool clamped        = false;
    const uint32_t freq = clampMasterClockHz(cfg.freq, &clamped);
    auto mapped         = error::error_t::OK;
    if (freq != _applied_freq) {
        if (clamped) {
            M5_LIB_LOGW("I2C master clock %u Hz exceeds the peripheral ceiling; clamped to %u Hz",
                        static_cast<unsigned>(cfg.freq), static_cast<unsigned>(freq));
        }
        ::i2c_config_t conf   = {};
        conf.mode             = I2C_MODE_MASTER;
        conf.sda_io_num       = static_cast<int>(_config.pin_sda);
        conf.scl_io_num       = static_cast<int>(_config.pin_scl);
        conf.sda_pullup_en    = true;
        conf.scl_pullup_en    = true;
        conf.master.clk_speed = freq;
        conf.clk_flags        = 0;
        mapped                = impl_espidf_gen4::mapEspErr(::i2c_param_config(_port, &conf));
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        _applied_freq = freq;
    }

    impl_espidf_gen4::detail::TempWriteBuffer write_bytes;
    auto built = write_bytes.build(data::ConstDataSpan{desc.prefix, desc.prefix_len}, src, tx_len);
    if (!built.has_value()) {
        return m5::stl::make_unexpected(built.error());
    }

    const bool have_tx = !write_bytes.empty();
    const bool have_rx = (dst != nullptr && rx_len > 0);
    const auto ticks   = impl_espidf_gen4::transactionTimeoutTicks(
        cfg.wire_timeout_ms, static_cast<uint64_t>(write_bytes.size()) + (have_rx ? rx_len : 0), freq);

    if (!have_tx && !have_rx) {
        auto cmd = ::i2c_cmd_link_create();
        if (cmd == nullptr) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        (void)::i2c_master_start(cmd);
        (void)::i2c_master_write_byte(cmd, static_cast<uint8_t>((cfg.i2c_addr << 1) | I2C_MASTER_WRITE), true);
        (void)::i2c_master_stop(cmd);
        auto err = ::i2c_master_cmd_begin(_port, cmd, ticks);
        ::i2c_cmd_link_delete(cmd);
        mapped = impl_espidf_gen4::mapEspErr(err);
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        return {};
    }

    // Data phase only: the staged buffer holds prefix + src, but the
    // prefix is not counted in the return value (matches SPI).
    size_t total    = write_bytes.size() - desc.prefix_len;
    size_t received = 0;
    ::esp_err_t err = ESP_OK;

    if (!have_rx) {
        err = ::i2c_master_write_to_device(_port, static_cast<uint8_t>(cfg.i2c_addr), write_bytes.data(),
                                           write_bytes.size(), ticks);
    } else {
        auto rsv = dst->reserve(rx_len);
        if (!rsv.has_value()) {
            return m5::stl::make_unexpected(rsv.error());
        }
        auto rx_span = rsv.value().first(rx_len);
        if (rx_span.size == 0) {
            // Write-only degradation: the src phase must still hit the
            // wire — returning early here reported "sent" without ever
            // transmitting.
            if (have_tx) {
                mapped = impl_espidf_gen4::mapEspErr(::i2c_master_write_to_device(
                    _port, static_cast<uint8_t>(cfg.i2c_addr), write_bytes.data(), write_bytes.size(), ticks));
                if (error::isError(mapped)) {
                    return m5::stl::make_unexpected(mapped);
                }
            }
            (void)received;
            return {};
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
            auto com = dst->commit(rx_span.size);
            if (!com.has_value()) {
                return m5::stl::make_unexpected(com.error());
            }
            total += rx_span.size;
            received += rx_span.size;
        }
    }

    mapped = impl_espidf_gen4::mapEspErr(err);
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    _transfer_totals.tx += total - received;
    _transfer_totals.rx += received;
    return {};
}

result_t<bus::TransferTotals> Bus_espidf::waitTransfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg)
{
    (void)owner;
    (void)cfg;
    auto totals = _transfer_totals;
    _transfer_totals.clear();
    return totals;
}

bool Bus_espidf::transferBusy(bus::IAccessor* owner)
{
    (void)owner;
    return false;
}

}  // namespace m5::hal::v2::i2c

#endif

#endif
