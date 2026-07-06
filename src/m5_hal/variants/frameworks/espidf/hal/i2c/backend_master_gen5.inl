// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN5_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_I2C_BACKEND_MASTER_GEN5_INL

#include "i2c.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2C_HAS_MASTER_GEN5

#include "backend_master_write_buffer.inl"

#include <esp_attr.h>
#include <esp_err.h>

#include "../../../freertos/hal/runtime/time.hpp"

namespace m5::hal::v2::i2c {

namespace {
namespace impl_espidf_gen5 {

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
        case ESP_ERR_NOT_FOUND:
            return error::error_t::I2C_NO_ACK;
        case ESP_ERR_INVALID_RESPONSE:
            // ESP-IDF v6.0+ reports a NACKed transfer with this code (v5.x
            // used ESP_ERR_INVALID_STATE, which is ambiguous with driver-state
            // errors and therefore not mapped to I2C_NO_ACK).
            return error::error_t::I2C_NO_ACK;
        default:
            return error::error_t::I2C_BUS_ERROR;
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

bool isValidAddress(const i2c::MasterAccessConfig& cfg)
{
    return cfg.address_is_10bit ? (cfg.i2c_addr <= 0x03FFu) : (cfg.i2c_addr <= 0x007Fu);
}

}  // namespace impl_espidf_gen5
}  // namespace

error::error_t Bus_espidf::attach(::i2c_master_bus_handle_t bus_handle)
{
    if (bus_handle == nullptr) {
        return error::error_t::INVALID_ARGUMENT;
    }
    if (_bus_handle != nullptr) {
        (void)release();
    }
    _bus_handle = bus_handle;
    _owns_bus   = false;
    return error::error_t::OK;
}

result_t<void> Bus_espidf::init(const BusConfig_espidf& config)
{
    _config = config;
    if (_config.pin_scl < 0 || _config.pin_sda < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (_bus_handle != nullptr) {
        (void)release();
    }

    _controller_port                     = config.i2c_port;  // cached for controllerId()
    ::i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port                  = config.i2c_port;
    bus_config.scl_io_num                = static_cast<::gpio_num_t>(_config.pin_scl);
    bus_config.sda_io_num                = static_cast<::gpio_num_t>(_config.pin_sda);
#if M5HAL_ESPIDF_I2C_LP_POOL
    // clk_source / lp_source_clk share a union (driver/i2c_master.h): an
    // LP_I2C port needs the LP source clock, and assigning the HP member
    // here would just be reinterpreting the same bits under the wrong name
    // for that port -- so branch on which member to set instead of always
    // assigning clk_source.
    if (isLowPowerControllerForI2C(controllerId())) {
        bus_config.lp_source_clk = LP_I2C_SCLK_DEFAULT;
    } else {
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    }
#else
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
#endif
    bus_config.glitch_ignore_cnt = 7;
    // Keep the gen5 master in ESP-IDF's synchronous mode. Setting
    // trans_queue_depth > 0 switches the bus to the experimental async queue
    // path; that path is not compatible with i2c_master_probe(), reports bus
    // errors only through callbacks, and constrains async use to one active
    // device per bus. M5HAL therefore keeps hardware I2C synchronous and
    // reserves ServiceRunner-driven async for software I2C and other backends
    // whose state machine is fully owned by M5HAL.
    bus_config.trans_queue_depth = 0;
    // enable_internal_pullup applies uniformly regardless of HP vs. LP port
    // (i2c_master.c: `pull_up_enable = bus_config->flags.enable_internal_pullup`
    // is the same assignment either way; ESP-IDF v5.5.3 checked) -- no LP-
    // specific branch needed here. Hardware measurement on an LP_I2C-capable
    // chip confirms LP_I2C works with this flag set as-is.
    bus_config.flags.enable_internal_pullup = 1;

    auto err    = ::i2c_new_master_bus(&bus_config, &_bus_handle);
    auto mapped = impl_espidf_gen5::mapEspErr(err);
    if (error::isError(mapped)) {
        _bus_handle = nullptr;
        return m5::stl::make_unexpected(mapped);
    }
    _owns_bus = true;
    return {};
}

result_t<void> Bus_espidf::release(void)
{
    if (_transfer_active) {
        auto waited = waitTransfer(_transfer_owner, i2c::MasterAccessConfig{});
        if (!waited.has_value()) {
            return m5::stl::make_unexpected(waited.error());
        }
    }

    auto removed = removeDevice();
    if (!removed.has_value()) {
        return m5::stl::make_unexpected(removed.error());
    }

    if (_bus_handle != nullptr && _owns_bus) {
        // Transactional release (D1/D9): clear the handle/ownership only after
        // the ESP-IDF delete succeeds (see gen4). On error keep them set so the
        // dtor / a retry can free the bus, and a swap keeps a usable old backend.
        auto mapped = impl_espidf_gen5::mapEspErr(::i2c_del_master_bus(_bus_handle));
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        _bus_handle = nullptr;
        _owns_bus   = false;
        return {};
    }
    _bus_handle = nullptr;
    _owns_bus   = false;
    return {};
}

result_t<void> Bus_espidf::removeDevice(void)
{
    if (_transfer_active) {
        auto waited = waitTransfer(_transfer_owner, i2c::MasterAccessConfig{});
        if (!waited.has_value()) {
            return m5::stl::make_unexpected(waited.error());
        }
    }

    if (_dev_handle == nullptr) {
        return {};
    }

    auto mapped = impl_espidf_gen5::mapEspErr(::i2c_master_bus_rm_device(_dev_handle));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    _dev_handle           = nullptr;
    _dev_async            = false;
    _dev_addr             = 0;
    _dev_freq             = 0;
    _dev_scl_wait_us      = 0;
    _dev_address_is_10bit = false;
    return {};
}

result_t<void> Bus_espidf::ensureDevice(const i2c::MasterAccessConfig& cfg)
{
#if !SOC_I2C_SUPPORT_10BIT_ADDR
    if (cfg.address_is_10bit) {
        // This target's I2C peripheral does not expose 10-bit addressing.
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#endif
    bool clamped               = false;
    const uint32_t freq        = clampMasterClockHz(cfg.freq, &clamped);
    const uint32_t scl_wait_us = ::m5::hal::v2::detail::timeoutMsToUsecU32(cfg.wire_timeout_ms);
    if (_dev_handle != nullptr && _dev_addr == cfg.i2c_addr && _dev_freq == freq && _dev_scl_wait_us == scl_wait_us &&
        _dev_address_is_10bit == cfg.address_is_10bit) {
        return {};
    }
    if (clamped) {
        M5_LIB_LOGW("I2C master clock %u Hz exceeds the peripheral ceiling; clamped to %u Hz",
                    static_cast<unsigned>(cfg.freq), static_cast<unsigned>(freq));
    }

    auto removed = removeDevice();
    if (!removed.has_value()) {
        return m5::stl::make_unexpected(removed.error());
    }

    ::i2c_device_config_t dev_config = {};
    dev_config.dev_addr_length       = impl_espidf_gen5::addressBitLen(cfg.address_is_10bit);
    dev_config.device_address        = cfg.i2c_addr;
    dev_config.scl_speed_hz          = freq;
    // i2c_device_config_t::scl_wait_us was added to the bus-device API in IDF
    // v5.2.2 (backported into the v5.2 line); skip it on older patches.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 2)
    dev_config.scl_wait_us = scl_wait_us;
#endif

    auto err    = ::i2c_master_bus_add_device(_bus_handle, &dev_config, &_dev_handle);
    auto mapped = impl_espidf_gen5::mapEspErr(err);
    if (error::isError(mapped)) {
        _dev_handle = nullptr;
        return m5::stl::make_unexpected(mapped);
    }

    _dev_addr             = cfg.i2c_addr;
    _dev_freq             = freq;
    _dev_scl_wait_us      = scl_wait_us;
    _dev_address_is_10bit = cfg.address_is_10bit;

    // Do not register i2c_master_event_callbacks here. Callback registration is
    // only valid on an ESP-IDF async-queue bus, which would make probe/scan and
    // multi-device ownership substantially harder to reason about.
    _dev_async = false;
    return {};
}

service::ServicePoll Bus_espidf::serviceImpl(const service::ServiceContext& ctx)
{
    return serviceTransfer(ctx);
}

bool IRAM_ATTR Bus_espidf::onTransferDone(::i2c_master_dev_handle_t dev, const ::i2c_master_event_data_t* evt,
                                          void* arg)
{
    (void)dev;
    auto* self = static_cast<Bus_espidf*>(arg);
    if (self == nullptr) {
        return false;
    }
    auto err = error::error_t::OK;
    if (evt != nullptr) {
        if (evt->event == I2C_EVENT_NACK) {
            err = error::error_t::I2C_NO_ACK;
        } else if (evt->event == I2C_EVENT_TIMEOUT) {
            err = error::error_t::TIMEOUT_ERROR;
        }
    }
    self->_transfer_callback_status = err;
    return false;
}

void Bus_espidf::unregisterTransferService(void)
{
    if (_transfer_registered) {
        (void)M5_Hal.Services.remove(*this);
        _transfer_registered = false;
    }
}

void Bus_espidf::clearTransferState(void)
{
    unregisterTransferService();
    _transfer_owner = nullptr;
    _transfer_dst   = nullptr;
    _transfer_tx_buf.reset();
    _transfer_tx_count        = 0;
    _transfer_rx_count        = 0;
    _transfer_active          = false;
    _transfer_done            = true;
    _transfer_callback_status = error::error_t::ASYNC_RUNNING;
}

service::ServiceResult Bus_espidf::serviceTransfer(const service::ServiceContext& ctx)
{
    (void)ctx;
    if (!_transfer_active || _transfer_done) {
        unregisterTransferService();
        return service::ServiceResult::Done;
    }
    if (_transfer_callback_status == error::error_t::ASYNC_RUNNING) {
        return service::ServiceResult::Idle;
    }

    unregisterTransferService();
    if (error::isError(_transfer_callback_status)) {
        return service::ServiceResult::Error;
    }
    if (_transfer_dst != nullptr && _transfer_rx_count > 0) {
        auto committed = _transfer_dst->commit(_transfer_rx_count);
        if (!committed.has_value()) {
            _transfer_callback_status = committed.error();
            return service::ServiceResult::Error;
        }
    }
    _transfer_totals.tx += _transfer_tx_count;
    _transfer_totals.rx += _transfer_rx_count;
    _transfer_done = true;
    return service::ServiceResult::Done;
}

result_t<void> Bus_espidf::transfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg,
                                    const i2c::TransferDesc& desc, data::Source* src, size_t tx_len, data::Sink* dst,
                                    size_t rx_len)
{
    auto waited = waitTransfer(owner, cfg);
    if (!waited.has_value()) {
        return m5::stl::make_unexpected(waited.error());
    }
    if (_bus_handle == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (cfg.freq == 0 || !impl_espidf_gen5::isValidAddress(cfg)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    impl_espidf_gen5::detail::TempWriteBuffer write_bytes;
    auto built = write_bytes.build(data::ConstDataSpan{desc.prefix, desc.prefix_len}, src, tx_len);
    if (!built.has_value()) {
        return m5::stl::make_unexpected(built.error());
    }

    const bool have_tx = !write_bytes.empty();
    const bool have_rx = (dst != nullptr && rx_len > 0);
    const int timeout  = static_cast<int>(cfg.wire_timeout_ms);

    if (!have_tx && !have_rx) {
        if (cfg.address_is_10bit) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        if (_dev_async && _owns_bus) {
            BusConfig_espidf config;
            config.pin_scl  = _config.pin_scl;
            config.pin_sda  = _config.pin_sda;
            config.i2c_port = _controller_port;
            auto released   = release();
            if (!released.has_value()) {
                return m5::stl::make_unexpected(released.error());
            }
            auto initialized = init(config);
            if (!initialized.has_value()) {
                return m5::stl::make_unexpected(initialized.error());
            }
        } else {
            auto removed = removeDevice();
            if (!removed.has_value()) {
                return m5::stl::make_unexpected(removed.error());
            }
        }

        auto err    = ::i2c_master_probe(_bus_handle, cfg.i2c_addr, timeout);
        auto mapped = impl_espidf_gen5::mapEspErr(err);
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        return {};
    }

    auto ensured = ensureDevice(cfg);
    if (!ensured.has_value()) {
        return m5::stl::make_unexpected(ensured.error());
    }

    // Data phase only: the staged buffer holds prefix + src, but the
    // prefix is not counted in the return value (matches SPI).
    size_t total    = write_bytes.size() - desc.prefix_len;
    size_t received = 0;

    auto finish = [&](::esp_err_t result) -> result_t<void> {
        auto result_map = impl_espidf_gen5::mapEspErr(result);
        if (error::isError(result_map)) {
            return m5::stl::make_unexpected(result_map);
        }
        _transfer_totals.tx += total - received;
        _transfer_totals.rx += received;
        return {};
    };

    auto arm_async = [&](data::Sink* async_dst, size_t rx_count, size_t tx_count) {
        _transfer_owner           = owner;
        _transfer_dst             = async_dst;
        _transfer_tx_count        = tx_count;
        _transfer_rx_count        = rx_count;
        _transfer_active          = true;
        _transfer_done            = false;
        _transfer_callback_status = error::error_t::ASYNC_RUNNING;
    };
    auto finish_async_start = [&](::esp_err_t err) -> result_t<void> {
        auto result_map = impl_espidf_gen5::mapEspErr(err);
        if (error::isError(result_map)) {
            clearTransferState();
            return m5::stl::make_unexpected(result_map);
        }
        if (owner == nullptr) {
            auto done = waitTransfer(owner, cfg);
            if (!done.has_value()) {
                return m5::stl::make_unexpected(done.error());
            }
        } else if (M5_Hal.Services.add(*this)) {
            _transfer_registered = true;
        } else {
            clearTransferState();
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        return {};
    };

    if (!have_rx) {
        if (!_dev_async) {
            return finish(::i2c_master_transmit(_dev_handle, write_bytes.data(), write_bytes.size(), timeout));
        }

        _transfer_tx_buf = memory::TempBuffer{memory::defaultAllocator(), write_bytes.size()};
        if (!_transfer_tx_buf) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        std::memcpy(_transfer_tx_buf.data(), write_bytes.data(), write_bytes.size());

        arm_async(nullptr, 0, total);
        return finish_async_start(::i2c_master_transmit(_dev_handle, static_cast<uint8_t*>(_transfer_tx_buf.data()),
                                                        write_bytes.size(), timeout));
    }

    auto rsv = dst->reserve(rx_len);
    if (!rsv.has_value()) {
        return m5::stl::make_unexpected(rsv.error());
    }
    auto rx_span = rsv.value().first(rx_len);
    if (rx_span.size == 0) {
        // A sink that reserves 0 bytes degrades this to write-only: the
        // src phase must still hit the wire — returning early here
        // reported "sent" without ever transmitting.
        if (have_tx) {
            return finish(::i2c_master_transmit(_dev_handle, write_bytes.data(), write_bytes.size(), timeout));
        }
        return finish(ESP_OK);
    }

    if (_dev_async) {
        if (have_tx) {
            _transfer_tx_buf = memory::TempBuffer{memory::defaultAllocator(), write_bytes.size()};
            if (!_transfer_tx_buf) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            std::memcpy(_transfer_tx_buf.data(), write_bytes.data(), write_bytes.size());
        }
        arm_async(dst, rx_span.size, total);
        ::esp_err_t err = ESP_OK;
        if (have_tx && cfg.use_restart) {
            err = ::i2c_master_transmit_receive(_dev_handle, static_cast<uint8_t*>(_transfer_tx_buf.data()),
                                                write_bytes.size(), rx_span.data, rx_span.size, timeout);
        } else if (have_tx) {
            ::i2c_operation_job_t ops[6] = {};
            ops[0].command               = I2C_MASTER_CMD_START;
            ops[1].command               = I2C_MASTER_CMD_WRITE;
            ops[1].write.ack_check       = true;
            ops[1].write.data            = static_cast<uint8_t*>(_transfer_tx_buf.data());
            ops[1].write.total_bytes     = write_bytes.size();
            ops[2].command               = I2C_MASTER_CMD_STOP;
            ops[3].command               = I2C_MASTER_CMD_START;
            ops[4].command               = I2C_MASTER_CMD_READ;
            ops[4].read.ack_value        = I2C_NACK_VAL;
            ops[4].read.data             = rx_span.data;
            ops[4].read.total_bytes      = rx_span.size;
            ops[5].command               = I2C_MASTER_CMD_STOP;
            err                          = ::i2c_master_execute_defined_operations(_dev_handle, ops, 6, timeout);
        } else {
            err = ::i2c_master_receive(_dev_handle, rx_span.data, rx_span.size, timeout);
        }
        return finish_async_start(err);
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
        auto com = dst->commit(rx_span.size);
        if (!com.has_value()) {
            return m5::stl::make_unexpected(com.error());
        }
        total += rx_span.size;
        received += rx_span.size;
    }
    return finish(err);
}

result_t<bus::TransferTotals> Bus_espidf::waitTransfer(bus::IAccessor* owner, const i2c::MasterAccessConfig& cfg)
{
    (void)cfg;
    if (_transfer_active && _transfer_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    while (_transfer_active && !_transfer_done && !error::isError(_transfer_callback_status)) {
        auto result = serviceTransfer(service::ServiceContext{service::fastTick()});
        if (result == service::ServiceResult::Error) {
            break;
        }
        if (result == service::ServiceResult::Idle) {
            runtime::yield();
        }
    }
    if (error::isError(_transfer_callback_status)) {
        const auto err = _transfer_callback_status;
        clearTransferState();
        return m5::stl::make_unexpected(err);
    }
    auto totals = _transfer_totals;
    _transfer_totals.clear();
    clearTransferState();
    return totals;
}

bool Bus_espidf::transferBusy(bus::IAccessor* owner)
{
    return _transfer_active && _transfer_owner == owner && !_transfer_done &&
           !error::isError(_transfer_callback_status);
}

}  // namespace m5::hal::v2::i2c

#endif

#endif
