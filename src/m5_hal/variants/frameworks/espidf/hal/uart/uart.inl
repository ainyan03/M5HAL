// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_INL
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_INL

#include "uart.hpp"

#if defined(ESP_PLATFORM)

#include <esp_err.h>
#include <freertos/FreeRTOS.h>

#include "../../../../../hal/v2/diag.hpp"
#include "../../../freertos/hal/runtime/time.hpp"

namespace m5::hal::v2::uart {

namespace {
namespace impl_espidf {

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

::uart_parity_t parity(uart::parity_t p)
{
    switch (p) {
        case uart::parity_t::Even:
            return UART_PARITY_EVEN;
        case uart::parity_t::Odd:
            return UART_PARITY_ODD;
        case uart::parity_t::None:
        default:
            return UART_PARITY_DISABLE;
    }
}

bool sameConfig(const uart::AccessConfig& lhs, const uart::AccessConfig& rhs)
{
    return lhs.baud_rate == rhs.baud_rate && lhs.data_bits == rhs.data_bits && lhs.stop_bits == rhs.stop_bits &&
           lhs.parity == rhs.parity && lhs.invert == rhs.invert;
}

::TickType_t ticks(uint32_t timeout_ms)
{
    return ::m5::hal::v2::detail::timeoutMsToTicks(timeout_ms);
}

// RAII unlock for a runtime::Mutex critical section (mirrors the pattern in
// service.inl's ControlUnlock) so an early return can never leak the lock.
struct MutexUnlock {
    runtime::Mutex* m;
    ~MutexUnlock()
    {
        m->unlock();
    }
};

}  // namespace impl_espidf
}  // namespace

result_t<void> Bus_espidf::init(const BusConfig_espidf& config)
{
    const auto new_port = impl_espidf::resolvePort(config.port_num);
    if (new_port < UART_NUM_0 || new_port >= UART_NUM_MAX) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    // Re-init: release the previous driver while `_port` still names the
    // OLD port (otherwise `uart_driver_install` fails on the already-
    // installed port and the old driver leaks). release() takes its own
    // `_state_mutex` critical section below; do not nest another one around
    // it (the mutex is non-recursive).
    if (_installed) {
        (void)release();
    }

    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    impl_espidf::MutexUnlock state_unlock{&_state_mutex};

    _config = config;
    _port   = new_port;

    const int rx_size = static_cast<int>(_config.rx_buffer_size == 0 ? 256 : _config.rx_buffer_size);
    const int tx_size = static_cast<int>(_config.tx_buffer_size);
    auto mapped       = impl_espidf::mapEspErr(::uart_driver_install(_port, rx_size, tx_size, 0, nullptr, 0));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    // Fire the rx interrupt earlier than the driver default (~100/128 bytes).
    // At multi-Mbaud rates the default leaves the ISR less than ~100 us of
    // headroom before the hardware FIFO overflows; concurrent DMA interrupt
    // load (e.g. I2S) was observed to trip that on hardware, leaving rx
    // stalled. Half-full doubles the margin at negligible interrupt cost.
    // (Threshold-only setters: uart_intr_config would overwrite the whole
    // interrupt enable mask and break the driver's tx handling.)
    (void)::uart_set_rx_full_threshold(_port, 64);
    _installed  = true;
    _configured = false;
    return {};
}

result_t<void> Bus_espidf::release(void)
{
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    impl_espidf::MutexUnlock state_unlock{&_state_mutex};

    if (_installed) {
        // Transactional release (D1/D9): clear flags only after the ESP-IDF
        // delete succeeds (see i2c gen4). On error keep _installed set so the
        // dtor / a retry can delete the driver.
        auto mapped = impl_espidf::mapEspErr(::uart_driver_delete(_port));
        if (error::isError(mapped)) {
            return m5::stl::make_unexpected(mapped);
        }
        _installed  = false;
        _configured = false;
    }
    return {};
}

uint32_t Bus_espidf::reconfigSkips()
{
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return 0;
    }
    impl_espidf::MutexUnlock state_unlock{&_state_mutex};
    return _reconfig_skips;
}

result_t<void> Bus_espidf::applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg)
{
    if (!_state_mutex.lock(types::TIMEOUT_FOREVER)) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    impl_espidf::MutexUnlock state_unlock{&_state_mutex};

    if (!_installed || cfg.baud_rate == 0 || cfg.data_bits < 5 || cfg.data_bits > 8 ||
        (cfg.stop_bits != 1 && cfg.stop_bits != 2)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (!_configured) {
        // First apply on a fresh driver install: no other owner can be
        // mid-transfer yet (spec/design/uart.md), so no quiescence gate.
        return applyConfigLocked(cfg);
    }
    if (impl_espidf::sameConfig(_applied_cfg, cfg)) {
        return {};
    }

    if (owner == nullptr) {
        // A reconfigure without an accessor identity cannot prove quiescence.
        ++_reconfig_skips;
        M5HAL_DIAG("uart reconfig skipped: no accessor identity (skips=%u)", static_cast<unsigned>(_reconfig_skips));
        return {};
    }

    // Reconfigure: apply only when the opposite channel is quiescent for
    // `owner` (spec/design/uart.md; IBus::tryAcquireOppositeChannel is the
    // sanctioned exception to the channel-lock -> state-mutex ordering).
    auto& ibus = static_cast<IBus&>(owner->getBus());
    auto grant = ibus.tryAcquireOppositeChannel(owner, entered);
    if (!grant.granted) {
        ++_reconfig_skips;
        M5HAL_DIAG("uart reconfig skipped: opposite channel busy (skips=%u)", static_cast<unsigned>(_reconfig_skips));
        return {};  // keep serving the currently applied config
    }
    auto applied = applyConfigLocked(cfg);
    ibus.releaseOppositeChannel(owner, grant);
    return applied;
}

result_t<void> Bus_espidf::applyConfigLocked(const uart::AccessConfig& cfg)
{
    ::uart_config_t native_cfg = {};
    native_cfg.baud_rate       = static_cast<int>(cfg.baud_rate);
    native_cfg.data_bits       = impl_espidf::wordLength(cfg.data_bits);
    native_cfg.parity          = impl_espidf::parity(cfg.parity);
    native_cfg.stop_bits       = impl_espidf::stopBits(cfg.stop_bits);
    native_cfg.flow_ctrl       = UART_HW_FLOWCTRL_DISABLE;
#if defined(UART_SCLK_DEFAULT)
    native_cfg.source_clk = UART_SCLK_DEFAULT;
#endif

    auto mapped = impl_espidf::mapEspErr(::uart_param_config(_port, &native_cfg));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    mapped =
        impl_espidf::mapEspErr(::uart_set_pin(_port, static_cast<int>(_config.pin_tx), static_cast<int>(_config.pin_rx),
                                              static_cast<int>(_config.pin_rts), static_cast<int>(_config.pin_cts)));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    mapped = impl_espidf::mapEspErr(
        ::uart_set_line_inverse(_port, cfg.invert ? UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV : 0));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }

    _applied_cfg = cfg;
    _configured  = true;
    return {};
}

result_t<size_t> Bus_espidf::rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    (void)timeout_ms;
    const int written = ::uart_write_bytes(_port, data, len);
    if (written < 0) {
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    return static_cast<size_t>(written);
}

result_t<size_t> Bus_espidf::rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    const int rd = ::uart_read_bytes(_port, buf, len, impl_espidf::ticks(timeout_ms));
    if (rd < 0) {
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    return static_cast<size_t>(rd);
}

result_t<size_t> Bus_espidf::rawReadableBytes()
{
    size_t size = 0;
    auto mapped = impl_espidf::mapEspErr(::uart_get_buffered_data_len(_port, &size));
    if (error::isError(mapped)) {
        return m5::stl::make_unexpected(mapped);
    }
    return size;
}

result_t<size_t> Bus_espidf::write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src, size_t len)
{
    auto applied = applyConfig(owner, Channel::Tx, cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    auto result = Bus_streaming::write(owner, cfg, src, len);
    auto mapped = impl_espidf::mapEspErr(::uart_wait_tx_done(_port, impl_espidf::ticks(cfg.write_timeout_ms)));
    if (error::isError(mapped) && result.has_value()) {
        return m5::stl::make_unexpected(mapped);
    }
    return result;
}

result_t<size_t> Bus_espidf::read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len)
{
    auto applied = applyConfig(owner, Channel::Rx, cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    return Bus_streaming::read(owner, cfg, dst, len);
}

result_t<size_t> Bus_espidf::readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg)
{
    auto applied = applyConfig(owner, Channel::Rx, cfg);
    if (!applied.has_value()) {
        return m5::stl::make_unexpected(applied.error());
    }
    return Bus_streaming::readableBytes(owner, cfg);
}

}  // namespace m5::hal::v2::uart

#endif

#endif
