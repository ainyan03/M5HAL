// SPDX-License-Identifier: MIT
#include "usb_jtag.hpp"
#include "error.hpp"

#if defined(ESP_PLATFORM) && defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED)

namespace m5::hal::v2::uart {

result_t<void> Bus_espidf_usb_jtag::init()
{
    return initWithBufferSizes(kRxDriverBufferSize, kTxDriverBufferSize, /*attach_existing=*/false);
}

result_t<void> Bus_espidf_usb_jtag::initWithBufferSizes(uint32_t rx_buffer_size, uint32_t tx_buffer_size,
                                                        bool attach_existing)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    if (_installed) {
        auto closed = teardownBackend();
        if (closed.disposition != bus::CloseDisposition::Success) {
            return m5::stl::make_unexpected(closed.error_code);
        }
    }

    usb_serial_jtag_driver_config_t native_cfg = {};
    native_cfg.rx_buffer_size                  = rx_buffer_size;
    native_cfg.tx_buffer_size                  = tx_buffer_size;

    const auto err = usb_serial_jtag_driver_install(&native_cfg);
    if (err == ESP_OK) {
        _driver_owned = true;
    } else if (err == ESP_ERR_INVALID_STATE && attach_existing) {
        _driver_owned = false;
    } else {
        return m5::stl::make_unexpected(impl_espidf::mapEspErr(err));
    }

    _config.rx_buffer_size = rx_buffer_size;
    _config.tx_buffer_size = tx_buffer_size;
    _installed             = true;
    clearRxCache();
    auto initialized = markInitializationSucceeded(false);
    if (!initialized) {
        (void)teardownBackend();
        return initialized;
    }
    return {};
}

bus::CloseOutcome Bus_espidf_usb_jtag::teardownBackend(void)
{
    if (!_installed) {
        return bus::CloseOutcome::success();
    }
    // Transactional teardown (matches uart.inl / i2c gen4): clear _installed
    // only after an owned ESP-IDF driver uninstalls successfully. An attached
    // console driver remains caller-owned and is only detached here.
    if (_driver_owned) {
        const auto mapped = impl_espidf::mapEspErr(usb_serial_jtag_driver_uninstall());
        if (error::isError(mapped)) {
            return bus::CloseOutcome::noMutation(mapped);
        }
    }
    _installed    = false;
    _driver_owned = false;
    clearRxCache();
    return bus::CloseOutcome::success();
}

result_t<size_t> Bus_espidf_usb_jtag::rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!_installed) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const int written = usb_serial_jtag_write_bytes(data, len, ticks(timeout_ms));
    if (written < 0 || static_cast<size_t>(written) > len) {
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    if (written == 0) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    return static_cast<size_t>(written);
}

result_t<size_t> Bus_espidf_usb_jtag::rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    if (!_installed) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const size_t cached = popRxCache(buf, len);
    if (cached > 0) {
        return cached;
    }
    const int rd = usb_serial_jtag_read_bytes(buf, len, ticks(timeout_ms));
    if (rd < 0 || static_cast<size_t>(rd) > len) {
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    return static_cast<size_t>(rd);
}

result_t<size_t> Bus_espidf_usb_jtag::rawReadableBytes()
{
    if (!_installed) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    pumpRxCache();
    return _rx_count;
}

void Bus_espidf_usb_jtag::clearRxCache()
{
    _rx_head  = 0;
    _rx_count = 0;
}

size_t Bus_espidf_usb_jtag::rxTail() const
{
    return (_rx_head + _rx_count) % kRxCacheSize;
}

void Bus_espidf_usb_jtag::pumpRxCache()
{
    while (_rx_count < kRxCacheSize) {
        const size_t tail = rxTail();
        const size_t room = (tail < _rx_head) ? (_rx_head - tail) : (kRxCacheSize - tail);
        const int rd      = usb_serial_jtag_read_bytes(&_rx_cache[tail], room, 0);
        if (rd <= 0 || static_cast<size_t>(rd) > room) {
            break;
        }
        _rx_count += static_cast<size_t>(rd);
    }
}

size_t Bus_espidf_usb_jtag::popRxCache(uint8_t* dst, size_t max_len)
{
    size_t copied = 0;
    while (copied < max_len && _rx_count > 0) {
        const size_t chunk_a = kRxCacheSize - _rx_head;
        const size_t want    = max_len - copied;
        size_t chunk         = _rx_count < chunk_a ? _rx_count : chunk_a;
        chunk                = chunk < want ? chunk : want;
        memcpy(dst + copied, &_rx_cache[_rx_head], chunk);
        _rx_head = (_rx_head + chunk) % kRxCacheSize;
        _rx_count -= chunk;
        copied += chunk;
    }
    if (_rx_count == 0) {
        _rx_head = 0;
    }
    return copied;
}

}  // namespace m5::hal::v2::uart

#endif  // ESP_PLATFORM && CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
