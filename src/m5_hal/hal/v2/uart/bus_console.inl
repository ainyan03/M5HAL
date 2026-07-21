// SPDX-License-Identifier: MIT
#ifndef M5_HAL_UART_BUS_CONSOLE_INL_
#define M5_HAL_UART_BUS_CONSOLE_INL_

#include "bus_console.hpp"

#include <new>

#if defined(ESP_PLATFORM)
#include "../../../variants/frameworks/freertos/hal/runtime/time.hpp"
#include "../../../variants/frameworks/espidf/hal/uart/usb_cdc.hpp"
#include "../../../variants/frameworks/espidf/hal/uart/usb_jtag.hpp"

#include <driver/uart.h>
#elif !defined(_WIN32) && !defined(ARDUINO)
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>
#endif

namespace m5::hal::v2::uart {

result_t<void> Bus_console::init(FILE* in, FILE* out)
{
#if defined(_WIN32) || (defined(ARDUINO) && !defined(ESP_PLATFORM))
    (void)in;
    (void)out;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
#else
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto closed = teardownBackend();
    if (closed.disposition != bus::CloseDisposition::Success) {
        return m5::stl::make_unexpected(closed.error_code);
    }

    if (in == nullptr || out == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _file_in  = in;
    _file_out = out;
    _fd_in    = fileno(in);
    _fd_out   = fileno(out);
    if (_fd_in < 0 || _fd_out < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

#if defined(ESP_PLATFORM)
    // Runtime auto-detection. When TinyUSB CDC is enabled, try OTG CDC
    // first (USB-JTAG driver install can succeed even when the phy is in
    // OTG mode, producing a silent dead path). Otherwise try USB-JTAG first.
#if defined(SOC_USB_OTG_SUPPORTED) && SOC_USB_OTG_SUPPORTED && defined(CONFIG_TINYUSB_CDC_ENABLED) && \
    CONFIG_TINYUSB_CDC_ENABLED && __has_include(<tinyusb.h>)
    {
        std::unique_ptr<Bus_espidf_usb_cdc> cdc{new (std::nothrow) Bus_espidf_usb_cdc()};
        if (cdc != nullptr && cdc->init().has_value()) {
            _delegate  = std::move(cdc);
            _transport = Transport::Delegate;
        }
    }
#endif
#if defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED) && CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    if (_transport == Transport::None) {
        std::unique_ptr<Bus_espidf_usb_jtag> jtag{new (std::nothrow) Bus_espidf_usb_jtag()};
        if (jtag != nullptr &&
            jtag->initWithBufferSizes(/*rx=*/1024, /*tx=*/1024, /*attach_existing=*/true).has_value()) {
            _delegate  = std::move(jtag);
            _transport = Transport::Delegate;
        }
    }
#endif
    if (_transport == Transport::None) {
        const uart_port_t port = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
        if (!uart_is_driver_installed(port)) {
            uart_driver_install(port, 256, 0, 0, nullptr, 0);
        }
        _transport = Transport::Uart;
    }
#else
    _transport = Transport::Posix;
#endif

    auto initialized = markInitializationSucceeded(false);
    if (!initialized.has_value()) {
        (void)teardownBackend();
        return initialized;
    }
    return {};
#endif
}

bus::CloseOutcome Bus_console::teardownBackend(void)
{
    if (_transport == Transport::None) {
        return bus::CloseOutcome::success();
    }

    if (_delegate != nullptr) {
        auto closed = closeOwnedBackend(*_delegate);
        if (closed.disposition != bus::CloseDisposition::Success) {
            return closed;
        }
        _delegate.reset();
    }
    _file_in   = nullptr;
    _file_out  = nullptr;
    _fd_in     = -1;
    _fd_out    = -1;
    _transport = Transport::None;
    return bus::CloseOutcome::success();
}

bus::CloseOutcome Bus_console::closeBackend(void)
{
    return teardownBackend();
}

result_t<void> Bus_console::beginOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    return _delegate != nullptr ? beginOperationOn(*_delegate, context) : Bus_streaming::beginOperationBackend(context);
}

result_t<void> Bus_console::endOperationBackend(bus::OperationContext<AccessConfig>& context)
{
    return _delegate != nullptr ? endOperationOn(*_delegate, context) : Bus_streaming::endOperationBackend(context);
}

result_t<size_t> Bus_console::writeBackend(bus::OperationContext<AccessConfig>& context, data::Source* src, size_t len)
{
    return _delegate != nullptr ? writeOn(*_delegate, context, src, len)
                                : Bus_streaming::writeBackend(context, src, len);
}

result_t<size_t> Bus_console::readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len)
{
    return _delegate != nullptr ? readOn(*_delegate, context, dst, len) : Bus_streaming::readBackend(context, dst, len);
}

result_t<size_t> Bus_console::readableBytesBackend(bus::OperationContext<AccessConfig>& context)
{
    return _delegate != nullptr ? readableBytesOn(*_delegate, context) : Bus_streaming::readableBytesBackend(context);
}

result_t<size_t> Bus_console::rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (_transport == Transport::None) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if defined(ESP_PLATFORM)
    switch (_transport) {
        case Transport::Uart: {
            const uart_port_t port = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
            const int n            = uart_write_bytes(port, data, len);
            (void)uart_wait_tx_done(port, ::m5::hal::v2::detail::timeoutMsToTicks(timeout_ms));
            return (n > 0) ? static_cast<size_t>(n) : static_cast<size_t>(0);
        }
        default:
            return static_cast<size_t>(0);
    }
#elif !defined(_WIN32) && !defined(ARDUINO)
    (void)timeout_ms;
    size_t done = ::fwrite(data, 1, len, _file_out);
    ::fflush(_file_out);
    return done;
#else
    (void)data;
    (void)len;
    (void)timeout_ms;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
#endif
}

result_t<size_t> Bus_console::rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    if (_transport == Transport::None) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if defined(ESP_PLATFORM)
    switch (_transport) {
        case Transport::Uart: {
            const uart_port_t port = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
            const int n = uart_read_bytes(port, buf, len, ::m5::hal::v2::detail::timeoutMsToTicks(timeout_ms));
            return (n > 0) ? static_cast<size_t>(n) : static_cast<size_t>(0);
        }
        default:
            return static_cast<size_t>(0);
    }
#elif !defined(_WIN32) && !defined(ARDUINO)
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(_fd_in, &fds);
    struct timeval tv;
    tv.tv_sec  = static_cast<time_t>(timeout_ms / 1000);
    tv.tv_usec = static_cast<suseconds_t>((timeout_ms % 1000) * 1000);
    int ready  = ::select(_fd_in + 1, &fds, nullptr, nullptr, &tv);
    if (ready < 0) {
        if (errno == EINTR) {
            return static_cast<size_t>(0);
        }
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    if (ready == 0) {
        return static_cast<size_t>(0);
    }
    ssize_t n = ::read(_fd_in, buf, len);
    if (n < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            return static_cast<size_t>(0);
        }
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    return static_cast<size_t>(n);
#else
    (void)buf;
    (void)len;
    (void)timeout_ms;
    return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
#endif
}

result_t<size_t> Bus_console::rawReadableBytes()
{
    if (_transport == Transport::None) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if defined(ESP_PLATFORM)
    switch (_transport) {
        case Transport::Uart: {
            size_t available       = 0;
            const uart_port_t port = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
            if (uart_get_buffered_data_len(port, &available) != ESP_OK) {
                return static_cast<size_t>(0);
            }
            return available;
        }
        default:
            return static_cast<size_t>(0);
    }
#elif !defined(_WIN32) && !defined(ARDUINO)
    int avail = 0;
    if (::ioctl(_fd_in, FIONREAD, &avail) != 0 || avail < 0) {
        return static_cast<size_t>(0);
    }
    return static_cast<size_t>(avail);
#else
    return static_cast<size_t>(0);
#endif
}

}  // namespace m5::hal::v2::uart

#endif  // M5_HAL_UART_BUS_CONSOLE_INL_
