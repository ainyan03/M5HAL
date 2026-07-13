// SPDX-License-Identifier: MIT
// Per-kind impl hub for the POSIX host framework variant. Included by
// M5HAL_v2.cpp when M5HAL_FRAMEWORK_HAS_POSIX is set; each subfile is
// self-contained for its kind. runtime is header-only and has no impl
// here. The UART kind honours the M5HAL_CONFIG_POSIX_UART opt-out.

#if M5HAL_CONFIG_POSIX_UART
#include "hal/uart/uart.inl"
#include "hal/uart/ports.inl"
#endif
#include "hal/tcp/tcp.inl"

// ---- Hal::initUart / Hal::initTcp (POSIX) ----------------------------------
#include "hal/remote/tcp_connection.hpp"

#if M5HAL_CONFIG_POSIX_UART
#include "hal/remote/uart_connection.hpp"
#endif

#define M5HAL_DETAIL_REMOTE_TCP_DEFINED_ 1
#if M5HAL_CONFIG_POSIX_UART
#define M5HAL_DETAIL_REMOTE_UART_DEFINED_ 1
#endif

namespace m5 {
namespace hal {
M5HAL_INLINE_V2 namespace v2
{
#if M5HAL_CONFIG_POSIX_UART
    result_t<void> Hal::initUart(const char* port, const remote::DeviceConfig& cfg)
    {
        namespace posix_remote = ::m5::variants::frameworks::posix::hal::v2::remote;
        auto conn              = posix_remote::PosixUartConnection::create(port, cfg);
        if (!conn.has_value()) {
            return m5::stl::make_unexpected(conn.error());
        }
        if (_has_remote_gpio) {
            (void)Gpio.removeGPIO(_remote_gpio_slot);
            _has_remote_gpio = false;
        }
        if (_connection != nullptr && _connection->service() != nullptr) {
            (void)Services.remove(*_connection->service());
        }
        auto* old_connection = _connection;
        if (old_connection != nullptr) {
            auto old_handle = old_connection->sessionHandle();
            if (old_handle) {
                old_handle->close();
            }
            retireRemoteGPIO();
        }
        _connection = conn.value();
        setBackendAll(&_connection->backend());
        delete old_connection;
        registerRemoteGPIO(_connection->gpio());
        if (_has_remote_gpio) {
            _connection->bindGpioEvents(Gpio, _remote_gpio_slot);
        }
        if (_connection->service() != nullptr) {
            (void)Services.add(*_connection->service());
        }
        return {};
    }
#endif

    result_t<void> Hal::initTcp(const char* endpoint, const remote::DeviceConfig& cfg)
    {
        namespace posix_remote = ::m5::variants::frameworks::posix::hal::v2::remote;
        auto conn              = posix_remote::PosixTcpConnection::create(endpoint, cfg);
        if (!conn.has_value()) {
            return m5::stl::make_unexpected(conn.error());
        }
        if (_has_remote_gpio) {
            (void)Gpio.removeGPIO(_remote_gpio_slot);
            _has_remote_gpio = false;
        }
        if (_connection != nullptr && _connection->service() != nullptr) {
            (void)Services.remove(*_connection->service());
        }
        auto* old_connection = _connection;
        if (old_connection != nullptr) {
            auto old_handle = old_connection->sessionHandle();
            if (old_handle) {
                old_handle->close();
            }
            retireRemoteGPIO();
        }
        _connection = conn.value();
        setBackendAll(&_connection->backend());
        delete old_connection;
        registerRemoteGPIO(_connection->gpio());
        if (_has_remote_gpio) {
            _connection->bindGpioEvents(Gpio, _remote_gpio_slot);
        }
        if (_connection->service() != nullptr) {
            (void)Services.add(*_connection->service());
        }
        return {};
    }
}
}  // namespace hal
}  // namespace m5
