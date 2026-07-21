// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_UART_CONNECTION_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_UART_CONNECTION_HPP

#include "../uart/uart.hpp"
#include "connection_core.hpp"

#include <cstdint>
#include <new>

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <termios.h>
#include <unistd.h>

namespace m5::variants::frameworks::posix::hal::v2::remote {

namespace m5hal = ::m5::hal::v2;

class PosixUartConnection : public m5hal::remote::RemoteConnectionState {
public:
    static m5hal::result_t<PosixUartConnection*> create(const char* port, const m5hal::remote::DeviceConfig& cfg)
    {
        auto* connection = new (std::nothrow) PosixUartConnection(cfg.baud_rate);
        if (connection == nullptr) {
            return m5::stl::make_unexpected(m5hal::error::error_t::OUT_OF_RESOURCE);
        }
        auto result = connection->connectAndHandshake(port, cfg);
        if (!result.has_value()) {
            const auto error = result.error();
            delete connection;
            return m5::stl::make_unexpected(error);
        }
        return connection;
    }

    ~PosixUartConnection() override
    {
        M5HAL_DIAG("uart connection teardown");
        _core.close();
        (void)_bus.close();
    }

    m5hal::remote::RemoteSession& session() override
    {
        return _core.session();
    }

    std::shared_ptr<m5hal::remote::RemoteSessionHandle> sessionHandle() override
    {
        return _core.sessionHandle();
    }

    m5hal::remote::RemoteBackend& backend() override
    {
        return _core.backend();
    }

    const m5hal::remote::Capabilities& capabilities() const override
    {
        return _core.capabilities();
    }

    const m5hal::gpio::IGPIO* gpio() const override
    {
        return _core.gpio();
    }

    std::unique_ptr<m5hal::remote::RemoteGpioOwner> releaseGpioOwnership() override
    {
        return _core.releaseGpioOwnership();
    }

    m5hal::service::IService* service() override
    {
        return _core.service();
    }

    void bindGpioEvents(m5hal::gpio::GPIOGroup& group, m5hal::types::gpio_slot_t slot) override
    {
        _core.bindGpioEvents(group, slot);
    }

private:
    explicit PosixUartConnection(uint32_t baud)
        : _uart_cfg{makeUartConfig(baud)}, _rx{_bus, _uart_cfg}, _tx{_bus, _uart_cfg}, _core{*this, _rx, _tx, "uart"}
    {
    }

    m5hal::result_t<void> connectAndHandshake(const char* port, const m5hal::remote::DeviceConfig& cfg)
    {
        auto init = _bus.init(m5hal::uart::BusConfig{},
                              m5hal::native::managed(m5hal::uart::NativePath{port}, m5hal::uart::NativeOptions{4096}));
        if (!init.has_value()) {
            return m5::stl::make_unexpected(init.error());
        }
        M5HAL_DIAG("uart open ok port=%s baud=%u", port, static_cast<unsigned>(_uart_cfg.baud_rate));

        ::usleep(500 * 1000);
        (void)::tcflush(_bus.nativeHandle(), TCIOFLUSH);
        return _core.initialize(cfg);
    }

    static m5hal::uart::AccessConfig makeUartConfig(uint32_t baud)
    {
        m5hal::uart::AccessConfig cfg;
        cfg.baud_rate             = baud;
        cfg.first_byte_timeout_ms = 2;
        cfg.inter_byte_timeout_ms = 2;
        cfg.write_timeout_ms      = 1000;
        return cfg;
    }

    m5hal::uart::Bus_posix _bus;
    m5hal::uart::AccessConfig _uart_cfg;
    m5hal::uart::RxAccessor _rx;
    m5hal::uart::TxAccessor _tx;
    PosixRemoteConnectionCore _core;
};

}  // namespace m5::variants::frameworks::posix::hal::v2::remote

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
