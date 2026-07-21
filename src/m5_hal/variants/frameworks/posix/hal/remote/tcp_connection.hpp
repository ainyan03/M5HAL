// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_TCP_CONNECTION_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_TCP_CONNECTION_HPP

#include "../tcp/tcp.hpp"
#include "connection_core.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

#if M5HAL_FRAMEWORK_HAS_POSIX

namespace m5::variants::frameworks::posix::hal::v2::remote {

namespace m5hal = ::m5::hal::v2;

class PosixTcpConnection : public m5hal::remote::RemoteConnectionState {
public:
    static m5hal::result_t<PosixTcpConnection*> create(const char* endpoint, const m5hal::remote::DeviceConfig& cfg)
    {
        auto* connection = new (std::nothrow) PosixTcpConnection();
        if (connection == nullptr) {
            return m5::stl::make_unexpected(m5hal::error::error_t::OUT_OF_RESOURCE);
        }
        auto result = connection->connectAndHandshake(endpoint, cfg);
        if (!result.has_value()) {
            const auto error = result.error();
            delete connection;
            return m5::stl::make_unexpected(error);
        }
        return connection;
    }

    ~PosixTcpConnection() override
    {
        M5HAL_DIAG("tcp connection teardown");
        _core.close();
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
    PosixTcpConnection() : _core{*this, _stream, _stream, "tcp"}
    {
    }

    m5hal::result_t<void> connectAndHandshake(const char* endpoint, const m5hal::remote::DeviceConfig& cfg)
    {
        if (endpoint == nullptr || endpoint[0] == '\0') {
            return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
        }
        char host[256];
        uint16_t port = 0;
        auto parsed   = parseEndpoint(endpoint, host, sizeof(host), port);
        if (!parsed.has_value()) {
            return m5::stl::make_unexpected(parsed.error());
        }
        auto error = _stream.connect(host, port);
        // The session pump paces itself; a long first-byte wait here only adds
        // dead time after each decoded response (measured ~100 ms per request).
        _stream.read_timeout_ms = 1;
        if (error != m5hal::error::error_t::OK) {
            return m5::stl::make_unexpected(error);
        }
        M5HAL_DIAG("tcp connect ok host=%s port=%u", host, static_cast<unsigned>(port));
        return _core.initialize(cfg);
    }

    static m5hal::result_t<void> parseEndpoint(const char* endpoint, char* host, size_t host_size, uint16_t& port)
    {
        const char* colon = nullptr;
        for (const char* p = endpoint; *p != '\0'; ++p) {
            if (*p == ':') {
                colon = p;
            }
        }
        if (colon == nullptr || colon == endpoint) {
            return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
        }
        const size_t host_length = static_cast<size_t>(colon - endpoint);
        if (host_length >= host_size) {
            return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
        }
        ::memcpy(host, endpoint, host_length);
        host[host_length] = '\0';

        unsigned long parsed_port = 0;
        for (const char* digit = colon + 1; *digit != '\0'; ++digit) {
            if (*digit < '0' || *digit > '9') {
                return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
            }
            parsed_port = parsed_port * 10 + static_cast<unsigned long>(*digit - '0');
            if (parsed_port > 65535) {
                return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
            }
        }
        if (parsed_port == 0) {
            return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
        }
        port = static_cast<uint16_t>(parsed_port);
        return {};
    }

    tcp::TcpStream _stream;
    PosixRemoteConnectionCore _core;
};

}  // namespace m5::variants::frameworks::posix::hal::v2::remote

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
