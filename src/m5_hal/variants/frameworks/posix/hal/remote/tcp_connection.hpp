// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_TCP_CONNECTION_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_TCP_CONNECTION_HPP

#include "../../../../../hal/v2/remote/remote_connection.hpp"
#include "../../../remote/backend.hpp"
#include "../../../remote/detail_helpers.hpp"
#include "../../../remote/hal/gpio/gpio.hpp"
#include "../../../remote/session.hpp"
#include "../tcp/tcp.hpp"
#include "wire_dump.hpp"
#include "../../../../../hal/v2/data/tap.hpp"
#include "../../../../../hal/v2/diag.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

#if M5HAL_FRAMEWORK_HAS_POSIX

namespace m5::variants::frameworks::posix::hal::v2::remote {

namespace m5hal = ::m5::hal::v2;

class PosixTcpConnection : public m5hal::remote::RemoteConnectionState, private m5hal::service::IService {
public:
    static m5hal::result_t<PosixTcpConnection*> create(const char* endpoint, const m5hal::remote::DeviceConfig& cfg)
    {
        auto* conn = new (std::nothrow) PosixTcpConnection();
        if (conn == nullptr) {
            return m5::stl::make_unexpected(m5hal::error::error_t::OUT_OF_RESOURCE);
        }
        auto r = conn->connectAndHandshake(endpoint, cfg);
        if (!r.has_value()) {
            auto err = r.error();
            delete conn;
            return m5::stl::make_unexpected(err);
        }
        return conn;
    }

    ~PosixTcpConnection() override
    {
        M5HAL_DIAG("tcp connection teardown");
        delete _gpio;
        destroyBackend();
        destroySession();
    }

    m5hal::remote::RemoteSession& session() override
    {
        return *sessPtr();
    }

    m5hal::remote::RemoteBackend& backend() override
    {
        return *_backend_ptr;
    }

    const m5hal::remote::Capabilities& capabilities() const override
    {
        return _caps;
    }

    const m5hal::gpio::IGPIO* gpio() const override
    {
        return _gpio;
    }

    m5hal::service::IService* service() override
    {
        return this;
    }

    void bindGpioEvents(m5hal::gpio::GPIOGroup& group, m5hal::types::gpio_slot_t slot) override
    {
        if (_gpio != nullptr) {
            _gpio->bindEventGroup(&group, slot);
        }
    }

private:
    PosixTcpConnection()
        : _wire_src{_tap_rx, m5hal::data::DataSpan{_rx_scratch, sizeof(_rx_scratch)}},
          _wire_snk{_tap_tx, m5hal::data::DataSpan{_tx_scratch, sizeof(_tx_scratch)}}
    {
    }

    m5hal::result_t<void> connectAndHandshake(const char* endpoint, const m5hal::remote::DeviceConfig& cfg)
    {
        if (endpoint == nullptr || endpoint[0] == '\0') {
            return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
        }
        char host[256];
        uint16_t port = 0;
        auto pr       = parseEndpoint(endpoint, host, sizeof(host), port);
        if (!pr.has_value()) {
            return m5::stl::make_unexpected(pr.error());
        }
        auto err = _stream.connect(host, port);
        // The session pump paces itself; a long first-byte wait here only adds
        // dead time after each decoded response (measured ~100 ms per request).
        _stream.read_timeout_ms = 1;
        if (err != m5hal::error::error_t::OK) {
            return m5::stl::make_unexpected(err);
        }
        M5HAL_DIAG("tcp connect ok host=%s port=%u", host, static_cast<unsigned>(port));

        _enc.setAllocator(m5hal::memory::defaultAllocator());
        _dec.setAllocator(m5hal::memory::defaultAllocator());

        constructSession(cfg);

        auto hello_r = sessPtr()->hello();
        if (!hello_r.has_value()) {
            return m5::stl::make_unexpected(hello_r.error());
        }
        auto decoded = m5hal::remote::detail::decodeHelloCaps(sessPtr()->lastResponse());
        if (!decoded.has_value()) {
            return m5::stl::make_unexpected(decoded.error());
        }
        _caps = decoded.value();
        M5HAL_DIAG("tcp hello ok proto_ver=%u has_gpio=%d bus_count=%zu gpio_port_count=%u gpio_pin_count=%u",
                   static_cast<unsigned>(_caps.proto_ver), _caps.has_gpio, _caps.bus_count,
                   static_cast<unsigned>(_caps.gpio_port_count), static_cast<unsigned>(_caps.gpio_pin_count));

        constructBackend();
        constructGPIO();
        return {};
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
        size_t hlen = static_cast<size_t>(colon - endpoint);
        if (hlen >= host_size) {
            return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
        }
        ::memcpy(host, endpoint, hlen);
        host[hlen]      = '\0';
        unsigned long p = 0;
        for (const char* s = colon + 1; *s != '\0'; ++s) {
            if (*s < '0' || *s > '9') {
                return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
            }
            p = p * 10 + static_cast<unsigned long>(*s - '0');
        }
        if (p == 0 || p > 65535) {
            return m5::stl::make_unexpected(m5hal::error::error_t::INVALID_ARGUMENT);
        }
        port = static_cast<uint16_t>(p);
        return {};
    }

    m5hal::remote::RemoteSession* sessPtr()
    {
        return reinterpret_cast<m5hal::remote::RemoteSession*>(_sess_storage);
    }

    void constructSession(const m5hal::remote::DeviceConfig& cfg)
    {
        destroySession();
        auto* p = new (_sess_storage) m5hal::remote::RemoteSession{_enc, _dec, _wire_src, _wire_snk};
        m5hal::remote::RemoteSession::Config sess_cfg;
        sess_cfg.response_timeout_ms = cfg.response_timeout_ms;
        p->setConfig(sess_cfg);
        _sess_ok = true;
    }

    void destroySession()
    {
        if (_sess_ok) {
            sessPtr()->~RemoteSession();
            _sess_ok = false;
        }
    }

    void constructBackend()
    {
        destroyBackend();
        _backend_ptr = new (_backend_storage) m5hal::remote::RemoteBackend{*sessPtr()};
        _backend_ptr->setCapabilities(_caps);
        _backend_ok = true;
    }

    void destroyBackend()
    {
        if (_backend_ok) {
            _backend_ptr->~RemoteBackend();
            _backend_ptr = nullptr;
            _backend_ok  = false;
        }
    }

    void constructGPIO()
    {
        if (!_caps.has_gpio || _caps.gpio_port_count == 0) {
            return;
        }
        _gpio =
            new (std::nothrow) m5hal::remote::RemoteGPIO{*sessPtr(), 0, _caps.gpio_port_count, _caps.gpio_pin_count};
        if (_gpio == nullptr) {
            return;
        }
        auto seed = _gpio->seedCache();
        if (!seed.has_value()) {
            delete _gpio;
            _gpio = nullptr;
            return;
        }
        sessPtr()->setEventHandler(&m5hal::remote::RemoteGPIO::onSessionEvent, _gpio);
        auto sub = _gpio->subscribeAll();
        if (!sub.has_value()) {
            sessPtr()->setEventHandler(nullptr, nullptr);
            delete _gpio;
            _gpio = nullptr;
            return;
        }
    }

    m5hal::service::ServicePoll serviceImpl(const m5hal::service::ServiceContext& ctx) override
    {
        if (!_sess_ok) {
            return {m5hal::service::ServiceResult::Idle};
        }
        auto r = pollSession();
        if (!r.has_value()) {
            return {m5hal::service::ServiceResult::Error};
        }
        auto due = static_cast<m5hal::service::fast_tick_t>(
            ctx.now_tick + m5hal::service::nsecToFastTickCeil(1000000, m5hal::service::fastTickFrequencyHz()));
        return {r.value() != 0 ? m5hal::service::ServiceResult::Progress : m5hal::service::ServiceResult::Idle, due};
    }

    tcp::TcpStream _stream;
    uint8_t _rx_scratch[4096];
    uint8_t _tx_scratch[4096];
    WireDump* _dump = WireDump::fromEnv();
    WireDumpWriter _dump_rx{_dump, "tcp", '<'};
    WireDumpWriter _dump_tx{_dump, "tcp", '>'};
    m5hal::data::TapReader _tap_rx{_stream, &_dump_rx};
    m5hal::data::TapWriter _tap_tx{_stream, &_dump_tx};
    m5hal::data::StreamSource _wire_src;
    m5hal::data::StreamSink _wire_snk;
    m5hal::data::MuxFrameEncoder _enc;
    m5hal::data::MuxFrameDecoder _dec;
    alignas(m5hal::remote::RemoteSession) uint8_t _sess_storage[sizeof(m5hal::remote::RemoteSession)];
    bool _sess_ok = false;
    alignas(m5hal::remote::RemoteBackend) uint8_t _backend_storage[sizeof(m5hal::remote::RemoteBackend)];
    m5hal::remote::RemoteBackend* _backend_ptr = nullptr;
    bool _backend_ok                           = false;
    m5hal::remote::Capabilities _caps;
    m5hal::remote::RemoteGPIO* _gpio = nullptr;
};

}  // namespace m5::variants::frameworks::posix::hal::v2::remote

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
