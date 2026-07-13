// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_UART_CONNECTION_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_UART_CONNECTION_HPP

#include "../../../../../hal/v2/remote/remote_connection.hpp"
#include "../../../remote/backend.hpp"
#include "../../../remote/detail_helpers.hpp"
#include "../../../remote/hal/gpio/gpio.hpp"
#include "../../../remote/session.hpp"
#include "../uart/uart.hpp"
#include "wire_dump.hpp"
#include "../../../../../hal/v2/data/tap.hpp"
#include "../../../../../hal/v2/diag.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>

#if M5HAL_FRAMEWORK_HAS_POSIX

#include <termios.h>
#include <unistd.h>

namespace m5::variants::frameworks::posix::hal::v2::remote {

namespace m5hal = ::m5::hal::v2;

class PosixUartConnection : public m5hal::remote::RemoteConnectionState, private m5hal::service::IService {
public:
    static m5hal::result_t<PosixUartConnection*> create(const char* port, const m5hal::remote::DeviceConfig& cfg)
    {
        auto* conn = new (std::nothrow) PosixUartConnection(cfg.baud_rate);
        if (conn == nullptr) {
            return m5::stl::make_unexpected(m5hal::error::error_t::OUT_OF_RESOURCE);
        }
        auto r = conn->connectAndHandshake(port, cfg);
        if (!r.has_value()) {
            auto err = r.error();
            delete conn;
            return m5::stl::make_unexpected(err);
        }
        return conn;
    }

    ~PosixUartConnection() override
    {
        M5HAL_DIAG("uart connection teardown");
        if (_session_handle != nullptr) {
            _session_handle->close();
        }
        destroyBackend();
        destroySession();
        (void)_bus.release();
    }

    m5hal::remote::RemoteSession& session() override
    {
        return *sessPtr();
    }

    std::shared_ptr<m5hal::remote::RemoteSessionHandle> sessionHandle() override
    {
        return _session_handle;
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
        return _gpio_owner != nullptr ? _gpio_owner->gpio() : nullptr;
    }

    std::unique_ptr<m5hal::remote::RemoteGpioOwner> releaseGpioOwnership() override
    {
        _gpio = nullptr;
        return std::move(_gpio_owner);
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
    explicit PosixUartConnection(uint32_t baud)
        : _uart_cfg{makeUartConfig(baud)},
          _rx{_bus, _uart_cfg},
          _tx{_bus, _uart_cfg},
          _wire_src{_tap_rx, m5hal::data::DataSpan{_rx_scratch, sizeof(_rx_scratch)}},
          _wire_snk{_tap_tx, m5hal::data::DataSpan{_tx_scratch, sizeof(_tx_scratch)}}
    {
    }

    m5hal::result_t<void> connectAndHandshake(const char* port, const m5hal::remote::DeviceConfig& cfg)
    {
        m5hal::uart::BusConfig_posix bus_cfg;
        bus_cfg.tx_coalesce_bytes = 4096;
        auto init                 = _bus.init(bus_cfg);
        if (!init.has_value()) {
            return m5::stl::make_unexpected(init.error());
        }
        auto open = _bus.open(port, _uart_cfg.baud_rate);
        if (open != m5hal::error::error_t::OK) {
            return m5::stl::make_unexpected(open);
        }
        M5HAL_DIAG("uart open ok port=%s baud=%u", port, static_cast<unsigned>(_uart_cfg.baud_rate));

        ::usleep(500 * 1000);
        (void)::tcflush(_bus.nativeHandle(), TCIOFLUSH);

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
        M5HAL_DIAG("uart hello ok proto_ver=%u has_gpio=%d bus_count=%zu gpio_port_count=%u gpio_pin_count=%u",
                   static_cast<unsigned>(_caps.proto_ver), _caps.has_gpio, _caps.bus_count,
                   static_cast<unsigned>(_caps.gpio_port_count), static_cast<unsigned>(_caps.gpio_pin_count));

        constructBackend();
        constructGPIO();
        return {};
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
        _sess_ok        = true;
        _session_handle = p->sharedHandle();
    }

    void destroySession()
    {
        if (_sess_ok) {
            _session_handle->close();
            sessPtr()->~RemoteSession();
            _sess_ok = false;
        }
    }

    void constructBackend()
    {
        destroyBackend();
        _backend_ptr = new (_backend_storage) m5hal::remote::RemoteBackend{_session_handle};
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
        using Owner = m5hal::remote::RemoteGpioOwnerModel<m5hal::remote::RemoteGPIO>;
        auto* owner = new (std::nothrow) Owner{_session_handle, static_cast<m5hal::types::gpio_slot_t>(0),
                                               _caps.gpio_port_count, _caps.gpio_pin_count};
        if (owner == nullptr) {
            return;
        }
        _gpio_owner.reset(owner);
        _gpio     = &owner->value();
        auto seed = _gpio->seedCache();
        if (!seed.has_value()) {
            _gpio_owner.reset();
            _gpio = nullptr;
            return;
        }
        sessPtr()->setEventHandler(&m5hal::remote::RemoteGPIO::onSessionEvent, _gpio);
        auto sub = _gpio->subscribeAll();
        if (!sub.has_value()) {
            sessPtr()->setEventHandler(nullptr, nullptr);
            _gpio_owner.reset();
            _gpio = nullptr;
            return;
        }
    }

    m5hal::service::ServicePoll serviceImpl(const m5hal::service::ServiceContext& ctx) override
    {
        (void)ctx;
        if (!_sess_ok) {
            return {m5hal::service::ServiceResult::Idle};
        }
        auto r = pollSession();
        if (!r.has_value()) {
            return {m5hal::service::ServiceResult::Error};
        }
        // Relative hint: poll again no sooner than ~1 ms from now.
        const auto delta = m5hal::service::nsecToFastTickCeil(1000000, m5hal::service::fastTickFrequencyHz());
        return {r.value() != 0 ? m5hal::service::ServiceResult::Progress : m5hal::service::ServiceResult::Idle, delta};
    }

    m5hal::uart::Bus_posix _bus;
    m5hal::uart::AccessConfig _uart_cfg;
    m5hal::uart::RxAccessor _rx;
    m5hal::uart::TxAccessor _tx;
    uint8_t _rx_scratch[4096];
    uint8_t _tx_scratch[4096];
    WireDump* _dump = WireDump::fromEnv();
    WireDumpWriter _dump_rx{_dump, "uart", '<'};
    WireDumpWriter _dump_tx{_dump, "uart", '>'};
    m5hal::data::TapReader _tap_rx{_rx, &_dump_rx};
    m5hal::data::TapWriter _tap_tx{_tx, &_dump_tx};
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
    std::unique_ptr<m5hal::remote::RemoteGpioOwner> _gpio_owner;
    std::shared_ptr<m5hal::remote::RemoteSessionHandle> _session_handle;
};

}  // namespace m5::variants::frameworks::posix::hal::v2::remote

#endif  // M5HAL_FRAMEWORK_HAS_POSIX

#endif
