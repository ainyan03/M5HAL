// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_CONNECTION_CORE_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_POSIX_HAL_REMOTE_CONNECTION_CORE_HPP

#include "../../../../../hal/v2/data/tap.hpp"
#include "../../../../../hal/v2/diag.hpp"
#include "../../../../../hal/v2/remote/remote_connection.hpp"
#include "../../../remote/backend.hpp"
#include "../../../remote/detail_helpers.hpp"
#include "../../../remote/hal/gpio/gpio.hpp"
#include "../../../remote/session.hpp"
#include "wire_dump.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

#if M5HAL_FRAMEWORK_HAS_POSIX

namespace m5::variants::frameworks::posix::hal::v2::remote {

namespace m5hal = ::m5::hal::v2;

/*! @brief Transport-neutral client protocol lifecycle for POSIX connections.

  TCP and UART own different byte streams, but session construction, hello,
  backend/GPIO projection, polling, and teardown must remain identical.  The
  transport object therefore only opens its stream and delegates that shared
  lifecycle here.
 */
class PosixRemoteConnectionCore final : private m5hal::service::IService {
public:
    PosixRemoteConnectionCore(m5hal::remote::RemoteConnectionState& owner, m5hal::data::StreamReader& reader,
                              m5hal::data::StreamWriter& writer, const char* transport_name)
        : _owner{owner},
          _transport_name{transport_name},
          _dump_rx{_dump, transport_name, '<'},
          _dump_tx{_dump, transport_name, '>'},
          _tap_rx{reader, &_dump_rx},
          _tap_tx{writer, &_dump_tx},
          _wire_src{_tap_rx, m5hal::data::DataSpan{_rx_scratch, sizeof(_rx_scratch)}},
          _wire_snk{_tap_tx, m5hal::data::DataSpan{_tx_scratch, sizeof(_tx_scratch)}}
    {
    }

    ~PosixRemoteConnectionCore()
    {
        close();
    }

    PosixRemoteConnectionCore(const PosixRemoteConnectionCore&)            = delete;
    PosixRemoteConnectionCore& operator=(const PosixRemoteConnectionCore&) = delete;

    m5hal::result_t<void> initialize(const m5hal::remote::DeviceConfig& cfg)
    {
        constructSession(cfg);
        if (_session_handle == nullptr) {
            return m5::stl::make_unexpected(m5hal::error::error_t::OUT_OF_RESOURCE);
        }

        auto hello_r = sessPtr()->hello();
        if (!hello_r.has_value()) {
            return m5::stl::make_unexpected(hello_r.error());
        }
        auto decoded = m5hal::remote::detail::decodeHelloCaps(sessPtr()->lastResponse());
        if (!decoded.has_value()) {
            return m5::stl::make_unexpected(decoded.error());
        }
        _caps = decoded.value();
        M5HAL_DIAG("%s hello ok proto_ver=%u has_gpio=%d bus_count=%zu gpio_port_count=%u gpio_pin_count=%u",
                   _transport_name, static_cast<unsigned>(_caps.proto_ver), _caps.has_gpio, _caps.bus_count,
                   static_cast<unsigned>(_caps.gpio_port_count), static_cast<unsigned>(_caps.gpio_pin_count));

        constructBackend();
        constructGPIO();
        return {};
    }

    void close()
    {
        if (_session_handle != nullptr) {
            (void)_session_handle->close();
        }
        destroyBackend();
        destroySession();
    }

    m5hal::remote::RemoteSession& session()
    {
        return *sessPtr();
    }

    std::shared_ptr<m5hal::remote::RemoteSessionHandle> sessionHandle()
    {
        return _session_handle;
    }

    m5hal::remote::RemoteBackend& backend()
    {
        return *_backend_ptr;
    }

    const m5hal::remote::Capabilities& capabilities() const
    {
        return _caps;
    }

    const m5hal::gpio::IGPIO* gpio() const
    {
        return _gpio_owner != nullptr ? _gpio_owner->gpio() : nullptr;
    }

    std::unique_ptr<m5hal::remote::RemoteGpioOwner> releaseGpioOwnership()
    {
        _gpio = nullptr;
        return std::move(_gpio_owner);
    }

    m5hal::service::IService* service()
    {
        return this;
    }

    void bindGpioEvents(m5hal::gpio::GPIOGroup& group, m5hal::types::gpio_slot_t slot)
    {
        if (_gpio != nullptr) {
            _gpio->bindEventGroup(&group, slot);
        }
    }

private:
    m5hal::remote::RemoteSession* sessPtr()
    {
        return reinterpret_cast<m5hal::remote::RemoteSession*>(_sess_storage);
    }

    void constructSession(const m5hal::remote::DeviceConfig& cfg)
    {
        destroySession();
        auto* session = new (_sess_storage) m5hal::remote::RemoteSession{_enc, _dec, _wire_src, _wire_snk};
        m5hal::remote::RemoteSession::Config session_cfg;
        session_cfg.response_timeout_ms = cfg.response_timeout_ms;
        session->setConfig(session_cfg);
        _session_ok     = true;
        _session_handle = session->sharedHandle();
    }

    void destroySession()
    {
        if (_session_ok) {
            if (_session_handle != nullptr) {
                (void)_session_handle->close();
            }
            sessPtr()->~RemoteSession();
            _session_ok = false;
            _session_handle.reset();
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
        auto subscribe = _gpio->subscribeAll();
        if (!subscribe.has_value()) {
            sessPtr()->setEventHandler(nullptr, nullptr);
            _gpio_owner.reset();
            _gpio = nullptr;
        }
    }

    m5hal::service::ServicePoll serviceImpl(const m5hal::service::ServiceContext& ctx) override
    {
        (void)ctx;
        if (!_session_ok) {
            return {m5hal::service::ServiceResult::Idle};
        }
        auto result = _owner.pollSession();
        if (!result.has_value()) {
            return {m5hal::service::ServiceResult::Error};
        }
        const auto delta = m5hal::service::nsecToFastTickCeil(1000000, m5hal::service::fastTickFrequencyHz());
        return {result.value() != 0 ? m5hal::service::ServiceResult::Progress : m5hal::service::ServiceResult::Idle,
                delta};
    }

    m5hal::remote::RemoteConnectionState& _owner;
    [[maybe_unused]] const char* _transport_name;
    uint8_t _rx_scratch[4096];
    uint8_t _tx_scratch[4096];
    WireDump* _dump = WireDump::fromEnv();
    WireDumpWriter _dump_rx;
    WireDumpWriter _dump_tx;
    m5hal::data::TapReader _tap_rx;
    m5hal::data::TapWriter _tap_tx;
    m5hal::data::StreamSource _wire_src;
    m5hal::data::StreamSink _wire_snk;
    m5hal::data::MuxFrameEncoder _enc;
    m5hal::data::MuxFrameDecoder _dec;
    alignas(m5hal::remote::RemoteSession) uint8_t _sess_storage[sizeof(m5hal::remote::RemoteSession)];
    bool _session_ok = false;
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
