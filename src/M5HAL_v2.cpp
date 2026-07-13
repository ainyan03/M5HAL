// SPDX-License-Identifier: MIT

#include "M5HAL_v2.hpp"
#include "m5_hal/hal/v2/data/ring.inl"
#include "m5_hal/hal/v2/data/stream.inl"
#include "m5_hal/hal/v2/data/mux.inl"
#include "m5_hal/hal/v2/bus/bus.inl"
#include "m5_hal/hal/v2/bus/allocation_core.inl"
#include "m5_hal/hal/v2/bus/local_backend.inl"
#include "m5_hal/hal/v2/service/service.inl"
#include "m5_hal/hal/v2/gpio/group.inl"
#include "m5_hal/hal/v2/bytecode/bytecode.inl"
#include "m5_hal/hal/v2/frame/frame.inl"
#include "m5_hal/hal/v2/remote/server.inl"
#include "m5_hal/hal/v2/remote/server_bus_pool.inl"
#include "m5_hal/hal/v2/remote/server_adapter.inl"
#include "m5_hal/hal/v2/remote/server_handler.inl"
#include "m5_hal/hal/v2/remote/server_connection_wiring.inl"
#include "m5_hal/hal/v2/remote/remote_connection.inl"
#include "m5_hal/variants/frameworks/remote/session.inl"
#include "m5_hal/variants/frameworks/remote/backend.inl"
#include "m5_hal/variants/frameworks/remote/remote_transfer.inl"
#if M5HAL_FRAMEWORK_HAS_BSD_SOCKET
#include "m5_hal/variants/frameworks/bsd/hal/remote/tcp_server.inl"
#endif
#include "m5_hal/variants/frameworks/remote/hal/i2c/i2c.inl"
#include "m5_hal/variants/frameworks/remote/hal/i2s/i2s.inl"
#include "m5_hal/variants/frameworks/remote/hal/spi/spi.inl"
#include "m5_hal/variants/frameworks/remote/hal/uart/uart.inl"
#include "m5_hal/variants/frameworks/remote/hal/gpio/gpio.inl"
#include "m5_hal/hal/v2/i2c/i2c.inl"
#include "m5_hal/hal/v2/i2c/slave.inl"
#include "m5_hal/hal/v2/memory/pool.inl"
#include "m5_hal/hal/v2/memory/allocator.inl"
#include "m5_hal/hal/v2/spi/spi.inl"
#include "m5_hal/hal/v2/uart/bus_streaming.inl"
#include "m5_hal/hal/v2/uart/bus_console.inl"
#include "m5_hal/hal/v2/uart/uart.inl"
#include "m5_hal/hal/v2/i2s/i2s.inl"

#define M5HAL_STATIC_MACRO_PATH_IMPL M5HAL_STATIC_MACRO_CONCAT(M5HAL_V2_DETECTED_PLATFORM_VARIANT_PATH, hal.inl)

#if M5HAL_V2_DETECTED_PLATFORM_VARIANT_ID != M5HAL_V2_VARIANT_ID_NONE
#include M5HAL_STATIC_MACRO_PATH_IMPL
#endif

// Pull in the Arduino-backed implementation only when Arduino is available.
#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include "./m5_hal/variants/frameworks/arduino/hal.inl"
#endif

// ESP-IDF framework variant impl hub. Arduino-on-IDF may compile this
// alongside the arduino framework variant; scan order controls defaults.
#if M5HAL_FRAMEWORK_HAS_ESPIDF
#include "./m5_hal/variants/frameworks/espidf/hal.inl"
#endif

// POSIX host framework variant impl hub (termios serial). Compiled only on a
// plain POSIX host build (see frameworks/_checker.hpp).
#if M5HAL_FRAMEWORK_HAS_POSIX
#include "./m5_hal/variants/frameworks/posix/hal.inl"
#endif

// software variant: always compiled in (provides bit-bang fallback)
#include "./m5_hal/variants/frameworks/software/hal.inl"

// ----- M5HALCore ctor + M5_Hal definition -----
//
// The includes above leave this TU in a "winner-binding complete"
// context, so `m5::hal::v2::gpio::getGPIO()` (the variant-supplied
// MCU GPIO `IGPIO*`) is resolvable here. Closing the
// `M5HALCore::ctor` body and the `M5_Hal` reference definition in
// this single spot keeps the file-stem exception confined to
// `m5_hal.hpp` (no separate `src/m5_hal/hal/v2/m5_hal.cpp`).
//
// An `addGPIO` failure inside the ctor is an invariant break — a
// correctly bound variant must succeed — so we assert /
// fail fast.

#include <cassert>
#include <cstring>

#if defined(ESP_PLATFORM)
#include <esp_heap_caps.h>
#endif

#if defined(ESP_PLATFORM)
namespace {
uint32_t m5halEspidfHeapCaps(m5::hal::v2::memory::usage_t usage)
{
    using m5::hal::v2::memory::usage_t;
    switch (usage) {
        case usage_t::PersistentSlow:
            return MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

        case usage_t::Temp:
        case usage_t::Persistent:
        default:
            return MALLOC_CAP_DEFAULT;
    }
}

void* m5halEspidfMalloc(size_t size, m5::hal::v2::memory::usage_t usage)
{
    void* ptr = heap_caps_malloc(size, m5halEspidfHeapCaps(usage));
    if (ptr == nullptr && usage == m5::hal::v2::memory::usage_t::PersistentSlow) {
        ptr = heap_caps_malloc(size, MALLOC_CAP_DEFAULT);
    }
    return ptr;
}

void* m5halEspidfRealloc(void* ptr, size_t, size_t new_size, m5::hal::v2::memory::usage_t usage)
{
    void* next = heap_caps_realloc(ptr, new_size, m5halEspidfHeapCaps(usage));
    if (next == nullptr && usage == m5::hal::v2::memory::usage_t::PersistentSlow) {
        next = heap_caps_realloc(ptr, new_size, MALLOC_CAP_DEFAULT);
    }
    return next;
}

void m5halEspidfFree(void* ptr)
{
    heap_caps_free(ptr);
}
}  // namespace
#endif

// ---- remote connection header (needed by Hal::~Hal / session / capabilities) ----
#include "m5_hal/hal/v2/remote/remote_connection.hpp"

namespace m5 {
namespace hal {
M5HAL_INLINE_V2 namespace v2
{
    namespace {
    bus::LocalBackend* s_local_backend = nullptr;

    bus::LocalBackend* localBackendForConnect()
    {
        (void)getM5_Hal();
        return s_local_backend;
    }

    bool isLocalEndpoint(const char* endpoint)
    {
        return endpoint == nullptr || endpoint[0] == '\0' || ::strcmp(endpoint, "local") == 0;
    }
    }  // namespace

    // ---- Hal dtor + session/capabilities accessors (always defined) ----

    Hal::~Hal()
    {
        setBackendAll(nullptr);
        Gpio.clearWatchers();
        if (_connection != nullptr && _connection->service() != nullptr) {
            (void)Services.remove(*_connection->service());
        }
        if (_has_remote_gpio) {
            (void)Gpio.removeGPIO(_remote_gpio_slot);
            _has_remote_gpio = false;
        }
        delete _connection;
        _connection = nullptr;
        // Avoid recursive destruction through RemoteGpioOwner::next after a
        // long-running process has reconnected many times.
        while (_retired_remote_gpios != nullptr) {
            auto next             = std::move(_retired_remote_gpios->next);
            _retired_remote_gpios = std::move(next);
        }
    }

    remote::RemoteSession* Hal::session(void)
    {
        return _connection != nullptr ? &_connection->session() : nullptr;
    }

    const remote::Capabilities* Hal::capabilities(void) const
    {
        return _connection != nullptr ? &_connection->capabilities() : nullptr;
    }

    result_t<bool> Hal::pumpRemote(const remote::PumpConfig& cfg)
    {
        if (_connection == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        auto keepalive = _connection->keepalive(cfg);
        if (!keepalive.has_value()) {
            return m5::stl::make_unexpected(keepalive.error());
        }
        return Services.runOnce();
    }

    result_t<void> Hal::connect(const char* endpoint, const remote::DeviceConfig& cfg)
    {
        if (isLocalEndpoint(endpoint)) {
            auto* local = localBackendForConnect();
            if (local == nullptr) {
                return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
            }
            if (_connection == nullptr && _backend == local) {
                if (!Gpio.hasGPIO(0)) {
                    (void)Gpio.addGPIO(gpio::getGPIO(), 0);
                }
                return {};
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
            _connection = nullptr;
            setBackendAll(local);
            delete old_connection;
            if (!Gpio.hasGPIO(0)) {
                (void)Gpio.addGPIO(gpio::getGPIO(), 0);
            }
            return {};
        }

        if (::strncmp(endpoint, "uart:", 5) == 0) {
            return initUart(endpoint + 5, cfg);
        }
        if (::strncmp(endpoint, "tcp:", 4) == 0) {
            return initTcp(endpoint + 4, cfg);
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    result_t<void> Hal::init(void)
    {
        if (_backend != nullptr) {
            return {};
        }
        return connect(nullptr);
    }

    // ---- initUart / initTcp fallback (when no platform provides them) ----
#if !defined(M5HAL_DETAIL_REMOTE_UART_DEFINED_)
    result_t<void> Hal::initUart(const char*, const remote::DeviceConfig&)
    {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
#endif
#if !defined(M5HAL_DETAIL_REMOTE_TCP_DEFINED_)
    result_t<void> Hal::initTcp(const char*, const remote::DeviceConfig&)
    {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
#endif

    // The I2C and SPI local adapters are initialized here (not via in-class
    // initializers) because this is the only TU where the winner-binding scan
    // has run, so the variant-supplied backend factories are visible. _backend
    // is constructed first (declaration order); adapters share its registry.
    // _hal is last: it takes &_backend and its BusViews delegate through it.
    // The two hardware-backend guards are independent, so the init list is
    // spelled out per combination below.
#if defined(M5HAL_DETAIL_I2C_HAS_HARDWARE_BACKEND_) && defined(M5HAL_DETAIL_SPI_HAS_HARDWARE_BACKEND_)
    M5HALCore::M5HALCore()
        : _i2c_adapter{_backend.busRegistry(), &i2c::makeSoftwareBackendForI2C, &i2c::makeHardwareBackendForI2C,
                       i2c::hardwareControllerCountForI2C(), i2c::controllerTopologyForI2C()},
          _spi_adapter{_backend.busRegistry(), &spi::makeSoftwareBackendForSPI, &spi::makeHardwareBackendForSPI,
                       spi::hardwareControllerCountForSPI()},
          _hal{&_backend}
#elif defined(M5HAL_DETAIL_I2C_HAS_HARDWARE_BACKEND_)
    M5HALCore::M5HALCore()
        : _i2c_adapter{_backend.busRegistry(), &i2c::makeSoftwareBackendForI2C, &i2c::makeHardwareBackendForI2C,
                       i2c::hardwareControllerCountForI2C(), i2c::controllerTopologyForI2C()},
          _spi_adapter{_backend.busRegistry(), &spi::makeSoftwareBackendForSPI},
          _hal{&_backend}
#elif defined(M5HAL_DETAIL_SPI_HAS_HARDWARE_BACKEND_)
    M5HALCore::M5HALCore()
        : _i2c_adapter{_backend.busRegistry(), &i2c::makeSoftwareBackendForI2C},
          _spi_adapter{_backend.busRegistry(), &spi::makeSoftwareBackendForSPI, &spi::makeHardwareBackendForSPI,
                       spi::hardwareControllerCountForSPI()},
          _hal{&_backend}
#else
    M5HALCore::M5HALCore()
        : _i2c_adapter{_backend.busRegistry(), &i2c::makeSoftwareBackendForI2C},
          _spi_adapter{_backend.busRegistry(), &spi::makeSoftwareBackendForSPI},
          _hal{&_backend}
#endif
    {
        s_local_backend = &_backend;
        _backend.registerKind(_i2c_adapter);
        _backend.registerKind(_spi_adapter);
#if defined(ESP_PLATFORM)
        _hal.Memory.setFallback(&m5halEspidfMalloc, &m5halEspidfRealloc, &m5halEspidfFree);
#endif
        auto r = _hal.Gpio.addGPIO(gpio::getGPIO(), 0);
        assert(r.has_value() && "M5HALCore::ctor: slot 0 (MCU GPIO) registration failed");
        (void)r;
    }

    Hal& M5_Hal = getM5_Hal();
}
}  // namespace hal
}  // namespace m5
