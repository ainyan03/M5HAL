// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_M5_HAL_HPP_
#define M5_HAL_HAL_V2_M5_HAL_HPP_

// Hal — the public HAL facade type.
//
// `Hal` exposes ResourceDomain-owned GPIOGroup, ServiceRunner, memory
// Allocator, and registry-backed per-kind BusViews behind a backend pointer. Callers
// write `void scan(Hal& hal) { hal.I2C.acquire(...); }` and the
// same function works for both local and remote backends.
//
// Three construction modes:
//   - Default local: M5_Hal bootstraps its own ResourceDomain.
//   - Isolated/shared local: Hal() owns an independent domain; Hal(domain)
//     shares the supplied domain. init()/connect("local") binds its backend.
//   - Remote: user creates `Hal remote;` then `remote.connect(endpoint)`.
//     connect establishes the transport, creates a RemoteBackend, and
//     wires BusViews to it. After connect, `remote.I2C.acquire()` creates
//     proxy buses on the remote peer.
//
// M5HALCore — internal singleton bootstrapping the default Hal facade.
//
// The Hal is exposed through `getM5_Hal()`
// and `M5_Hal`.
//
// Instance access:
//   - `getM5_Hal()` (Magic Static; first-use lazy; thread-safe by
//     C++17 §6.7).
//   - `extern Hal& M5_Hal` — eager-initialized alias used by the
//     public `M5_Hal.Gpio.foo()` syntax. Defined in M5HAL_v2.cpp.
//
// Important caveat:
//   - `M5_Hal` is a public syntax sugar with eager initialization.
//     Use it only from caller-side code (setup / loop / function-local
//     scopes). From namespace-scope initializers or another library's
//     global constructor, ALWAYS go through `getM5_Hal()` to avoid the
//     static initialization order fiasco.
//
// Placement: an exception to the 1:1 namespace/filename rule. The
// file stem is `m5_hal`, the namespace is `m5::hal::v2`, the class
// is `Hal` / `M5HALCore`. See spec/architecture.md, the namespace-rule
// exception clause.

#include "../../../m5_hal_config.hpp"  // M5HAL_INLINE_V2

#include "bus/hal_backend.hpp"
#include "bus/local_backend.hpp"
#include "bus/registry.hpp"
#include "gpio/group.hpp"
#include "i2c/i2c.hpp"
#include "i2s/i2s.hpp"
#include "memory/allocator.hpp"
#include "remote/remote_connection.hpp"
#include "resource_domain.hpp"
#include "service/service.hpp"
#include "spi/spi.hpp"
#include "uart/uart.hpp"

namespace m5 {
namespace hal {
M5HAL_INLINE_V2 namespace v2
{
    class M5HALCore;

    namespace remote {
    struct RemoteConnectionState;
    }
    namespace detail {
    struct LocalConnectionState;
    }

    /*!
      @brief Public HAL facade — local and remote share this surface.

      A `Hal` exposes the resource objects co-owned by its `ResourceDomain`
      (GPIO, service runner, memory allocator, registry) and per-kind bus views
      behind an `IHalBackend*`. A local binding is owned per Hal; `M5HALCore`
      only bootstraps the default `M5_Hal`. Remote instances are backed by
      `RemoteBackend`.
      User code writes `void scan(Hal& hal)` and works with either:
      @code
      void scanI2C(Hal& hal) {
          i2c::BusConfig cfg{i2c::Scl{22}, i2c::Sda{21}};
          auto bus = hal.I2C.acquire(cfg);
          // works identically for local and remote
      }
      scanI2C(M5_Hal);      // local
      scanI2C(remote_hal);  // remote (after connect/initUart/initTcp)
      @endcode

      Copy and move are disabled: the local singleton must not move, and
      remote instances hold non-trivial state (connections, proxy caches).
      Use `Hal&` for passing and `std::unique_ptr<Hal>` for ownership
      (e.g. `std::vector<std::unique_ptr<Hal>>`).
     */
    class Hal {
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
    public:
        enum class RemoteAdoptionStep : uint8_t {
            RemoveOldService,
            RemoveOldGpio,
            AddNewGpio,
            AddNewService,
            RemoveNewGpioRollback,
            AddOldGpioRollback,
            AddOldServiceRollback,
        };

    private:
        struct RemoteAdoptionFault {
            RemoteAdoptionStep step = RemoteAdoptionStep::RemoveOldService;
            error::error_t error    = error::error_t::OK;
        };
        RemoteAdoptionFault _remote_adoption_faults[2]{};
        size_t _remote_adoption_fault_count = 0;
        size_t _remote_adoption_fault_next  = 0;
#endif
        ResourceDomain _domain;
        bus::IHalBackend* _backend;
        std::shared_ptr<detail::LocalConnectionState> _local_connection;
        remote::RemoteConnectionState* _connection = nullptr;
        // A Pin/PortAccess is a non-owning value handle.  Preserve only the
        // disconnected GPIO ports until this Hal dies; never retain the old
        // connection, session object, or transport buffers.
        std::unique_ptr<remote::RemoteGpioOwner> _retired_remote_gpios;

    public:
        gpio::GPIOGroup& Gpio;
        service::ServiceRunner& Services;
        memory::Allocator& Memory;

        /*!
          @name Per-kind bus access.

          All kinds are backend-delegating views:
          `hal.I2C.acquire(cfg)` / `hal.SPI.acquire(cfg)` /
          `hal.UART.acquire(cfg)` / `hal.I2S.acquire(cfg)` / `hal.PDM.acquire(cfg)` intern the bus
          for `cfg`'s pins and return a `shared_ptr` — one physical wiring,
          one shared instance.
          @{
         */
        i2c::BusView I2C;
        spi::BusView SPI;
        uart::BusView UART;
        i2s::BusView I2S;
        pdm::BusView PDM;
        /*! @} */

        Hal()
            : _domain{}, _backend{nullptr}, Gpio{_domain.gpio()}, Services{_domain.services()}, Memory{_domain.memory()}
        {
        }

        explicit Hal(const ResourceDomain& domain)
            : _domain{domain},
              _backend{nullptr},
              Gpio{_domain.gpio()},
              Services{_domain.services()},
              Memory{_domain.memory()}
        {
        }

        ~Hal();

        bus::IHalBackend* backend(void)
        {
            return _backend;
        }

        const ResourceDomain& resourceDomain(void) const
        {
            return _domain;
        }

        /*!
          @name Remote connection (exclusive binding).

          Establish a transport to a remote peer and bind ALL kind views
          of this `Hal` to that peer's backend (any previous binding —
          including host-local buses on a PC build — is replaced; one
          `Hal` is the window to exactly one device). `connect()` accepts
          `nullptr`, `""`, or `"local"` for the process-local backend;
          `"uart:<path>"` for serial; and `"tcp:<host>:<port>"` for TCP.
          `init()` is an idempotent ensure-bound operation: it keeps any
          existing local or remote binding, and binds local only when this
          `Hal` is still unbound. To use host-local buses and a remote
          device at the same time, construct a second `Hal` for the remote
          side. The defaulted `DeviceConfig` is sufficient for a standard
          connection. `port` is a serial device path (e.g. "/dev/ttyUSB0",
          "COM5"); `endpoint` is "host:port" (e.g. "192.168.1.10:3333").

          A remote `gpio::Pin` / `GPIOGroup::PortAccess` obtained before a
          successful reconnect is never rebound to the new peer. Its port
          storage remains valid until this `Hal` is destroyed, but its old
          session is closed: reads return the final cached level, while writes
          and mode changes are no-ops.
          @{
         */
        [[nodiscard]] result_t<void> connect(const char* endpoint = nullptr, const remote::DeviceConfig& cfg = {});
        [[nodiscard]] result_t<void> init(void);
        result_t<void> initUart(const char* port, const remote::DeviceConfig& cfg = {});
        result_t<void> initTcp(const char* endpoint, const remote::DeviceConfig& cfg = {});
        /*! @} */

        /*! @brief Whether this Hal currently owns a remote connection.

          This is a binding-kind snapshot, not a transport liveness probe.
          It is false for both an unbound Hal and a Hal bound to the local
          backend. A transport failure does not destroy the connection
          object, so operation `result_t` values remain authoritative.
         */
        bool hasRemoteConnection(void) const
        {
            return _connection != nullptr;
        }

        // Borrowed low-level escape hatch. The pointer is valid only until the
        // next connect/initUart/initTcp call or this Hal's destruction. Bus
        // proxies use an internal shared session handle and remain safely
        // callable after that point (they report CLOSED instead of retaining
        // this connection).
        remote::RemoteSession* session(void);
        const remote::Capabilities* capabilities(void) const;
        result_t<bool> pumpRemote(const remote::PumpConfig& cfg = {});

#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
        void setRemoteAdoptionFaults(RemoteAdoptionStep first, error::error_t first_error, RemoteAdoptionStep second,
                                     error::error_t second_error)
        {
            _remote_adoption_faults[0]   = {first, first_error};
            _remote_adoption_faults[1]   = {second, second_error};
            _remote_adoption_fault_count = 2;
            _remote_adoption_fault_next  = 0;
        }
#endif

        types::gpio_slot_t remoteGpioSlot(void) const
        {
            return _remote_gpio_slot;
        }
        bool hasRemoteGpio(void) const
        {
            return _has_remote_gpio;
        }

        Hal(const Hal&)            = delete;
        Hal& operator=(const Hal&) = delete;
        Hal(Hal&&)                 = delete;
        Hal& operator=(Hal&&)      = delete;

    protected:
        explicit Hal(bus::IHalBackend* backend)
            : _domain{},
              _backend{backend},
              Gpio{_domain.gpio()},
              Services{_domain.services()},
              Memory{_domain.memory()},
              I2C{backend},
              SPI{backend},
              UART{backend},
              I2S{backend},
              PDM{backend}
        {
        }

        friend class M5HALCore;

    private:
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
        error::error_t takeRemoteAdoptionFault(RemoteAdoptionStep step)
        {
            if (_remote_adoption_fault_next < _remote_adoption_fault_count &&
                _remote_adoption_faults[_remote_adoption_fault_next].step == step) {
                return _remote_adoption_faults[_remote_adoption_fault_next++].error;
            }
            return error::error_t::OK;
        }
#endif
        template <typename Operation>
        result_t<void> runRemoteAdoptionStep(
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
            RemoteAdoptionStep step,
#endif
            Operation&& operation)
        {
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
            const auto injected = takeRemoteAdoptionFault(step);
            if (error::isError(injected)) {
                return m5::stl::make_unexpected(injected);
            }
#endif
            return operation();
        }
        void setBackendAll(bus::IHalBackend* backend)
        {
            _backend = backend;
            I2C.setBackend(backend);
            SPI.setBackend(backend);
            UART.setBackend(backend);
            I2S.setBackend(backend);
            PDM.setBackend(backend);
        }
        // Select the lowest slot before touching the current connection. The
        // current remote slot is reclaimable because connection replacement
        // removes it before committing the new registration.
        result_t<bool> preflightRemoteGPIO(const gpio::IGPIO* gpio, types::gpio_slot_t& selected_slot) const
        {
            if (gpio == nullptr) {
                return false;
            }

            const uint16_t pin_count = gpio->getPinCount();
            if (pin_count == 0 || pin_count > 256) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }

            size_t occupied = 0;
            bool found      = false;
            for (size_t candidate = 0; candidate < gpio::GPIOGroup::kSlotCount; ++candidate) {
                const auto slot                = static_cast<types::gpio_slot_t>(candidate);
                const bool reclaiming_old_slot = _has_remote_gpio && slot == _remote_gpio_slot;
                if (Gpio.hasGPIO(slot) && !reclaiming_old_slot) {
                    ++occupied;
                    continue;
                }
                if (!found) {
                    selected_slot = slot;
                    found         = true;
                }
            }

            if (!found || occupied >= gpio::GPIOGroup::kMaxEntries) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }
            return true;
        }
        result_t<void> registerRemoteGPIO(const gpio::IGPIO* gpio, types::gpio_slot_t slot)
        {
            if (gpio == nullptr) {
                return {};
            }
            auto r = Gpio.addGPIO(gpio, slot);
            if (!r.has_value()) {
                return m5::stl::make_unexpected(r.error());
            }
            _remote_gpio_slot = slot;
            _has_remote_gpio  = true;
            return {};
        }
        void retireRemoteGPIO()
        {
            if (_connection == nullptr) {
                return;
            }
            auto owner = _connection->releaseGpioOwnership();
            if (owner != nullptr) {
                owner->next           = std::move(_retired_remote_gpios);
                _retired_remote_gpios = std::move(owner);
            }
        }
        void retainRemoteGPIOOwner(remote::RemoteConnectionState* connection)
        {
            if (connection == nullptr) {
                return;
            }
            auto owner = connection->releaseGpioOwnership();
            if (owner != nullptr) {
                owner->next           = std::move(_retired_remote_gpios);
                _retired_remote_gpios = std::move(owner);
            }
        }
        void quarantineRemoteAdoption(remote::RemoteConnectionState* old_connection,
                                      std::unique_ptr<remote::RemoteConnectionState>& next_connection)
        {
            // A rollback failure means neither registration set can be
            // represented truthfully. Stop both producers, close both
            // transports, and publish one unbound facade. Cleanup is best
            // effort, but GPIO owners are retained even when an entry cannot
            // be removed so GPIOGroup never contains a dangling pointer.
            if (old_connection != nullptr && old_connection->service() != nullptr) {
                (void)Services.remove(*old_connection->service());
            }
            if (next_connection != nullptr && next_connection->service() != nullptr) {
                (void)Services.remove(*next_connection->service());
            }
            if (_has_remote_gpio) {
                (void)Gpio.removeGPIO(_remote_gpio_slot);
            }
            _has_remote_gpio = false;

            if (old_connection != nullptr) {
                auto handle = old_connection->sessionHandle();
                if (handle != nullptr) {
                    (void)handle->close();
                }
            }
            if (next_connection != nullptr) {
                auto handle = next_connection->sessionHandle();
                if (handle != nullptr) {
                    (void)handle->close();
                }
            }
            retainRemoteGPIOOwner(old_connection);
            retainRemoteGPIOOwner(next_connection.get());
            _connection = nullptr;
            setBackendAll(nullptr);
            delete old_connection;
        }
        result_t<void> adoptRemoteConnection(std::unique_ptr<remote::RemoteConnectionState> connection)
        {
            if (connection == nullptr) {
                return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
            }

            types::gpio_slot_t next_gpio_slot = 0;
            auto gpio_plan                    = preflightRemoteGPIO(connection->gpio(), next_gpio_slot);
            if (!gpio_plan.has_value()) {
                return m5::stl::make_unexpected(gpio_plan.error());
            }

            auto* old_connection            = _connection;
            const bool restore_remote_gpio  = _has_remote_gpio;
            const auto old_remote_gpio_slot = _remote_gpio_slot;
            const auto* old_remote_gpio =
                restore_remote_gpio && old_connection != nullptr ? old_connection->gpio() : nullptr;

            if (old_connection != nullptr && old_connection->service() != nullptr) {
                auto removed = runRemoteAdoptionStep(
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
                    RemoteAdoptionStep::RemoveOldService,
#endif
                    [&]() { return Services.remove(*old_connection->service()); });
                if (!removed.has_value()) {
                    return m5::stl::make_unexpected(removed.error());
                }
            }
            if (_has_remote_gpio) {
                auto removed = runRemoteAdoptionStep(
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
                    RemoteAdoptionStep::RemoveOldGpio,
#endif
                    [&]() { return Gpio.removeGPIO(_remote_gpio_slot); });
                if (!removed.has_value()) {
                    if (old_connection != nullptr && old_connection->service() != nullptr) {
                        auto restored = runRemoteAdoptionStep(
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
                            RemoteAdoptionStep::AddOldServiceRollback,
#endif
                            [&]() { return Services.add(*old_connection->service()); });
                        if (!restored.has_value()) {
                            const auto original = removed.error();
                            quarantineRemoteAdoption(old_connection, connection);
                            return m5::stl::make_unexpected(original);
                        }
                    }
                    return m5::stl::make_unexpected(removed.error());
                }
                _has_remote_gpio = false;
            }

            auto restore_old_registration = [&]() -> result_t<void> {
                if (_has_remote_gpio) {
                    auto removed = runRemoteAdoptionStep(
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
                        RemoteAdoptionStep::RemoveNewGpioRollback,
#endif
                        [&]() { return Gpio.removeGPIO(_remote_gpio_slot); });
                    if (!removed.has_value()) {
                        return m5::stl::make_unexpected(removed.error());
                    }
                    _has_remote_gpio = false;
                }
                if (restore_remote_gpio) {
                    auto restored = runRemoteAdoptionStep(
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
                        RemoteAdoptionStep::AddOldGpioRollback,
#endif
                        [&]() { return Gpio.addGPIO(old_remote_gpio, old_remote_gpio_slot); });
                    if (!restored.has_value()) {
                        return m5::stl::make_unexpected(restored.error());
                    }
                    _remote_gpio_slot = old_remote_gpio_slot;
                    _has_remote_gpio  = true;
                }
                if (old_connection != nullptr && old_connection->service() != nullptr) {
                    auto restored = runRemoteAdoptionStep(
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
                        RemoteAdoptionStep::AddOldServiceRollback,
#endif
                        [&]() { return Services.add(*old_connection->service()); });
                    if (!restored.has_value()) {
                        return m5::stl::make_unexpected(restored.error());
                    }
                }
                return {};
            };

            if (gpio_plan.value()) {
                auto registration = runRemoteAdoptionStep(
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
                    RemoteAdoptionStep::AddNewGpio,
#endif
                    [&]() { return registerRemoteGPIO(connection->gpio(), next_gpio_slot); });
                if (!registration.has_value()) {
                    auto restored = restore_old_registration();
                    if (!restored.has_value()) {
                        const auto original = registration.error();
                        quarantineRemoteAdoption(old_connection, connection);
                        return m5::stl::make_unexpected(original);
                    }
                    return m5::stl::make_unexpected(registration.error());
                }
            }
            if (_has_remote_gpio) {
                connection->bindGpioEvents(Gpio, _remote_gpio_slot);
            }
            if (connection->service() != nullptr) {
                auto added = runRemoteAdoptionStep(
#if defined(M5HAL_TEST_REMOTE_ADOPTION_FAULTS)
                    RemoteAdoptionStep::AddNewService,
#endif
                    [&]() { return Services.add(*connection->service()); });
                if (!added.has_value()) {
                    auto restored = restore_old_registration();
                    if (!restored.has_value()) {
                        const auto original = added.error();
                        quarantineRemoteAdoption(old_connection, connection);
                        return m5::stl::make_unexpected(original);
                    }
                    return m5::stl::make_unexpected(added.error());
                }
            }

            if (old_connection != nullptr) {
                auto old_handle = old_connection->sessionHandle();
                if (old_handle != nullptr) {
                    (void)old_handle->close();
                }
                retireRemoteGPIO();
            }
            _connection = connection.release();
            setBackendAll(&_connection->backend());
            delete old_connection;
            return {};
        }
        types::gpio_slot_t _remote_gpio_slot = 0;
        bool _has_remote_gpio                = false;
    };

    /*!
      @brief Internal singleton bootstrapping the default Hal facade.

      The public surface lives in `Hal _hal`; callers interact through
      `getM5_Hal()` / `M5_Hal` which return `Hal&`.
     */
    class M5HALCore {
    public:
        Hal& hal(void)
        {
            return _hal;
        }

        M5HALCore(const M5HALCore&)            = delete;
        M5HALCore& operator=(const M5HALCore&) = delete;
        M5HALCore(M5HALCore&&)                 = delete;
        M5HALCore& operator=(M5HALCore&&)      = delete;

    private:
        M5HALCore();
        Hal _hal;

        friend Hal& getM5_Hal();
    };

    // ----- instance accessor -----

    /*!
      @brief Singleton accessor (Magic Static; first-use lazy;
             thread-safe by C++17 §6.7).

      Typical callers use `M5_Hal.*`, but namespace-scope
      initializers and another library's global ctor MUST call
      `getM5_Hal()` directly to avoid the static initialization
      order fiasco.
     */
    inline Hal& getM5_Hal()
    {
        static M5HALCore s_core;
        return s_core._hal;
    }

    /*!
      @brief Reference alias for the `M5_Hal.Gpio.foo()` syntax.

      Eager-initialized: namespace-scope dynamic initialization runs
      `getM5_Hal()` once at startup. The actual ctor body is still
      protected by the Magic Static guard. The definition lives in
      M5HAL_v2.cpp.
     */
    extern Hal& M5_Hal;
}
}  // namespace hal
}  // namespace m5

#endif  // M5_HAL_HAL_V2_M5_HAL_HPP_
