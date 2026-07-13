// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_M5_HAL_HPP_
#define M5_HAL_HAL_V2_M5_HAL_HPP_

// Hal — the public HAL facade type.
//
// `Hal` bundles HAL sub-objects (GPIOGroup, ServiceRunner, memory
// Allocator, per-kind BusViews) behind a backend pointer. Callers
// write `void scan(Hal& hal) { hal.I2C.acquire(...); }` and the
// same function works for both local and remote backends.
//
// Two construction modes:
//   - Local (M5_Hal singleton): M5HALCore creates Hal with LocalBackend.
//   - Remote: user creates `Hal remote;` then `remote.connect(endpoint)`.
//     connect establishes the transport, creates a RemoteBackend, and
//     wires BusViews to it. After connect, `remote.I2C.acquire()` creates
//     proxy buses on the remote peer.
//
// M5HALCore — internal singleton owning the local backend.
//
// Holds a `LocalBackend`, per-kind adapters, and a `Hal` instance
// wired to that backend. The Hal is exposed through `getM5_Hal()`
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

    /*!
      @brief Public HAL facade — local and remote share this surface.

      A `Hal` bundles the HAL sub-objects (GPIO, service runner, memory
      allocator, per-kind bus views) behind an `IHalBackend*`. The local
      singleton (`M5_Hal`) is backed by `LocalBackend` inside
      `M5HALCore`; remote instances are backed by `RemoteBackend`.
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
        bus::IHalBackend* _backend;
        remote::RemoteConnectionState* _connection = nullptr;
        // A Pin/PortAccess is a non-owning value handle.  Preserve only the
        // disconnected GPIO ports until this Hal dies; never retain the old
        // connection, session object, or transport buffers.
        std::unique_ptr<remote::RemoteGpioOwner> _retired_remote_gpios;

    public:
        gpio::GPIOGroup Gpio;
        service::ServiceRunner Services;
        memory::Allocator Memory;

        /*!
          @name Per-kind bus access.

          All kinds are backend-delegating views:
          `hal.I2C.acquire(cfg)` / `hal.SPI.acquire(cfg)` /
          `hal.UART.acquire(cfg)` / `hal.I2S.acquire(cfg)` intern the bus
          for `cfg`'s pins and return a `shared_ptr` — one physical wiring,
          one shared instance.
          @{
         */
        i2c::BusView I2C;
        spi::BusView SPI;
        uart::BusView UART;
        i2s::BusView I2S;
        /*! @} */

        Hal() : _backend{nullptr}
        {
            Gpio.bindServiceRunner(&Services);
        }

        ~Hal();

        bus::IHalBackend* backend(void)
        {
            return _backend;
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
        result_t<void> connect(const char* endpoint = nullptr, const remote::DeviceConfig& cfg = {});
        result_t<void> init(void);
        result_t<void> initUart(const char* port, const remote::DeviceConfig& cfg = {});
        result_t<void> initTcp(const char* endpoint, const remote::DeviceConfig& cfg = {});
        /*! @} */

        bool isConnected(void) const
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
            : _backend{backend}, I2C{backend}, SPI{backend}, UART{backend}, I2S{backend}
        {
            Gpio.bindServiceRunner(&Services);
        }

        friend class M5HALCore;

    private:
        void setBackendAll(bus::IHalBackend* backend)
        {
            _backend = backend;
            I2C.setBackend(backend);
            SPI.setBackend(backend);
            UART.setBackend(backend);
            I2S.setBackend(backend);
        }
        void registerRemoteGPIO(const gpio::IGPIO* gpio)
        {
            if (gpio == nullptr) {
                return;
            }
            types::gpio_slot_t slot = Gpio.hasGPIO(0) ? 1 : 0;
            auto r                  = Gpio.addGPIO(gpio, slot);
            if (r.has_value()) {
                _remote_gpio_slot = slot;
                _has_remote_gpio  = true;
            }
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
        types::gpio_slot_t _remote_gpio_slot = 0;
        bool _has_remote_gpio                = false;
    };

    /*!
      @brief Internal singleton owning the local backend and adapters.

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

        bus::LocalBackend _backend;
        bus::LocalKindAdapter<i2c::BusTraits> _i2c_adapter;
        bus::LocalKindAdapter<spi::BusTraits> _spi_adapter;
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
