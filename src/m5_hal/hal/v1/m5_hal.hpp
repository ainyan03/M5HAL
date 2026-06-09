#ifndef M5_HAL_HAL_V1_M5_HAL_HPP_
#define M5_HAL_HAL_V1_M5_HAL_HPP_

// M5HALCore — singleton HAL object layer.
//
// Role: bundle HAL sub-objects (GPIOGroup, ServiceRunner, ...) into a
// single instance. Callers reach the sub-objects through
// `M5_Hal.Gpio.foo()`.
//
// Instance access:
//   - `getM5_Hal()` (Magic Static; first-use lazy; thread-safe by
//     C++17 §6.7).
//   - `extern M5HALCore& M5_Hal` — eager-initialized alias used by
//     the public `M5_Hal.Gpio.foo()` syntax. Defined in M5HAL_v1.cpp.
//
// Important caveat:
//   - `M5_Hal` is a public syntax sugar with eager initialization.
//     Use it only from caller-side code (setup / loop / function-local
//     scopes). From namespace-scope initializers or another library's
//     global constructor, ALWAYS go through `getM5_Hal()` to avoid the
//     static initialization order fiasco.
//   - `M5HALCore` enforces singleton (private ctor + `getM5_Hal()`
//     friend); no other instance may be constructed. A future multi-
//     instance / Local-Remote symmetric design would first require
//     reshaping `gpio_number_t` into a HAL-relative handle.
//
// Placement: an exception to the 1:1 namespace/filename rule. The
// file stem is `m5_hal`, the namespace is `m5::hal::v1`, the class
// is `M5HALCore`. See spec/architecture.md, the namespace-rule exception clause.

#include "../../../m5_hal_config.hpp"  // M5HAL_INLINE_V1

#include "gpio/group.hpp"
#include "memory/allocator.hpp"
#include "service/service.hpp"

namespace m5 {
namespace hal {
M5HAL_INLINE_V1 namespace v1
{
    /*!
      @brief Singleton HAL object layer.

      Owns the HAL sub-objects (`GPIOGroup`, `ServiceRunner`,
      `memory::Allocator`, ...) as direct members. Callers reach the
      sub-objects via `M5_Hal.Gpio.*` etc.
     */
    class M5HALCore {
    public:
        /*!
          @brief The GPIO registry, owned by M5HALCore.

          Process-unique because the ctor (singleton-enforced) runs
          only once, through the `M5_Hal` definition.
         */
        gpio::GPIOGroup Gpio;

        /*!
          @brief Cooperative service runner.

          The runner lives in M5HALCore; each driver / adapter owns
          its own service instance and registers / unregisters with
          this runner on init / release.
         */
        service::ServiceRunner Services;

        /*!
          @brief Temporary memory allocator.

          Provides a fixed-block temp pool with malloc fallback for
          short-lived internal buffers. Persistent allocations bypass
          the pool and use the fallback path directly. Task context only;
          do not allocate or free from ISR.
         */
        memory::Allocator Memory;

        // Room to grow: future additions like `i2c::I2CBusGroup I2C;`
        // or `spi::SPIBusGroup SPI;` would live here. Each addition
        // should be evaluated for its startup-cost impact (eager
        // initialization through `M5_Hal`) and its side-effect surface.

        // Copy / move disabled (singleton).
        M5HALCore(const M5HALCore&)            = delete;
        M5HALCore& operator=(const M5HALCore&) = delete;
        M5HALCore(M5HALCore&&)                 = delete;
        M5HALCore& operator=(M5HALCore&&)      = delete;

    private:
        // Private ctor: only `getM5_Hal()` can construct (via
        // friendship). The body lives in M5HAL_v1.cpp where
        // flat-injection has already exposed `gpio::getGPIO()`.
        M5HALCore();

        friend M5HALCore& getM5_Hal();
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
    inline M5HALCore& getM5_Hal()
    {
        static M5HALCore s_hal;
        return s_hal;
    }

    /*!
      @brief Reference alias for the `M5_Hal.Gpio.foo()` syntax.

      Eager-initialized: namespace-scope dynamic initialization runs
      `getM5_Hal()` once at startup. The actual ctor body is still
      protected by the Magic Static guard. The definition lives in
      M5HAL_v1.cpp.
     */
    extern M5HALCore& M5_Hal;
}
}  // namespace hal
}  // namespace m5

#endif  // M5_HAL_HAL_V1_M5_HAL_HPP_
