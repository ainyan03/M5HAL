// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_MANAGED_BUS_HPP_
#define M5_HAL_BUS_MANAGED_BUS_HPP_

#include "../types.hpp"
#include "./bus.hpp"

#include <memory>

/*!
  @namespace m5::hal::v2::bus
  @brief Kind-neutral seams for intent-driven controller allocation.
 */
namespace m5::hal::v2::bus {

// --- Allocation-intent helpers (the sanctioned way to build an intent) -------
// Kind-agnostic: they touch only the cross-kind HARDWARE bit, so each kind
// re-exposes them in its own namespace with `using bus::requireHardware;` etc.
// Callers write `i2c::requireHardware()` / `spi::requireHardware()` and the
// capability masks stay internal.

/*! @brief Must run on a hardware controller; commit fails if none is free. */
constexpr types::AllocationIntent requireHardware(void)
{
    types::AllocationIntent a;
    a.require = types::backend_caps::HARDWARE;
    return a;
}
/*! @brief Prefer a hardware controller, but fall back to software when none is free. */
constexpr types::AllocationIntent preferHardware(void)
{
    types::AllocationIntent a;
    a.prefer = types::backend_caps::HARDWARE;
    return a;
}
/*! @brief Always run in software (yields any hardware controller to other buses). */
constexpr types::AllocationIntent software(void)
{
    types::AllocationIntent a;
    a.forbid = types::backend_caps::HARDWARE;
    return a;
}
/*! @brief No preference: hardware when a controller is free, else software (the default). */
constexpr types::AllocationIntent automatic(void)
{
    return types::AllocationIntent{};
}
/*! @brief Require a specific hardware controller index; commit fails if it is unavailable. */
constexpr types::AllocationIntent requireController(int8_t controller)
{
    types::AllocationIntent a;
    a.require         = types::backend_caps::HARDWARE;
    a.controller_id   = controller;
    a.controller_mode = types::ControllerMode::Require;
    return a;
}
/*! @brief Prefer a specific hardware controller; take another (or software) when it is busy. */
constexpr types::AllocationIntent preferController(int8_t controller)
{
    types::AllocationIntent a;
    a.prefer          = types::backend_caps::HARDWARE;
    a.controller_id   = controller;
    a.controller_mode = types::ControllerMode::Prefer;
    return a;
}
/*!
  @brief Require a hardware controller in a low-power domain (e.g. LP_I2C);
         commit fails if none is free.

  `LOW_POWER` is opt-in (see `IAllocationKind::optInCaps`), so plain
  `requireHardware()` never lands on such a controller; this helper is the
  sanctioned way to ask for one.
 */
constexpr types::AllocationIntent requireLowPower(void)
{
    types::AllocationIntent a;
    a.require = types::backend_caps::HARDWARE | types::backend_caps::LOW_POWER;
    return a;
}
/*! @brief Prefer a low-power-domain hardware controller, falling back to any hardware controller when none is free. */
constexpr types::AllocationIntent preferLowPower(void)
{
    types::AllocationIntent a;
    a.require = types::backend_caps::HARDWARE;
    a.prefer  = types::backend_caps::LOW_POWER;
    return a;
}

/*!
  @brief The per-bus facade surface the allocation resolver touches.

  A kind facade (e.g. `i2c::Bus`) inherits this in addition to its kind
  `IBus`, so the kind-neutral `AllocationCore` can read a bus's intent and
  hot-swap its backend without naming the kind. The resolver reaches a bus's
  query metadata (`backendKind()` / `controllerId()`) through the `IBus`
  view it already holds; this interface only adds the intent + swap surface.

  `swapBackend` keeps the strict non-null contract (a real backend). The
  separate `swapPending` is the relaxed path used ONLY by software-less kinds
  whose placeholder is null: it detaches the backend so the bus becomes
  "pending" (unusable until a later commit gives it a controller). A kind with
  a software placeholder (i2c) never calls `swapPending`, so its strict
  `swapBackend` guard is preserved and its behaviour is unchanged.
 */
struct IManagedBus {
    virtual ~IManagedBus(void) = default;

    /*! @brief Snapshot the acquire intent recorded for this bus (commit-time resolver). */
    virtual types::AllocationIntent intent(void) const = 0;

    /*! @brief Whether this bus opted into intent-driven management. */
    virtual bool managed(void) const = 0;

    /*!
      @brief Replace the recorded intent while the allocation core is serialized.

      Internal resolver seam. Logical re-acquire routes through
      `AllocationCore::retagIntent`, which serializes this short update against
      the commit-time intent snapshot; callers must not invoke this hook
      without that guard. A retag after the snapshot belongs to the next
      commit.
     */
    virtual void retagIntent(const types::AllocationIntent& intent) = 0;

    /*! @brief Hot-swap to a real (non-null) backend under the bus lock. */
    virtual result_t<void> swapBackend(std::unique_ptr<IBus> new_backend,
                                       uint32_t timeout_ms = types::TIMEOUT_FOREVER) = 0;

    /*! @brief Detach to a null (pending) backend under the bus lock (software-less kinds). */
    virtual result_t<void> swapPending(uint32_t timeout_ms = types::TIMEOUT_FOREVER) = 0;
};

/*!
  @brief The per-kind adapter the allocation resolver calls back into.

  Implemented by a kind's `LocalKindAdapter`, which owns the build-injected
  backend factories and the per-controller capability model.
  The resolver (`AllocationCore`) is kind-neutral: it holds a reference to
  this interface and drives backend creation / capability checks through it.

  Raw backend factories remain private to the concrete local adapter. The
  resolver only sees the atomic commit hooks, so alternate policies and test
  fakes do not have to expose construction helpers that the core never calls.
 */
struct IAllocationKind {
    virtual ~IAllocationKind(void) = default;

    /*! @brief This kind's tag (the registry filter for live-bus enumeration). */
    virtual types::bus_kind_t kind(void) const = 0;

    /*! @brief Number of hardware controllers (the silicon budget; 0 = none). */
    virtual uint8_t controllerCapacity(void) const = 0;

    /*! @brief Whether this build can make hardware backends at all (a hardware factory exists). */
    virtual bool hasHardware(void) const = 0;

    /*! @brief Adapt a live `IBus` of this kind to its managed-bus surface (kind-safe downcast). */
    virtual IManagedBus& toManaged(IBus& bus) const = 0;

    /*!
      @brief Build the placeholder backend and swap it in UNDER THE BUS LOCK.

      The build (and its `init()`) runs inside the bus lock so it cannot race
      an in-flight transfer on the old backend.
      `intent` is the immutable commit snapshot threaded through the build and
      any rollback reconstruction.
      A null placeholder from a software-less kind is its intentional
      pending detach; a null from a kind WITH a software factory is a build
      failure — the swap rolls back to a re-made hardware backend on the
      controller being given up and returns the error. Release errors
      propagate.
     */
    virtual result_t<void> commitPlaceholder(IManagedBus& bus, const types::AllocationIntent& intent,
                                             uint32_t timeout_ms) const = 0;

    /*!
      @brief Build the hardware backend for `controller` and swap it in UNDER THE BUS LOCK.

      The build (and its `init()`) runs inside the bus lock. `intent` is the
      immutable commit snapshot used by planning,
      pin eligibility, the build, and any rollback. A null result is
      `OUT_OF_RESOURCE`; the swap rolls back to a re-made placeholder
      (or pending for a software-less kind). Release errors propagate.
     */
    virtual result_t<void> commitHardware(IManagedBus& bus, const types::AllocationIntent& intent, int8_t controller,
                                          uint32_t timeout_ms) const = 0;

    /*!
      @brief Whether all controllers offer identical capabilities.

      True for i2c (interchangeable controllers): the capability filter
      (require/forbid against `controllerCaps()`) still runs -- so a require
      bit the kind never offers fails instead of silently succeeding -- but
      the opt-in and pin-domain checks are skipped. False for a kind with
      non-equivalent controllers (i2s: only I2S0 has DAC/ADC; an LP_I2C
      build), where `controllerCaps` is a real per-controller value and the
      opt-in / pin-domain hooks apply.
     */
    virtual bool uniformControllers(void) const = 0;

    /*!
      @brief Capability bitmask offered by `controller`.

      ALWAYS consulted for require/forbid matching, uniform or not: a
      uniform kind must return one constant mask for every controller.
     */
    virtual types::backend_caps_t controllerCaps(int8_t controller) const = 0;

    /*!
      @brief Capability bits of this kind that require explicit opt-in.

      A controller whose `controllerCaps()` includes one of these bits is
      eligible for an allocation request only when the request's intent
      explicitly declares the same bit via `require`/`prefer`, or names the
      controller directly via `ControllerMode::Prefer`/`Require`. Otherwise
      the controller is skipped by automatic allocation (e.g. a low-power
      domain controller such as LP_I2C, which must never be handed out to a
      plain `requireHardware()` request).

      Defaulted to 0 (no opt-in bits) so existing kinds and test fakes need
      no change. A kind with `uniformControllers() == true` MUST return 0
      (the opt-in filter is skipped for uniform kinds, so a non-zero value
      there would be dead and misleading).
     */
    virtual types::backend_caps_t optInCaps(void) const
    {
        return 0;
    }

    /*!
      @brief Pin-domain check: does `controller` accept this bus's wiring?

      A non-uniform kind (`uniformControllers() == false`) may restrict a
      controller to a fixed pin pair or a candidate-pin set -- e.g. a
      low-power-domain controller whose SDA/SCL are hard-wired to a single
      IOMUX pad pair, or wired through a GPIO matrix that only reaches a
      restricted pin set. The resolver consults this AFTER the capability
      filter, so it only ever runs on a controller already eligible on caps.
      `intent` is the same immutable snapshot used by the rest of the commit.

      Defaulted to always-true (no pin restriction), so existing kinds and
      test fakes need no change. A uniform kind is NEVER asked -- the
      eligibility filter short-circuits before reaching this check.
     */
    virtual bool controllerAcceptsBus(const IManagedBus& bus, const types::AllocationIntent& intent,
                                      int8_t controller) const
    {
        (void)bus;
        (void)intent;
        (void)controller;
        return true;
    }
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_MANAGED_BUS_HPP_
