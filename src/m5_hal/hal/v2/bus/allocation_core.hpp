// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_ALLOCATION_CORE_HPP_
#define M5_HAL_BUS_ALLOCATION_CORE_HPP_

#include "../error.hpp"
#include "../types.hpp"
#include "./bus.hpp"
#include "./hw_pool.hpp"
#include "./managed_bus.hpp"
#include "./registry.hpp"

#include <memory>

/*!
  @namespace m5::hal::v2::bus
  @brief Kind-neutral intent-driven controller allocation.
 */
namespace m5::hal::v2::bus {

/*!
  @brief The kind-neutral controller-allocation resolver.

  Holds one kind's hardware-controller pool and resolves every live bus of
  that kind from its `AllocationIntent`, exactly as the original I2C
  `commitBuses` did, but driven through `IAllocationKind` (backend factories
  + capability model) and `IManagedBus` (per-bus intent + hot-swap) so spi /
  i2s reuse it without changing this code.

  For a kind whose controllers are interchangeable (`uniformControllers()`),
  the capability eligibility filter short-circuits to "always eligible", so
  control flow is identical to the pre-generalization I2C path.
 */
class AllocationCore {
public:
    AllocationCore(BusRegistry& registry, const IAllocationKind& kind);

    AllocationCore(BusRegistry& registry, const IAllocationKind& kind, uint8_t controller_capacity);

    AllocationCore(const AllocationCore&)            = delete;
    AllocationCore& operator=(const AllocationCore&) = delete;

    BusRegistry& registry(void);

    /*! @brief Hardware controllers currently leased (observation: tests). */
    uint8_t hardwareInUse(void) const;

    /*!
      @brief Resolve every live bus of this kind's backend from its intent.

      See the original I2C `commitBuses` for the contract: highest priority
      first (required controller, required hardware, preferred, automatic),
      incumbency, deterministic lowest-free tie-break, conflict /
      over-subscription failures BEFORE any swap, pool-mediated demote-then-
      promote hot-swap. A software-less kind's placeholder is null, so a
      demoted bus becomes pending instead of software.
     */
    result_t<void> commitBuses(uint32_t timeout_ms = types::TIMEOUT_FOREVER);

    /*!
      @brief Claim a hardware controller OUTSIDE the intent resolver.

      For a caller with no `IManagedBus` of its own (e.g. a standalone
      slave), not the master `acquire()`/`commitBuses()` path. Resolution
      mirrors the resolver's own rules, restricted to the bus-independent
      part (capability + opt-in; there is no bus to run the pin-domain hook
      against): `Require(id)` claims exactly `id` (`OUT_OF_RESOURCE` if busy,
      `INVALID_ARGUMENT` if out of range or capability-ineligible);
      `Prefer(id)` tries `id` first, then falls back to Auto; `Auto` (the
      default, `intent == {}`) claims the lowest-numbered eligible free
      controller. `forbid & HARDWARE` or an invalid intent is
      `INVALID_ARGUMENT` (a claim is defined as hardware occupancy). A
      claimed controller survives `commitBuses()`'s `releaseAll()` rebuild
      (see `HwControllerPool::releaseAll`) and is reserved out of the
      resolver's `used[]` set for the whole commit.
     */
    result_t<int8_t> claimController(const types::AllocationIntent& intent);

    /*! @brief Return a controller claimed via `claimController`. */
    result_t<void> releaseClaimedController(int8_t controller);

private:
    void _syncPoolFromLive(IBus* const* qbus, size_t n);

    // Bus-independent half of _eligible: the capability + opt-in filter,
    // shared by the commit-time resolver AND claimController (which has no
    // IManagedBus to run the pin-domain hook against).
    bool _eligibleCaps(const types::AllocationIntent& want, int8_t controller) const;

    // Whether a controller satisfies a bus's capability + pin-domain
    // constraints. The capability check (require/forbid/opt-in) runs for
    // EVERY kind, uniform or not (a uniform kind's caps are still
    // Traits::CAPS_HARDWARE, so a require bit it does not offer, e.g.
    // LOW_POWER on a plain kind, now fails here instead of silently
    // succeeding); the pin-domain hook (`controllerAcceptsBus`) is
    // non-uniform-only, matching the pre-generalization i2c path exactly
    // when `uniformControllers()` is true.
    bool _eligible(const IManagedBus& bus, const types::AllocationIntent& want, int8_t controller) const;

    // Map an AllocationIntent to a controller-resolver priority tier:
    // 0 = a specific controller is required, 1 = hardware required, 2 = hardware
    // or a specific controller preferred, 3 = automatic, -1 = software only.
    static int allocTier(const types::AllocationIntent& a);

    // Serializes commitBuses / claimController / releaseClaimedController
    // against each other. The resolver was designed to drive a whole
    // bin-packing pass from one task context, but claimController is a
    // public entry another task may call while a commit is mid-rebuild
    // (_syncPoolFromLive drops every ordinary lease before re-establishing
    // them one by one); without this lock a concurrent claim could take a
    // controller a live bus still occupies. Held across the whole commit;
    // claims never take a bus lock, so the nesting order (this, then
    // pool/bus internals) is one-way and cannot deadlock.
    struct SerialGuard {
        runtime::Mutex& m;
        explicit SerialGuard(runtime::Mutex& mtx) : m{mtx}
        {
            (void)m.lock(types::TIMEOUT_FOREVER);
        }
        ~SerialGuard(void)
        {
            m.unlock();
        }
        SerialGuard(const SerialGuard&)            = delete;
        SerialGuard& operator=(const SerialGuard&) = delete;
    };

    BusRegistry& _registry;
    const IAllocationKind& _kind;
    HwControllerPool _pool;
    runtime::Mutex _serial_mutex;
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_ALLOCATION_CORE_HPP_
