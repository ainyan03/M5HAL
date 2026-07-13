// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_BUS_ALLOCATION_CORE_INL_
#define M5_HAL_HAL_V2_BUS_ALLOCATION_CORE_INL_

#include "allocation_core.hpp"

namespace m5::hal::v2::bus {

AllocationCore::AllocationCore(BusRegistry& registry, const IAllocationKind& kind)
    : AllocationCore{registry, kind, kind.controllerCapacity()}
{
}

AllocationCore::AllocationCore(BusRegistry& registry, const IAllocationKind& kind, uint8_t controller_capacity)
    : _registry{registry}, _kind{kind}, _pool{controller_capacity}
{
}

BusRegistry& AllocationCore::registry(void)
{
    return _registry;
}

uint8_t AllocationCore::hardwareInUse(void) const
{
    return _pool.inUse();
}

result_t<void> AllocationCore::commitBuses(uint32_t timeout_ms)
{
    // Excludes claimController/releaseClaimedController for the whole pass:
    // the plan below and the pool rebuild (_syncPoolFromLive) both assume
    // nobody else touches the pool mid-commit (see SerialGuard).
    SerialGuard serial{_serial_mutex};
    const types::bus_kind_t kind = _kind.kind();

    // 1. Snapshot all live buses of this kind. The allocation plan is
    // derived from this stable view; later swap steps may change live
    // backend state, but not the already computed plan.
    struct Entry {
        IManagedBus* managed_bus = nullptr;
        IBus* bus                = nullptr;
        std::shared_ptr<IBus> holder{};
        types::AllocationIntent intent{};
        types::backend_kind_t backend_kind = types::backend_kind_t::Software;
        int8_t controller_id               = HwControllerPool::kNone;
        bool managed                       = false;
    };

    Entry entries[BusRegistry::kCapacity];
    IBus* qbus[BusRegistry::kCapacity];
    size_t n = 0;
    _registry.forEachLive(kind, [&](const std::shared_ptr<IBus>& sp) {
        if (n >= BusRegistry::kCapacity) {
            return;
        }
        entries[n].holder        = sp;
        entries[n].bus           = sp.get();
        entries[n].managed_bus   = &_kind.toManaged(*sp);
        entries[n].managed       = entries[n].managed_bus->managed();
        entries[n].backend_kind  = entries[n].bus->backendKind();
        entries[n].controller_id = entries[n].bus->controllerId();
        if (entries[n].managed) {
            entries[n].intent = entries[n].managed_bus->intent();
        }
        qbus[n] = entries[n].bus;
        ++n;
    });

    // Reject conflicting intents (a cap both required and forbidden) up
    // front, before any backend is swapped, so no bus is left half-changed.
    for (size_t i = 0; i < n; ++i) {
        if (entries[i].managed && !entries[i].intent.valid()) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
        }
    }

    // 2. Compute the target controller per MANAGED bus (priority +
    //    incumbency), purely in a local mask -- no side effects, so an
    //    over-subscribed require-hardware / require-controller aborts before any swap.
    const uint8_t capacity = _pool.capacity();
    int8_t target[BusRegistry::kCapacity];
    bool used[HwControllerPool::kMaxControllers] = {false};
    // A bus whose demote (release) failed must not be re-touched by the
    // promote loop -- otherwise a hardware retarget would retry release on a
    // backend whose swap already aborted, defeating the keep-old-on-failure
    // policy and risking a leaked / mis-accounted controller.
    bool demote_failed[BusRegistry::kCapacity] = {false};
    for (size_t i = 0; i < n; ++i) {
        target[i] = HwControllerPool::kNone;
    }
    // Reserve the controllers held by UNMANAGED hardware buses first.
    for (size_t i = 0; i < n; ++i) {
        if (!entries[i].managed && entries[i].backend_kind == types::backend_kind_t::Hardware) {
            const int8_t cur = entries[i].controller_id;
            if (cur >= 0 && static_cast<uint8_t>(cur) < capacity) {
                used[cur] = true;
            }
        }
    }
    // Reserve externally-claimed controllers (claimController) the same way:
    // they are occupied by a caller entirely outside this bus list, so a
    // tier-0 Require targeting one must fail OUT_OF_RESOURCE instead of the
    // resolver handing it out.
    for (uint8_t c = 0; c < capacity; ++c) {
        if (_pool.isExternal(static_cast<int8_t>(c))) {
            used[c] = true;
        }
    }
    // Tiers 0..3 derived from each managed bus's intent (see allocTier).
    for (int tier = 0; tier < 4; ++tier) {
        for (size_t i = 0; i < n; ++i) {
            if (!entries[i].managed) {
                continue;
            }
            const types::AllocationIntent& want = entries[i].intent;
            if (allocTier(want) != tier) {
                continue;
            }
            if (tier == 0) {
                // A specific controller is required: no fallback.
                const int8_t pin = want.controller_id;
                if (pin < 0 || static_cast<uint8_t>(pin) >= capacity || used[pin]) {
                    return m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
                }
                const types::backend_caps_t cap = _kind.controllerCaps(pin);
                if ((cap & want.require) != want.require || (cap & want.forbid) != 0) {
                    return m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
                }
                if (!_eligible(*entries[i].managed_bus, want, pin)) {
                    // Caps already matched above (and this is a named
                    // controller, so the opt-in bypass in _eligible always
                    // passes), so the only remaining check that can fail here
                    // is the pin-domain hook: a wiring mismatch on a named
                    // controller is a configuration error, not a resource
                    // shortage.
                    return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
                }
                used[pin] = true;
                target[i] = pin;
                continue;
            }
            // Incumbency: keep the current controller when still free and eligible.
            int8_t chosen = HwControllerPool::kNone;
            if (entries[i].backend_kind == types::backend_kind_t::Hardware) {
                const int8_t cur = entries[i].controller_id;
                if (cur >= 0 && static_cast<uint8_t>(cur) < capacity && !used[cur] &&
                    _eligible(*entries[i].managed_bus, want, cur)) {
                    chosen = cur;
                }
            }
            // A preferred specific controller wins the slot when free and eligible.
            if (chosen < 0 && want.controller_mode == types::ControllerMode::Prefer && want.controller_id >= 0 &&
                static_cast<uint8_t>(want.controller_id) < capacity && !used[want.controller_id] &&
                _eligible(*entries[i].managed_bus, want, want.controller_id)) {
                chosen = want.controller_id;
            }
            // A capability preference (e.g. an opt-in low-power controller)
            // wins over an equally-eligible controller that lacks it. Only
            // meaningful for a non-uniform kind, where controllerCaps() is a
            // real per-controller value.
            if (chosen < 0 && want.prefer != 0 && !_kind.uniformControllers()) {
                for (uint8_t c = 0; c < capacity; ++c) {
                    if (!used[c] && _eligible(*entries[i].managed_bus, want, static_cast<int8_t>(c)) &&
                        (_kind.controllerCaps(static_cast<int8_t>(c)) & want.prefer) == want.prefer) {
                        chosen = static_cast<int8_t>(c);
                        break;
                    }
                }
            }
            // Otherwise the lowest free eligible controller (deterministic tie-break).
            if (chosen < 0) {
                for (uint8_t c = 0; c < capacity; ++c) {
                    if (!used[c] && _eligible(*entries[i].managed_bus, want, static_cast<int8_t>(c))) {
                        chosen = static_cast<int8_t>(c);
                        break;
                    }
                }
            }
            if (chosen < 0) {
                if (tier == 1) {
                    return m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
                }
                continue;  // preferred / automatic: stay on the placeholder
            }
            used[chosen] = true;
            target[i]    = chosen;
        }
    }

    // 3. Rebuild the pool from the live hardware buses (reclaim leaked leases).
    _syncPoolFromLive(qbus, n);

    m5::hal::v2::result_t<void> first_error{};
    bool have_error = false;

    // 4. Demote: a managed hardware bus that should not keep its controller
    //    drops to its kind placeholder (software, or null=pending) and
    //    returns the controller to the pool.
    for (size_t i = 0; i < n; ++i) {
        if (!entries[i].managed) {
            continue;
        }
        if (entries[i].backend_kind != types::backend_kind_t::Hardware) {
            continue;
        }
        const int8_t cur = entries[i].controller_id;
        if (target[i] == cur) {
            continue;  // keeper: untouched (required-hardware immunity falls out here)
        }
        // commitPlaceholder builds + adopts under the bus lock:
        // makePlaceholder's init() no longer runs outside the swap guard.
        auto r = _kind.commitPlaceholder(*entries[i].managed_bus, timeout_ms);
        if (!r.has_value()) {
            demote_failed[i] = true;  // its release failed: keep it off the promote loop
            if (!have_error) {
                first_error = r;
                have_error  = true;
            }
            continue;
        }
        _pool.release(cur);
    }

    // 5. Promote: a bus whose target is hardware but is not already on that
    //    controller claims it from the pool and swaps in a hardware backend.
    for (size_t i = 0; i < n; ++i) {
        if (target[i] < 0) {
            continue;
        }
        if (demote_failed[i]) {
            continue;  // demote/release failed above; do not retry a swap on this bus
        }
        if (entries[i].backend_kind == types::backend_kind_t::Hardware && entries[i].controller_id == target[i]) {
            continue;
        }
        if (!_kind.hasHardware() || !_pool.acquireSpecific(target[i])) {
            if (!have_error) {
                first_error = m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
                have_error  = true;
            }
            continue;
        }
        // commitHardware builds + adopts under the bus lock; a null
        // hardware factory result becomes OUT_OF_RESOURCE inside it.
        auto r = _kind.commitHardware(*entries[i].managed_bus, target[i], timeout_ms);
        if (!r.has_value()) {
            _pool.release(target[i]);
            if (!have_error) {
                first_error = r;
                have_error  = true;
            }
        }
    }

    // A backend swap may fail after partially changing its live state.
    // Keep the observable pool state derived from the bus state.
    _syncPoolFromLive(qbus, n);

    return have_error ? first_error : m5::hal::v2::result_t<void>{};
}

void AllocationCore::_syncPoolFromLive(IBus* const* qbus, size_t n)
{
    _pool.releaseAll();
    for (size_t i = 0; i < n; ++i) {
        if (qbus[i]->backendKind() == types::backend_kind_t::Hardware) {
            (void)_pool.acquireSpecific(qbus[i]->controllerId());
        }
    }
}

bool AllocationCore::_eligibleCaps(const types::AllocationIntent& want, int8_t controller) const
{
    // The capability check runs for EVERY kind, uniform or not: a uniform
    // kind's controllerCaps() is a constant (Traits::CAPS_HARDWARE), so a
    // require bit it never offers (e.g. LOW_POWER on a plain-hardware kind)
    // now fails here instead of an earlier uniform short-circuit silently
    // accepting it.
    const types::backend_caps_t cap = _kind.controllerCaps(controller);
    if ((cap & want.require) != want.require || (cap & want.forbid) != 0) {
        return false;
    }
    if (_kind.uniformControllers()) {
        return true;
    }
    // Opt-in caps (e.g. LOW_POWER): a controller offering one is eligible
    // only when the intent explicitly declares the same bit (require/prefer)
    // or names this controller directly via ControllerMode::Prefer/Require.
    // Otherwise it stays out of the automatic-allocation pool.
    const types::backend_caps_t opt = _kind.optInCaps() & cap;
    if (opt != 0) {
        const bool named = (want.controller_mode == types::ControllerMode::Prefer ||
                            want.controller_mode == types::ControllerMode::Require) &&
                           want.controller_id == controller;
        if (!named && (opt & ~(want.require | want.prefer)) != 0) {
            return false;
        }
    }
    return true;
}

bool AllocationCore::_eligible(const IManagedBus& bus, const types::AllocationIntent& want, int8_t controller) const
{
    if (!_eligibleCaps(want, controller)) {
        return false;
    }
    if (_kind.uniformControllers()) {
        return true;
    }
    // Pin-domain check: non-uniform only, consulted last so the capability /
    // opt-in filters above still run identically to before.
    return _kind.controllerAcceptsBus(bus, controller);
}

result_t<int8_t> AllocationCore::claimController(const types::AllocationIntent& intent)
{
    SerialGuard serial{_serial_mutex};
    if (!intent.valid() || (intent.forbid & types::backend_caps::HARDWARE) != 0) {
        // A claim is defined as hardware occupancy: an intent that forbids
        // hardware (or is otherwise malformed) cannot be satisfied by one.
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    const uint8_t capacity = _pool.capacity();

    if (intent.controller_mode == types::ControllerMode::Require) {
        const int8_t pin = intent.controller_id;
        if (pin < 0 || static_cast<uint8_t>(pin) >= capacity || !_eligibleCaps(intent, pin)) {
            // Out of range or capability-ineligible: a configuration error,
            // not a resource shortage.
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
        }
        if (!_pool.claimExternal(pin)) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
        }
        return pin;
    }

    if (intent.controller_mode == types::ControllerMode::Prefer) {
        const int8_t pin = intent.controller_id;
        if (pin >= 0 && static_cast<uint8_t>(pin) < capacity && _eligibleCaps(intent, pin) &&
            _pool.claimExternal(pin)) {
            return pin;
        }
        // Falls through to Auto below.
    }

    // A capability preference (e.g. an opt-in low-power controller) wins
    // over an equally-eligible controller that lacks it -- the resolver's
    // own preferred-capability pass, mirrored here so a claim and a managed
    // bus resolve the same intent the same way. Only meaningful for a
    // non-uniform kind, where controllerCaps() is a real per-controller value.
    if (intent.prefer != 0 && !_kind.uniformControllers()) {
        for (uint8_t c = 0; c < capacity; ++c) {
            if (_eligibleCaps(intent, static_cast<int8_t>(c)) &&
                (_kind.controllerCaps(static_cast<int8_t>(c)) & intent.prefer) == intent.prefer &&
                _pool.claimExternal(static_cast<int8_t>(c))) {
                return static_cast<int8_t>(c);
            }
        }
    }

    // Auto: lowest-numbered eligible free controller. Try-claim per
    // candidate (rather than pre-checking isLeased then claiming) so the
    // scan/claim stays a single pool operation per index.
    for (uint8_t c = 0; c < capacity; ++c) {
        if (!_eligibleCaps(intent, static_cast<int8_t>(c))) {
            continue;
        }
        if (_pool.claimExternal(static_cast<int8_t>(c))) {
            return static_cast<int8_t>(c);
        }
    }
    return m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
}

result_t<void> AllocationCore::releaseClaimedController(int8_t controller)
{
    SerialGuard serial{_serial_mutex};
    // releaseExternal checks-and-clears under one pool lock and reports the
    // authoritative outcome: false means "not an external claim" (never
    // leased, already released, or an ordinary resolver lease), which is a
    // caller error here -- an ordinary lease is NOT freed by this path.
    if (!_pool.releaseExternal(controller)) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    return {};
}

int AllocationCore::allocTier(const types::AllocationIntent& a)
{
    if (a.forbid & types::backend_caps::HARDWARE) {
        return -1;
    }
    if (a.controller_mode == types::ControllerMode::Require && a.controller_id >= 0) {
        return 0;
    }
    if (a.require & types::backend_caps::HARDWARE) {
        return 1;
    }
    if ((a.prefer & types::backend_caps::HARDWARE) ||
        (a.controller_mode == types::ControllerMode::Prefer && a.controller_id >= 0)) {
        return 2;
    }
    return 3;
}

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_HAL_V2_BUS_ALLOCATION_CORE_INL_
