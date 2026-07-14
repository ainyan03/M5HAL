// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_LOCAL_BACKEND_HPP_
#define M5_HAL_BUS_LOCAL_BACKEND_HPP_

#include "./allocation_core.hpp"
#include "./hal_backend.hpp"
#include "./managed_bus.hpp"

#include <new>

/*!
  @namespace m5::hal::v2::bus
  @brief LocalBackend: the local (on-device) IHalBackend implementation.
 */
namespace m5::hal::v2::bus {

//-------------------------------------------------------------------------
/*!
  @brief Extended kind adapter for LocalBackend's logical acquire path.

  Adds facade creation and intent retagging to the base IAllocationKind.
  The AllocationCore only needs IAllocationKind (factory + commit hooks);
  LocalBackend additionally needs to create facades on a logical-acquire
  miss and retag intents on a hit.
 */
struct ILocalKindAdapter : public IAllocationKind {
    /*!
      @brief Create a new facade with software backend for a logical miss.

      The `req.config` points to the kind-specific `LogicalBusConfig` (borrowed).
      Concrete adapters cast it to the correct type and call the software factory.
     */
    virtual result_t<std::shared_ptr<IBus>> createLogicalFacade(const AllocationRequest& req) const = 0;

    /*!
      @brief Re-tag the acquire intent on an existing bus (logical hit).

      A re-acquire of the same wiring with a different intent updates the bus's
      recorded intent for the next `commitBuses()` resolution.
     */
    virtual void retagIntent(IBus& bus, const AllocationRequest& req) = 0;

    /*!
      @brief Access the kind's AllocationCore (null for non-intent kinds).

      Intent-driven kinds (I2C/SPI) return their embedded core; static kinds
      (UART/I2S) return nullptr. Used by LocalBackend for commitBuses and
      hardwareInUse.
     */
    virtual AllocationCore* allocationCore(void)             = 0;
    virtual const AllocationCore* allocationCore(void) const = 0;

    /*!
      @brief Complete/validate a logical request before interning (pin
             auto-fill for single-element pin domains, early
             `INVALID_ARGUMENT` on mismatch). Default: no-op.

      `logical_config` borrows the kind's `LogicalBusConfig` (type-erased
      here so `IHalBackend::completeLogicalRequest` can call through it
      without knowing the kind); a concrete adapter casts it back.
     */
    virtual result_t<void> completeLogical(void* logical_config) const
    {
        (void)logical_config;
        return {};
    }
};

//-------------------------------------------------------------------------
/*!
  @brief Per-kind adapter template for LocalBackend.

  Implements ILocalKindAdapter (facade creation + intent retag) and the full
  IAllocationKind interface (factory hooks + commit hooks used by AllocationCore).
  Extracted from the former `BusViewBase<Traits>` — the same logic, now owned
  by LocalBackend instead of BusView.

  The AllocationCore is embedded as a member: the adapter IS the kind's
  allocation policy owner. The core is constructed in the adapter's ctor and
  shares the backend's BusRegistry.

  Traits contract: same as the former BusViewBase Traits — `IBus`, `BusType`,
  `LogicalBusConfig`, `KIND`, `CAPS_HARDWARE`, `applyAdopt`, `fillLogical`,
  and the `BackendFor` selector. `SoftwareBackendFactory` / `HardwareBackendFactory`
  follow the BusViewBase convention: function pointers taking `LogicalBusConfig`.
 */
template <class Traits>
class LocalKindAdapter : public ILocalKindAdapter {
public:
    using KindIBus         = typename Traits::IBus;
    using BusType          = typename Traits::BusType;
    using LogicalBusConfig = typename Traits::LogicalBusConfig;
    using SwFactory        = KindIBus* (*)(const LogicalBusConfig&);
    using HwFactory        = KindIBus* (*)(const LogicalBusConfig&, int8_t controller);

    /*!
      @brief Non-uniform controller topology: the pieces a kind's variant
             supplies when its controllers are NOT interchangeable (e.g. an
             espidf I2C build with an LP_I2C instance).

      Every field is optional and defaults to "uniform, no restriction", so a
      kind that never passes a `Topology` (all existing call sites) keeps the
      original uniform behaviour exactly.
     */
    struct Topology {
        /// Per-controller capability bitmask; nullptr means uniform
        /// (`Traits::CAPS_HARDWARE` for every controller).
        types::backend_caps_t (*controller_caps)(int8_t controller) = nullptr;
        /// Capability bits that require explicit opt-in (see
        /// `IAllocationKind::optInCaps`).
        types::backend_caps_t opt_in = 0;
        /// Validates/auto-fills a logical request before interning (see
        /// `ILocalKindAdapter::completeLogical`); nullptr means no-op.
        result_t<void> (*complete_logical)(LogicalBusConfig&) = nullptr;
        /// Pin-domain membership check (see `IAllocationKind::controllerAcceptsBus`);
        /// nullptr means every controller accepts every wiring.
        bool (*pins_allowed)(const LogicalBusConfig&, int8_t controller) = nullptr;
    };

    LocalKindAdapter(BusRegistry& registry, SwFactory sw, HwFactory hw = nullptr, uint8_t hw_capacity = 0,
                     Topology topo = {})
        : _sw_factory{sw}, _hw_factory{hw}, _capacity{hw_capacity}, _core{registry, *this, hw_capacity}, _topo{topo}
    {
    }

    LocalKindAdapter(const LocalKindAdapter&)            = delete;
    LocalKindAdapter& operator=(const LocalKindAdapter&) = delete;

    AllocationCore* allocationCore(void) override
    {
        return &_core;
    }
    const AllocationCore* allocationCore(void) const override
    {
        return &_core;
    }

    // --- ILocalKindAdapter ------------------------------------------------

    result_t<std::shared_ptr<IBus>> createLogicalFacade(const AllocationRequest& req) const override
    {
        const auto& logical = *static_cast<const LogicalBusConfig*>(req.config);
        std::shared_ptr<BusType> facade{new (std::nothrow) BusType()};
        if (!facade) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        std::unique_ptr<KindIBus> sw{_sw_factory != nullptr ? _sw_factory(logical) : nullptr};
        if (!sw) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        auto r = facade->adoptBackend(std::move(sw), logical);
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
        return std::shared_ptr<IBus>{facade};
    }

    void retagIntent(IBus& bus, const AllocationRequest& req) override
    {
        const auto& logical = *static_cast<const LogicalBusConfig*>(req.config);
        _core.retagIntent(static_cast<BusType&>(bus), logical.intent);
    }

    // --- IAllocationKind --------------------------------------------------

    types::bus_kind_t kind(void) const override
    {
        return Traits::KIND;
    }
    uint8_t controllerCapacity(void) const override
    {
        return _capacity;
    }
    bool hasHardware(void) const override
    {
        return _hw_factory != nullptr;
    }
    IManagedBus& toManaged(IBus& b) const override
    {
        return static_cast<BusType&>(b);
    }
    IBus* makePlaceholder(IManagedBus& mb, const types::AllocationIntent& intent) const override
    {
        auto& self = static_cast<BusType&>(mb);
        return _sw_factory != nullptr ? _sw_factory(self.logicalConfig(intent)) : nullptr;
    }
    IBus* makeHardware(IManagedBus& mb, const types::AllocationIntent& intent, int8_t controller) const override
    {
        auto& self = static_cast<BusType&>(mb);
        return _hw_factory != nullptr ? _hw_factory(self.logicalConfig(intent), controller) : nullptr;
    }
    result_t<void> commitPlaceholder(IManagedBus& mb, const types::AllocationIntent& intent,
                                     uint32_t timeout_ms) const override
    {
        auto& self = static_cast<BusType&>(mb);
        // The resolver only routes a bus here while it is CURRENTLY hardware
        // (AllocationCore's demote step), so the pre-swap controller read
        // below is always the one the swap is about to give up: on a failed
        // rebuild, the rollback factory re-makes hardware on that same
        // controller instead of leaving the bus without a backend.
        const int8_t cur = self.controllerId();
        // A null placeholder is a valid outcome ONLY for a software-less
        // kind (makePlaceholder() returns null by design there, not by
        // failure). A kind WITH a software factory returning null instead
        // means the build itself failed (e.g. OOM): that must be treated as
        // a swap failure so the rollback factory below runs, rather than
        // silently landing the bus on no backend at all.
        return self.swapBackendWith(
            timeout_ms, /*allow_null=*/_sw_factory == nullptr,
            [&]() -> IBus* { return this->makePlaceholder(mb, intent); },
            [this, &mb, &intent, cur]() -> IBus* { return this->makeHardware(mb, intent, cur); });
    }
    result_t<void> commitHardware(IManagedBus& mb, const types::AllocationIntent& intent, int8_t controller,
                                  uint32_t timeout_ms) const override
    {
        auto& self = static_cast<BusType&>(mb);
        // The resolver only routes a bus here once it is off hardware (see
        // commitPlaceholder above -- a same-pass demote runs first), so the
        // rollback factory recreating the placeholder is always correct: for
        // a kind with a software placeholder it rebuilds that backend, and
        // for a software-less kind makePlaceholder() itself returns null,
        // which is the correct pending fallback.
        return self.swapBackendWith(
            timeout_ms, /*allow_null=*/false, [&]() -> IBus* { return this->makeHardware(mb, intent, controller); },
            [this, &mb, &intent]() -> IBus* { return this->makePlaceholder(mb, intent); });
    }
    bool uniformControllers(void) const override
    {
        return _topo.controller_caps == nullptr;
    }
    types::backend_caps_t controllerCaps(int8_t controller) const override
    {
        return _topo.controller_caps != nullptr ? _topo.controller_caps(controller) : Traits::CAPS_HARDWARE;
    }
    types::backend_caps_t optInCaps(void) const override
    {
        return _topo.opt_in;
    }
    bool controllerAcceptsBus(const IManagedBus& bus, const types::AllocationIntent& intent,
                              int8_t controller) const override
    {
        return _topo.pins_allowed != nullptr
                   ? _topo.pins_allowed(static_cast<const BusType&>(bus).logicalConfig(intent), controller)
                   : true;
    }

    // --- ILocalKindAdapter (continued) ------------------------------------

    result_t<void> completeLogical(void* logical_config) const override
    {
        if (_topo.complete_logical == nullptr) {
            return {};
        }
        return _topo.complete_logical(*static_cast<LogicalBusConfig*>(logical_config));
    }

private:
    SwFactory _sw_factory = nullptr;
    HwFactory _hw_factory = nullptr;
    uint8_t _capacity     = 0;
    AllocationCore _core;
    Topology _topo{};
};

//-------------------------------------------------------------------------
/*!
  @brief Local (on-device) backend: owns the bus registry and per-kind adapters.

  Implements IHalBackend for the local case. The typed acquire path is handled
  by BusView calling `busRegistry().acquireOrFind()` directly (template make-
  lambda, no virtual needed). The logical path and commit go through virtuals.

  Per-kind adapters are registered at construction time (M5HALCore ctor) and
  borrowed (not owned). The adapters live as M5HALCore members, declared before
  the BusViews in member-init order. Each adapter owns its AllocationCore.
 */
class LocalBackend : public IHalBackend {
public:
    LocalBackend(void) = default;

    LocalBackend(const LocalBackend&)            = delete;
    LocalBackend& operator=(const LocalBackend&) = delete;

    void registerKind(ILocalKindAdapter& adapter);

    // --- IHalBackend ------------------------------------------------------

    result_t<std::shared_ptr<IBus>> acquireBusLogical(types::bus_kind_t kind, const IdentityKey& id,
                                                      const AllocationRequest& req) override;

    result_t<void> completeLogicalRequest(types::bus_kind_t kind, void* logical_config) override;

    result_t<void> commitBuses(types::bus_kind_t kind, uint32_t timeout_ms) override;

    uint8_t hardwareInUse(types::bus_kind_t kind) const override;

    result_t<int8_t> claimController(types::bus_kind_t kind, const types::AllocationIntent& intent) override;

    result_t<void> releaseClaimedController(types::bus_kind_t kind, int8_t controller) override;

private:
    static constexpr size_t kMaxKinds = 5;

    struct KindSlot {
        ILocalKindAdapter* adapter = nullptr;
    };
    KindSlot _slots[kMaxKinds];

    static size_t kindIndex(types::bus_kind_t k);
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_LOCAL_BACKEND_HPP_
