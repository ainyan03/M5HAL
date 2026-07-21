// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_LOCAL_BACKEND_HPP_
#define M5_HAL_BUS_LOCAL_BACKEND_HPP_

#include "./allocation_core.hpp"
#include "./hal_backend.hpp"
#include "./managed_bus.hpp"
#include "../resource_domain.hpp"

#include <new>

/*!
  @namespace m5::hal::v2::bus
  @brief LocalBackend: the local (on-device) IHalBackend implementation.
 */
namespace m5::hal::v2::bus {

//-------------------------------------------------------------------------
/*!
  @brief Type-erased local provider for portable acquisition.

  A provider is registered independently of a config's concrete C++ type.
  It validates registry hits with the kind Traits and builds a ready backend
  on misses. Provider objects are borrowed and must outlive LocalBackend.
 */
struct ILocalPortableProvider {
    virtual types::bus_kind_t kind(void) const                                                  = 0;
    virtual ResourceKey identityFromConfig(const IBusConfig& cfg) const                         = 0;
    virtual bool configCompatible(const IBusConfig& current, const IBusConfig& requested) const = 0;
    virtual result_t<std::shared_ptr<IBus>> createFacade(const LocalResourceContext& resources,
                                                         const IBusConfig& cfg) const           = 0;
    virtual ~ILocalPortableProvider()                                                           = default;
};

/*!
  @brief Traits adapter for a selected variant's portable backend factory.

  `Factory` receives only the kind-level config and returns an initialized
  backend. The adapter wraps it in the kind's runtime facade so all providers
  share the same registry and public bus type.
 */
template <class Traits>
class LocalPortableProvider : public ILocalPortableProvider {
public:
    using KindIBus   = typename Traits::IBus;
    using KindConfig = typename Traits::IBusConfig;
    using BusType    = typename Traits::BusType;
    using Factory    = result_t<std::unique_ptr<KindIBus>> (*)(const LocalResourceContext&, const KindConfig&);

    explicit LocalPortableProvider(Factory factory) : _factory{factory}
    {
    }

    types::bus_kind_t kind(void) const override
    {
        return Traits::KIND;
    }

    ResourceKey identityFromConfig(const IBusConfig& cfg) const override
    {
        return Traits::identityFromConfig(static_cast<const KindConfig&>(cfg));
    }

    bool configCompatible(const IBusConfig& current, const IBusConfig& requested) const override
    {
        return Traits::configCompatible(static_cast<const KindConfig&>(current),
                                        static_cast<const KindConfig&>(requested));
    }

    result_t<std::shared_ptr<IBus>> createFacade(const LocalResourceContext& resources,
                                                 const IBusConfig& cfg) const override
    {
        if (_factory == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
        }
        auto made = _factory(resources, static_cast<const KindConfig&>(cfg));
        if (!made.has_value()) {
            return m5::stl::make_unexpected(made.error());
        }
        if (!made.value()) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        std::shared_ptr<BusType> facade{new (std::nothrow) BusType()};
        if (!facade) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        facade->bindLocalResources(resources);
        auto adopted = facade->adoptPortableBackend(std::move(made.value()), static_cast<const KindConfig&>(cfg));
        if (!adopted.has_value()) {
            return m5::stl::make_unexpected(adopted.error());
        }
        return std::shared_ptr<IBus>{std::move(facade)};
    }

private:
    Factory _factory = nullptr;
};

//-------------------------------------------------------------------------
/*!
  @brief Extended kind adapter for LocalBackend's logical acquire path.

  Adds facade creation and intent retagging to the base IAllocationKind.
  The AllocationCore only needs IAllocationKind (planning + atomic commit hooks);
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
      (UART/I2S/PDM) return nullptr. Used by LocalBackend for commitBuses and
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
  IAllocationKind interface (planning + commit hooks used by AllocationCore).
  Extracted from the former `BusViewBase<Traits>` — the same logic, now owned
  by LocalBackend instead of BusView.

  The AllocationCore is embedded as a member: the adapter IS the kind's
  allocation policy owner. The core is constructed in the adapter's ctor and
  shares the backend's BusRegistry.

  Traits contract: same as the former BusViewBase Traits — `IBus`, `BusType`,
  `LogicalBusConfig`, `KIND`, `CAPS_HARDWARE`, `applyAdopt`, `fillLogical`,
  `SoftwareBackendFactory` / `HardwareBackendFactory`
  follow the BusViewBase convention: function pointers taking `LogicalBusConfig`.
 */
template <class Traits>
class LocalKindAdapter : public ILocalKindAdapter {
public:
    using KindIBus         = typename Traits::IBus;
    using BusType          = typename Traits::BusType;
    using LogicalBusConfig = typename Traits::LogicalBusConfig;
    using SwFactory        = KindIBus* (*)(const LocalResourceContext&, const LogicalBusConfig&);
    using HwFactory        = KindIBus* (*)(const LocalResourceContext&, const LogicalBusConfig&, int8_t controller);

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
                     Topology topo = {}, LocalResourceContext resources = {})
        : _sw_factory{sw},
          _hw_factory{hw},
          _capacity{hw_capacity},
          _core{registry, *this, hw_capacity},
          _topo{topo},
          _resources{std::move(resources)}
    {
    }

    LocalKindAdapter(const LocalKindAdapter&)            = delete;
    LocalKindAdapter& operator=(const LocalKindAdapter&) = delete;

    void setLocalResources(LocalResourceContext resources)
    {
        _resources = std::move(resources);
    }
    void bindLifetime(const std::shared_ptr<void>& lifetime)
    {
        _lifetime = lifetime;
    }

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
        auto resources      = localResources();
        std::shared_ptr<BusType> facade{new (std::nothrow) BusType()};
        if (!facade) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        facade->bindLocalResources(resources);
        std::unique_ptr<KindIBus> sw{_sw_factory != nullptr ? _sw_factory(resources, logical) : nullptr};
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
        // kind (_makePlaceholder() returns null by design there, not by
        // failure). A kind WITH a software factory returning null instead
        // means the build itself failed (e.g. OOM): that must be treated as
        // a swap failure so the rollback factory below runs, rather than
        // silently landing the bus on no backend at all.
        return self.swapBackendWith(
            timeout_ms, /*allow_null=*/_sw_factory == nullptr,
            [&]() -> IBus* { return this->_makePlaceholder(mb, intent); },
            [this, &mb, &intent, cur]() -> IBus* { return this->_makeHardware(mb, intent, cur); });
    }
    result_t<void> commitHardware(IManagedBus& mb, const types::AllocationIntent& intent, int8_t controller,
                                  uint32_t timeout_ms) const override
    {
        auto& self = static_cast<BusType&>(mb);
        // The resolver only routes a bus here once it is off hardware (see
        // commitPlaceholder above -- a same-pass demote runs first), so the
        // rollback factory recreating the placeholder is always correct: for
        // a kind with a software placeholder it rebuilds that backend, and
        // for a software-less kind _makePlaceholder() itself returns null,
        // which is the correct pending fallback.
        return self.swapBackendWith(
            timeout_ms, /*allow_null=*/false, [&]() -> IBus* { return this->_makeHardware(mb, intent, controller); },
            [this, &mb, &intent]() -> IBus* { return this->_makePlaceholder(mb, intent); });
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
    LocalResourceContext localResources(void) const
    {
        auto resources = _resources;
        if (auto lifetime = _lifetime.lock()) {
            resources.lifetime = std::move(lifetime);
        }
        return resources;
    }

    IBus* _makePlaceholder(IManagedBus& mb, const types::AllocationIntent& intent) const
    {
        auto& self     = static_cast<BusType&>(mb);
        auto resources = localResources();
        return _sw_factory != nullptr ? _sw_factory(resources, self.logicalConfig(intent)) : nullptr;
    }

    IBus* _makeHardware(IManagedBus& mb, const types::AllocationIntent& intent, int8_t controller) const
    {
        auto& self     = static_cast<BusType&>(mb);
        auto resources = localResources();
        return _hw_factory != nullptr ? _hw_factory(resources, self.logicalConfig(intent), controller) : nullptr;
    }

    SwFactory _sw_factory = nullptr;
    HwFactory _hw_factory = nullptr;
    uint8_t _capacity     = 0;
    AllocationCore _core;
    Topology _topo{};
    LocalResourceContext _resources;
    std::weak_ptr<void> _lifetime;
};

//-------------------------------------------------------------------------
/*!
  @brief Local backend bound to a co-owned ResourceDomain.

  Implements IHalBackend for the local case. Portable acquire is routed to a
  registered type-erased provider, while the logical path is routed to the
  allocation adapter. Both paths intern facades in the domain registry.

  The registry and GPIO/Services/Memory dependencies come from the injected
  domain. Per-kind adapters are borrowed from the owning local connection
  state; each allocation adapter owns its AllocationCore, while portable
  provider adapters hold the selected variant factory.
 */
class LocalBackend : public IHalBackend {
public:
    LocalBackend(void) = default;
    explicit LocalBackend(const ResourceDomain& domain) : _domain{domain}
    {
    }

    LocalBackend(const LocalBackend&)            = delete;
    LocalBackend& operator=(const LocalBackend&) = delete;

    void registerKind(ILocalKindAdapter& adapter);
    void registerPortableProvider(ILocalPortableProvider& provider);

    BusRegistry& busRegistry(void) override
    {
        return _domain.busRegistry();
    }
    const BusRegistry& busRegistry(void) const override
    {
        return _domain.busRegistry();
    }

    const ResourceDomain& resourceDomain(void) const
    {
        return _domain;
    }
    const ResourceDomain* localResourceDomain(void) const override
    {
        return &_domain;
    }
    LocalResourceContext localResources(void) const override
    {
        auto resources = _domain.localResources();
        if (auto lifetime = _lifetime.lock()) {
            resources.lifetime = std::move(lifetime);
        }
        return resources;
    }
    void bindLifetime(const std::shared_ptr<void>& lifetime)
    {
        _lifetime = lifetime;
    }

    // --- IHalBackend ------------------------------------------------------

    result_t<std::shared_ptr<IBus>> acquireBusPortable(types::bus_kind_t kind, const ResourceKey& id,
                                                       const IBusConfig& cfg) override;

    result_t<std::shared_ptr<IBus>> acquireBusLogical(types::bus_kind_t kind, const ResourceKey& id,
                                                      const AllocationRequest& req) override;

    result_t<void> completeLogicalRequest(types::bus_kind_t kind, void* logical_config) override;

    result_t<void> commitBuses(types::bus_kind_t kind, uint32_t timeout_ms) override;

    uint8_t hardwareInUse(types::bus_kind_t kind) const override;

    result_t<int8_t> claimController(types::bus_kind_t kind, const types::AllocationIntent& intent) override;

    result_t<void> releaseClaimedController(types::bus_kind_t kind, int8_t controller) override;

private:
    ResourceDomain _domain;
    std::weak_ptr<void> _lifetime;
    static constexpr size_t kMaxKinds = 6;

    struct KindSlot {
        ILocalKindAdapter* adapter                = nullptr;
        ILocalPortableProvider* portable_provider = nullptr;
    };
    KindSlot _slots[kMaxKinds];

    static size_t kindIndex(types::bus_kind_t k);
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_LOCAL_BACKEND_HPP_
