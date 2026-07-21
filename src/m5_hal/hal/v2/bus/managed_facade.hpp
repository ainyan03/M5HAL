// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_MANAGED_FACADE_HPP_
#define M5_HAL_BUS_MANAGED_FACADE_HPP_

#include "../data.hpp"
#include "../error.hpp"
#include "../types.hpp"
#include "./bus.hpp"
#include "./managed_bus.hpp"
#include "./registry.hpp"

#include <atomic>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

/*!
  @namespace m5::hal::v2::bus
  @brief Kind-neutral runtime bus facades.

  Every kind's runtime `Bus` shares a `unique_ptr`-held swappable backend,
  portable-backend adoption, `init`, `close`, and the
  lock-free backend-query mirror. That shared spine lives in the base all five
  kinds (i2c / spi / uart / i2s / pdm) derive from:

  - `FacadeCore<Traits>` — the shared `Bus` spine: backend ownership + `init` +
    `close` + the query mirror. It only needs a Traits with `IBus`,
    `IBusConfig`.

  The master kinds (i2c / spi) extend the facade with the intent +
  hot-swap surface:

  - `ManagedBusFacade<Traits>` — `FacadeCore<Traits>` plus `IManagedBus` (the
    master `transfer` / `waitTransfer` / `transferBusy`, the acquire intent,
    and the swap-under-lock seam the allocation resolver drives).

  A kind derives a thin `Bus` from these and supplies a `Traits` struct (see
  `i2c::BusTraits`). BusView is now a plain class in each kind header,
  delegating to `bus::IHalBackend`; the public type names (`i2c::Bus`, ...)
  stay exactly what callers and the compiler see.
 */
namespace m5::hal::v2::bus {

namespace detail {

// Keep the sequence-lock reader out of FacadeCore<Traits>: the algorithm and
// storage layout are identical for every bus kind, so one shared body avoids
// emitting five copies in embedded builds.
#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline))
#elif defined(_MSC_VER)
__declspec(noinline)
#endif
inline BusCapabilities
snapshotFacadeCapabilities(const std::atomic<uint32_t>& sequence, const std::atomic<uint32_t>& features,
                           const std::atomic<uint32_t>& limit_present, const std::atomic<uint32_t>* limits,
                           const std::atomic<uint32_t>& generation)
{
    BusCapabilitiesBuilder builder;
    for (;;) {
        const auto before = sequence.load(std::memory_order_acquire);
        if ((before & 1u) != 0) {
            continue;
        }
        builder = BusCapabilitiesBuilder{};
        builder.setRawFeatureMask(features.load(std::memory_order_relaxed));
        const auto present = limit_present.load(std::memory_order_relaxed);
        for (uint8_t i = 0; i < 4; ++i) {
            builder.setRawLimit(i, (present & (uint32_t{1} << i)) != 0, limits[i].load(std::memory_order_relaxed));
        }
        builder.setGeneration(generation.load(std::memory_order_relaxed));
        const auto after = sequence.load(std::memory_order_acquire);
        if (before == after) {
            return builder.build();
        }
    }
}

}  // namespace detail

/*!
  @brief Per-kind Traits contract consumed by the facade bases.

  This is documentation, not a base to inherit: each kind defines its own
  struct (e.g. `i2c::BusTraits`) next to its `Bus`. The members split by
  facade capability:

  - `FacadeCore<Traits>` (every kind's `Bus` spine) needs only `IBus` and
    `IBusConfig`. A kind whose `Bus` derives `FacadeCore`
    directly (uart / i2s) supplies a minimal Traits with just those three.
  - `ManagedBusFacade<Traits>` (master kinds) additionally needs
    `MasterAccessConfig`, `MasterAccessor`, `TransferDesc`, `LogicalBusConfig`,
    `applyAdopt`, and `fillLogical`.

  BusView and `LocalKindAdapter<Traits>` also consume `BusType`, `KIND`,
  `identityFromConfig`, and for managed kinds `CAPS_HARDWARE` plus
  `identityFromLogical`.

  The i2c / spi master Traits provide the full set:

  @code
  struct BusTraits {
      using IBus               = i2c::IBus;                 // concrete kind bus (FacadeCore)
      using IBusConfig         = i2c::IBusConfig;           // bus-level config base (FacadeCore)

      using LogicalBusConfig   = i2c::LogicalBusConfig;     // wiring + intent request (ManagedBusFacade)
      using MasterAccessConfig = i2c::MasterAccessConfig;   // accessor config (sentinel)
      using MasterAccessor     = i2c::MasterAccessor;       // accessor type (sentinel)
      using TransferDesc       = i2c::TransferDesc;         // transfer descriptor
      struct Bus;                                           // forward; the facade
      using BusType            = Bus;                        // BusView / LocalKindAdapter

      static constexpr types::bus_kind_t      KIND          = types::bus_kind_t::I2C;
      static constexpr types::backend_caps_t  CAPS_HARDWARE = i2c::caps::HARDWARE;

      static void applyAdopt(IBusConfig& cfg, const LogicalBusConfig& logical);  // logical -> _config pins
      static void fillLogical(LogicalBusConfig& out, const IBusConfig& cfg);     // _config pins -> logical
      static ResourceKey identityFromConfig(const IBusConfig& cfg);              // BusView typed path
      static ResourceKey identityFromLogical(const LogicalBusConfig& req);       // BusView logical path
  };
  @endcode
 */

/*!
  @brief Shared spine for every kind's runtime `Bus`.

  Inherits the kind's concrete `IBus` (so it owns the mutex + lock/unlock + the
  covariant `getConfig` reading the protected `_config`). It holds the concrete
  backend (`Bus_<variant>`) behind a `unique_ptr` and provides the parts every
  kind shares regardless of intent management:

  - portable provider initialization and backend adoption, followed by caching
    the pin config and backend query metadata.
  - `close` — close and drop the backend, refreshing the query mirror.
  - the lock-free backend-query mirror (`backendKind` / `controllerId` /
    `maxFrequency` / `backendGeneration`) read from facade-owned atomics so the
    query path never dereferences the live `_backend` during a swap.

  This base only needs the Traits subset `IBus` / `IBusConfig`;
  the master-only Traits members (`MasterAccessConfig`, `TransferDesc`,
  `LogicalBusConfig`, `applyAdopt`, ...) are required only by
  `ManagedBusFacade`. The protected `_backend`, `_generation`, `backend()`, and
  `_cacheBackendMeta()` are the seam `ManagedBusFacade` reaches when it swaps
  the backend under the bus lock; query atomics stay private (only this base's
  mirror and query readers touch them).

  A kind with no data-path beyond the base (none today) could derive `Bus`
  directly; uart / i2s derive it and add only their kind data-path overrides,
  while i2c / spi go through `ManagedBusFacade` for the intent + hot-swap surface.
 */
template <class Traits>
struct FacadeCore : public Traits::IBus {
    using IBus       = typename Traits::IBus;
    using IBusConfig = typename Traits::IBusConfig;

    FacadeCore(void) = default;
    ~FacadeCore(void) override
    {
        const bool abandoning = _registry != nullptr &&
                                _registry->beginAbandon(_registration, _resource_key, static_cast<const IBus*>(this));
        bool teardown_confirmed = true;
        if (_backend) {
            const auto outcome = this->closeOwnedBackend(*_backend);
            teardown_confirmed = outcome.disposition == CloseDisposition::Success;
        }
        // Backend destructors remain a best-effort second line of cleanup.
        // The observable close result above decides whether this identity
        // may be reused; an unknown/failed teardown leaves a quarantine.
        _backend.reset();
        if (abandoning) {
            _registry->finishAbandon(_registration, _resource_key, static_cast<const IBus*>(this), teardown_confirmed);
        }
    }

    FacadeCore(const FacadeCore&)            = delete;
    FacadeCore& operator=(const FacadeCore&) = delete;

    bool bindRegistryRegistration(BusRegistry& registry, const ResourceKey& key, uint16_t slot,
                                  uint32_t generation) override
    {
        if (_registry != nullptr || !key.isValid() || generation == 0) {
            return false;
        }
        _registry     = &registry;
        _resource_key = key;
        _registration = {slot, 0, generation};
        return _registration.valid();
    }

    const ResourceKey* registryResourceKey(void) const override
    {
        return _registration.valid() ? &_resource_key : nullptr;
    }

    /*!
      @brief Adopt a ready backend created from a portable configuration.

      Provider selection and backend initialization happen before this call.
      The facade owns the ready backend and retains only the kind-level config,
      keeping provider-native state out of the public portable object.
     */
    result_t<void> adoptPortableBackend(std::unique_ptr<IBus> backend, const IBusConfig& cfg)
    {
        if (!backend) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
        }
        if (_backend) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
        }
        const bool registry_bound = registryBound();
        if (!this->initializationAllowed(registry_bound)) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
        }
        auto initialized = this->markInitializationSucceeded(registry_bound);
        if (!initialized.has_value()) {
            (void)this->closeOwnedBackend(*backend);
            return initialized;
        }
        _backend      = std::move(backend);
        this->_config = cfg;
        _cacheBackendMeta(true);
        return {};
    }

    /*! @brief Close a directly owned facade and release its backend. */
    [[nodiscard]] result_t<void> close(void)
    {
        return bus::IBus::close();
    }

protected:
    CloseOutcome closeBackend(void) override
    {
        if (!_backend) {
            return CloseOutcome::success();
        }
        auto outcome = this->closeOwnedBackend(*_backend);
        if (outcome.disposition != CloseDisposition::Success) {
            return outcome;
        }
        _backend.reset();
        _cacheBackendMeta(true);
        return outcome;
    }

public:
    // Backend query API. To stay safe when another task commits and
    // hot-swaps the backend, metadata is mirrored into facade-owned atomics.
    // The legacy scalar queries remain individually atomic. capabilities()
    // additionally uses a sequence counter so its multi-field value always
    // comes from one committed backend generation.
    types::backend_kind_t backendKind(void) const override
    {
        return _q_kind.load(std::memory_order_acquire);
    }
    int8_t controllerId(void) const override
    {
        return _q_controller.load(std::memory_order_acquire);
    }
    uint32_t maxFrequency(void) const override
    {
        return _q_maxfreq.load(std::memory_order_acquire);
    }
    uint32_t backendGeneration(void) const override
    {
        return _generation.load(std::memory_order_acquire);
    }
    BusCapabilities capabilities(void) const override
    {
        return detail::snapshotFacadeCapabilities(_q_sequence, _q_features, _q_limit_present, _q_limits, _generation);
    }

protected:
    bool registryBound(void) const
    {
        return _registration.valid();
    }

    /*! @brief Live backend for kind-specific forwards (e.g. SPI begin/endOperation). */
    IBus* backend(void)
    {
        return _backend.get();
    }

    /*!
      @brief Forward a kind-specific operation to the live backend.

      The runtime facade owns the Access lifecycle lock / accessor binding and delegates
      only the data path to its current backend. Most kind-specific facade
      methods share the same guard: no backend means the operation is not
      implemented yet; otherwise invoke the backend method. Keeping that shape
      here avoids re-spelling it in every kind facade.
     */
    template <class Fn>
    auto forwardBackend(Fn&& fn) -> decltype(fn(std::declval<IBus&>()))
    {
        auto* b = backend();
        if (!b) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::UNSUPPORTED);
        }
        return fn(*b);
    }

    // Mirror the live backend's query metadata into the facade atomics. Called
    // wherever `_backend` changes (init / close / adopt / swap), under the
    // bus lock for the swap path. With no backend, the safe base defaults apply.
    void _cacheBackendMeta(bool bump_generation = false)
    {
        // Do not call IBus::capabilities() on this facade after detaching the
        // backend: its implementation dispatches backendKind()/maxFrequency()
        // virtually and would read the previous atomic mirror. A detached
        // facade starts from genuinely empty provider capabilities.
        auto caps = _backend ? _backend->capabilities() : BusCapabilities{};
        detail::BusCapabilitiesBuilder caps_builder{caps};
        if constexpr (Traits::MANAGED_ALLOCATION) {
            caps_builder.enable(BusFeature::ManagedAllocation);
        }
        caps = caps_builder.build();

        _q_sequence.fetch_add(1, std::memory_order_acq_rel);  // odd: writer active
        if (_backend) {
            _q_kind.store(_backend->backendKind(), std::memory_order_release);
            _q_controller.store(_backend->controllerId(), std::memory_order_release);
            _q_maxfreq.store(_backend->maxFrequency(), std::memory_order_release);
        } else {
            _q_kind.store(bus::IBus::backendKind(), std::memory_order_release);
            _q_controller.store(bus::IBus::controllerId(), std::memory_order_release);
            _q_maxfreq.store(bus::IBus::maxFrequency(), std::memory_order_release);
        }
        _q_features.store(detail::BusCapabilitiesBuilder::featureMask(caps), std::memory_order_relaxed);
        _q_limit_present.store(detail::BusCapabilitiesBuilder::limitPresentMask(caps), std::memory_order_relaxed);
        for (uint8_t i = 0; i < 4; ++i) {
            _q_limits[i].store(detail::BusCapabilitiesBuilder::limitValue(caps, i), std::memory_order_relaxed);
        }
        if (bump_generation) {
            _generation.fetch_add(1, std::memory_order_relaxed);
        }
        _q_sequence.fetch_add(1, std::memory_order_release);  // even: snapshot committed
    }

    std::unique_ptr<IBus> _backend;        // the swappable backend (hot-swap seam)
    std::atomic<uint32_t> _generation{0};  // bumped on every backend swap (poll baseline)

private:
    BusRegistry* _registry = nullptr;
    ResourceKey _resource_key{};
    RegistryEntryToken _registration{};
    // Query metadata mirror (read lock-free by the query API; written on swap).
    std::atomic<types::backend_kind_t> _q_kind{types::backend_kind_t::Software};
    std::atomic<int8_t> _q_controller{-1};
    std::atomic<uint32_t> _q_maxfreq{0};
    std::atomic<uint32_t> _q_sequence{0};
    std::atomic<uint32_t> _q_features{0};
    std::atomic<uint32_t> _q_limit_present{0};
    std::atomic<uint32_t> _q_limits[4]{};
};

/*!
  @brief Master runtime facade with the intent + hot-swap surface.

  Extends `FacadeCore<Traits>` (the shared backend spine) with `bus::IManagedBus`
  (the intent + hot-swap surface the resolver touches). `bus::IManagedBus` does
  NOT inherit `bus::IBus`, so there is no duplicate-`IBus` diamond: the facade
  has exactly one `bus::IBus` subobject, owned by `FacadeCore`.

  The concrete backend lives behind `FacadeCore`'s `unique_ptr`; `transfer` is
  forwarded to it while every lock/accessor-binding stays on THIS object. That
  is the hot-swap seam: a later commit can replace the backend under the facade
  lock without rebinding accessors. The backend the swap touches is
  the inherited `FacadeCore` member, reached through `this->_backend` /
  `this->_cacheBackendMeta()` / `this->_generation` (dependent-base names need
  the `this->` qualification under two-phase lookup).

  A kind derives `Bus` from this and implements its protected checked-facade
  backend hooks there. The kind hook validates once on the facade object and
  reaches the owned backend through `FacadeCore::forwardBackend`; this generic
  ownership spine deliberately has no raw transfer forwarding surface.
 */
template <class Traits>
struct ManagedBusFacade : public FacadeCore<Traits>, public IManagedBus {
    using IBus               = typename Traits::IBus;
    using IBusConfig         = typename Traits::IBusConfig;
    using LogicalBusConfig   = typename Traits::LogicalBusConfig;
    using MasterAccessConfig = typename Traits::MasterAccessConfig;
    using MasterAccessor     = typename Traits::MasterAccessor;
    using TransferDesc       = typename Traits::TransferDesc;

    ManagedBusFacade(void)                               = default;
    ManagedBusFacade(const ManagedBusFacade&)            = delete;
    ManagedBusFacade& operator=(const ManagedBusFacade&) = delete;

    /*!
      @brief Adopt a ready (already-`init`-ed) backend for a logical acquire.

      Backs the logical acquire path (`BusView::acquire(LogicalBusConfig)`):
      the view builds the backend through a factory and hands it here, where
      the facade caches the wiring (for `getConfig`/`probe`) and records the
      acquire intent for the commit-time resolver. Marks the bus as
      intent-managed so `commitBuses()` may reassign it.
     */
    result_t<void> adoptBackend(std::unique_ptr<IBus> backend, const LogicalBusConfig& logical)
    {
        if (!backend) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
        }
        const bool registry_bound = this->registryBound();
        if (!this->initializationAllowed(registry_bound)) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
        }
        auto initialized = this->markInitializationSucceeded(registry_bound);
        if (!initialized.has_value()) {
            (void)this->closeOwnedBackend(*backend);
            return initialized;
        }
        this->_backend = std::move(backend);
        Traits::applyAdopt(this->_config, logical);
        _intent  = logical.intent;
        _managed = true;
        this->_cacheBackendMeta(true);
        return {};
    }

    /*! @brief The acquire intent recorded for this bus (commit-time resolver). */
    types::AllocationIntent intent(void) const override
    {
        return _intent;
    }

    /*!
      @brief Whether this bus opted into intent-driven management.

      Only buses acquired through the logical path (`acquire(LogicalBusConfig)`)
      are managed; a bus pinned to a specific backend through the typed
      `acquire<CfgT>` / `init<CfgT>` route is NOT managed, so `commitBuses()`
      leaves its explicitly chosen backend untouched (it never had an intent).
     */
    bool managed(void) const override
    {
        return _managed;
    }

    /*!
      @brief Update the recorded acquire intent (a re-acquire re-tags the bus).

      The commit-time resolver acts on the latest intent, so promoting a bus
      to `requireHardware()` (or demoting to `software()`) is a logical
      re-acquire followed by `commitBuses()`. `AllocationCore` calls this hook
      under the same short lock used to snapshot a commit's intents;
      re-tagging also marks a typed-created facade intent-managed. An update
      after the snapshot belongs to the next commit.
     */
private:
    void retagIntent(const types::AllocationIntent& intent) override
    {
        _intent  = intent;
        _managed = true;
    }

public:
    /*!
      @brief Rebuild the logical request from the recorded wiring + intent.

      The commit-time resolver hands this to a backend factory to re-make the
      backend (software or hardware) for this bus during a swap.
     */
    LogicalBusConfig logicalConfig(const types::AllocationIntent& intent) const
    {
        LogicalBusConfig logical;
        Traits::fillLogical(logical, this->_config);
        logical.intent = intent;
        return logical;
    }

    /*!
      @brief Close the old backend, then build (under the lock) and adopt a
             new one via a factory, with a rollback factory for a failed
             build (hot-swap).

      Takes the facade's lock -- the same one accessors contend on -- through a
      stack sentinel, so a swap waits for any in-flight transfer to finish and,
      with a finite `timeout_ms`, returns `TIMEOUT_ERROR` instead of wedging
      when the bus stays busy. Processing order, all under the lock:

       1. The OLD backend (if any) is closed and dropped FIRST, before the new
          one is built. A NoMutation failure keeps both backend and facade
          live and propagates the exact error. A PartialOrUnknown failure also
          keeps the backend, but quarantines the facade before releasing the
          sentinel lock so later Access attempts return `CLOSED`.
       2. `make()` runs to build the new backend; its `init()` (which drives
          pins / installs a driver) is guarded by the same lock accessors
          contend on, so it cannot race an in-flight transfer.
       3. A null `make()` result (unless `allow_null`, the software-less
          "pending" path) means the new backend could not be built with the
          old one already closed: `rollback()` is invoked to try to
          reconstruct the old configuration (a fresh backend for the same
          kind/controller the old one had). Whatever `rollback()` returns is
          adopted -- a real backend on success, or null ("pending") if it
          also fails -- and `OUT_OF_RESOURCE` is returned either way (a
          degraded bus is never left silent).

      Releasing before building (rather than the reverse) removes two hazards
      a build-first order has: two initialized backends never coexist on the
      same pins (a new backend's `init()` racing the old backend's
      not-yet-run close), and an aborted swap never leaves an
      initialized-but-unused backend for a `unique_ptr` to destroy (whose
      dtor would run its own close and disturb the pins the "kept" old
      backend still owns). The cost is that `rollback()` is a best-effort
      re-`init()`, not a guarantee (a driver re-install can fail for the
      same reason the build did, e.g. OOM) -- callers must still check the
      returned `result_t`.

      The query metadata is refreshed and the generation counter is bumped
      LAST in every adopted outcome (success, or the null/rollback fallback)
      so a poller that sees the new generation reads settled metadata.

      `make()`/`rollback()` return a raw `bus::IBus*`; it is always this
      kind's `IBus` (this kind's factory made it), so it is downcast to the
      typed backend the facade owns.
     */
    template <class MakeBackend, class RollbackBackend>
    result_t<void> swapBackendWith(uint32_t timeout_ms, bool allow_null, MakeBackend&& make, RollbackBackend&& rollback)
    {
        MasterAccessConfig sentinel_cfg;
        MasterAccessor sentinel{*this, sentinel_cfg};
        return bus::guarded([&] { return this->acquireAccessLock(sentinel, timeout_ms); },
                            [&]() -> result_t<void> {
                                if (this->_backend) {
                                    const auto outcome = this->closeOwnedBackend(*this->_backend);
                                    if (outcome.disposition != CloseDisposition::Success) {
                                        if (outcome.disposition == CloseDisposition::PartialOrUnknown) {
                                            // The sentinel already owns the Access mutex. Mark
                                            // the facade quarantined before guarded() releases
                                            // that mutex, so no later Access can observe the
                                            // uncertain backend as live.
                                            this->quarantineLifecycleAfterPartialTeardown();
                                        }
                                        return m5::stl::make_unexpected(outcome.error_code);
                                    }
                                    this->_backend.reset();
                                }
                                bus::IBus* raw = make();  // init() runs here, under the lock, old backend already gone
                                if (raw == nullptr && !allow_null) {
                                    // The new backend could not be built and the old one is
                                    // already closed: try to reconstruct it instead of
                                    // leaving the bus backend-less.
                                    this->_backend.reset(static_cast<IBus*>(rollback()));
                                    this->_cacheBackendMeta(true);
                                    return m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
                                }
                                this->_backend.reset(static_cast<IBus*>(raw));
                                this->_cacheBackendMeta(true);
                                return {};
                            },
                            [&] { return this->releaseAccessLock(sentinel); });
    }

    /*!
      @brief `swapBackendWith` without a rollback factory.

      A failed `make()` detaches to null ("pending") instead of reconstructing
      the old backend -- equivalent to a rollback factory that always returns
      null. Used by the strict non-null (`swapBackend`) and the software-less
      pending-detach (`swapPending`) paths, where `make()` cannot fail for a
      reason a real rollback would help with (their own docs), and by callers
      (e.g. tests) that have no old configuration worth reconstructing.
     */
    template <class MakeBackend>
    result_t<void> swapBackendWith(uint32_t timeout_ms, bool allow_null, MakeBackend&& make)
    {
        return swapBackendWith(timeout_ms, allow_null, std::forward<MakeBackend>(make),
                               []() -> bus::IBus* { return nullptr; });
    }

    /*!
      @brief Hot-swap to a ready (already-`init`-ed) backend under the bus lock.

      The strict non-null path. The commit-time resolver instead uses
      `IAllocationKind::commitHardware` so `init()` happens under the lock; this
      overload remains for callers that already hold an initialized backend
      (e.g. tests). Close errors propagate as in `swapBackendWith`.

      WEAKER GUARANTEE than the factory path: the caller's backend is already
      initialized before the old one is closed, so the two coexist until the
      swap completes, and a close-failure abort destroys the provided
      backend while its dtor's own close runs beside the kept old one.
      Do not use this overload to swap backends that share a physical
      resource (pins / controller) with the current backend -- build those
      through a factory (`swapBackendWith`) so init() happens after the old
      close.
     */
    result_t<void> swapBackend(std::unique_ptr<bus::IBus> new_backend,
                               uint32_t timeout_ms = types::TIMEOUT_FOREVER) override
    {
        if (!new_backend) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
        }
        // Keep ownership inside the lambda until make() actually runs: if lock
        // acquisition fails, swapBackendWith never invokes make, and the moved
        // unique_ptr frees the prepared backend when the lambda is destroyed (no
        // leak). On success make() releases ownership to the locked body.
        return swapBackendWith(timeout_ms, /*allow_null=*/false,
                               [nb = std::move(new_backend)]() mutable -> bus::IBus* { return nb.release(); });
    }

    /*!
      @brief Detach to a null (pending) backend under the bus lock.

      The software-less reassignment path: a kind with no software placeholder
      (not i2c / spi) has its donor bus detached here instead of dropped to
      software. A kind whose placeholder is always a software backend never
      reaches this path, so its behaviour is unchanged.
     */
    result_t<void> swapPending(uint32_t timeout_ms = types::TIMEOUT_FOREVER) override
    {
        return swapBackendWith(timeout_ms, /*allow_null=*/true, []() -> bus::IBus* { return nullptr; });
    }

private:
    // Acquire intent, recorded for the commit-time resolver.
    types::AllocationIntent _intent{};  // capability-based allocation request
    bool _managed = false;              // true => commitBuses() may reassign (logical acquire only)
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_MANAGED_FACADE_HPP_
