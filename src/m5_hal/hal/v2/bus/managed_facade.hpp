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
  @brief Kind-neutral runtime bus facades (ADR 034 phase D/E).

  Every kind's runtime `Bus` shares a `unique_ptr`-held swappable backend, a
  templated `init`, a `release`, and the lock-free backend-query mirror. That
  shared spine lives in the phase-1/2 base all four kinds (i2c / spi / uart /
  i2s) derive from:

  - `FacadeCore<Traits>` — the shared `Bus` spine: backend ownership + `init` +
    `release` + the query mirror. It only needs a Traits with `IBus`,
    `IBusConfig`, and `BackendFor`.

  The master kinds (i2c / spi) extend the facade with the phase-3 intent +
  hot-swap surface:

  - `ManagedBusFacade<Traits>` — `FacadeCore<Traits>` plus `IManagedBus` (the
    master `transfer`, the acquire intent, and the swap-under-lock seam the
    allocation resolver drives).

  A kind derives a thin `Bus` from these and supplies a `Traits` struct (see
  `i2c::BusTraits`). BusView is now a plain class in each kind header,
  delegating to `bus::IHalBackend`; the public type names (`i2c::Bus`, ...)
  stay exactly what callers and the compiler see.
 */
namespace m5::hal::v2::bus {

/*!
  @brief Per-kind Traits contract consumed by the facade bases.

  This is documentation, not a base to inherit: each kind defines its own
  struct (e.g. `i2c::BusTraits`) next to its `Bus`. The members split by
  facade capability:

  - `FacadeCore<Traits>` (every kind's `Bus` spine) needs only `IBus`,
    `IBusConfig`, and `BackendFor`. A kind whose `Bus` derives `FacadeCore`
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
      template <class CfgT> using BackendFor = i2c::BackendFor<CfgT>;  // variant selector (FacadeCore)

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
      static IdentityKey identityFromConfig(const IBusConfig& cfg);              // BusView typed path
      static IdentityKey identityFromLogical(const LogicalBusConfig& req);       // BusView logical path
  };
  @endcode
 */

/*!
  @brief Shared phase-1/2 spine for every kind's runtime `Bus`.

  Inherits the kind's concrete `IBus` (so it owns the mutex + lock/unlock + the
  covariant `getConfig` reading the protected `_config`). It holds the concrete
  backend (`Bus_<variant>`) behind a `unique_ptr` and provides the parts every
  kind shares regardless of intent management:

  - `init<CfgT>` — heap-allocate + `init` the variant backend selected by the
    config type, adopt it, and cache the pin config + backend query metadata.
  - `release` — release and drop the backend, refreshing the query mirror.
  - the lock-free backend-query mirror (`backendKind` / `controllerId` /
    `maxFrequency` / `backendGeneration`) read from facade-owned atomics so the
    query path never dereferences the live `_backend` during a swap.

  This base only needs the Traits subset `IBus` / `IBusConfig` / `BackendFor`;
  the master-only Traits members (`MasterAccessConfig`, `TransferDesc`,
  `LogicalBusConfig`, `applyAdopt`, ...) are required only by
  `ManagedBusFacade`. The protected `_backend`, `_generation`, `backend()`, and
  `_cacheBackendMeta()` are the seam `ManagedBusFacade` reaches when it swaps
  the backend under the bus lock; query atomics stay private (only this base's
  mirror and query readers touch them).

  A kind with no data-path beyond the base (none today) could derive `Bus`
  directly; uart / i2s derive it and add only their kind data-path overrides,
  while i2c / spi go through `ManagedBusFacade` for the phase-3 surface.
 */
template <class Traits>
struct FacadeCore : public Traits::IBus {
    using IBus       = typename Traits::IBus;
    using IBusConfig = typename Traits::IBusConfig;

    FacadeCore(void) = default;
    ~FacadeCore(void) override
    {
        // The backend's own dtor runs its release(); resetting here keeps
        // the single-release path (do not also call release() to avoid a
        // double release on backends that are not idempotent).
        _backend.reset();
    }

    FacadeCore(const FacadeCore&)            = delete;
    FacadeCore& operator=(const FacadeCore&) = delete;

    /*!
      @brief Create the backend for `cfg`'s variant and initialize it.

      `CfgT` must have a `Traits::BackendFor` specialization (every offered
      variant provides one). Heap-allocates the backend; allocation failure is
      `OUT_OF_RESOURCE`. On success the facade adopts the backend and caches
      the pin config for `getConfig`/`probe`.

      Lifecycle note (D1/F8): `init`/`release` are NOT synchronized with access
      windows. The supported paths never race: a typed `acquire<CfgT>` interns a
      fresh facade and first-inits it (no accessor exists yet), the managed
      hot-swap path replaces the backend under the bus lock (`swapBackendWith`),
      and teardown runs through the dtor (no live owners). Direct re-`init` of an
      already-initialized facade, or `release` while a task owns an accessor and
      is mid-transfer, would race the backend lifetime, so treat direct
      re-init/release as a startup/shutdown-only operation: do not call it while
      any accessor on this bus is in an access window. (A locked guard would need
      a kind-aware ownership probe this base does not carry; deferred until a
      direct re-init/release path is actually exposed.)
     */
    template <class CfgT>
    result_t<void> init(const CfgT& cfg)
    {
        static_assert(std::is_base_of<IBusConfig, CfgT>::value,
                      "Bus::init expects a BusConfig of this bus kind (a BusConfig_<variant>)");
        using BackendT = typename Traits::template BackendFor<CfgT>::type;
        std::unique_ptr<IBus> backend{new (std::nothrow) BackendT()};
        if (!backend) {
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
        }
        auto r = static_cast<BackendT*>(backend.get())->init(cfg);
        if (!r.has_value()) {
            return r;
        }
        _backend      = std::move(backend);
        this->_config = cfg;  // slice to pin/kind for getConfig()/probe(); backend keeps variant fields
        _cacheBackendMeta();
        return {};
    }

    /*!
      @brief Release and drop the backend (startup/shutdown only).

      See the `init` lifecycle note (D1/F8): not synchronized with access
      windows. Call only when no accessor on this bus is in an access window.
     */
    result_t<void> release(void) override
    {
        if (!_backend) {
            return {};
        }
        auto r = _backend->release();
        if (!r.has_value() && r.error() != m5::hal::v2::error::error_t::NOT_IMPLEMENTED) {
            return r;
        }
        _backend.reset();
        _cacheBackendMeta();
        return {};
    }

    // Backend query API (ADR 034). To stay safe when another task commits and
    // hot-swaps the backend, the metadata is mirrored into facade-owned atomics
    // (refreshed inside the swap, under the bus lock) so the query path never
    // dereferences the live `_backend`. Reads are lock-free; the per-field
    // atomics may briefly disagree across a swap, with `backendGeneration()`
    // (bumped last) as the canonical "it changed" signal.
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

protected:
    /*! @brief Live backend for kind-specific forwards (e.g. SPI begin/endTransaction). */
    IBus* backend(void)
    {
        return _backend.get();
    }

    /*!
      @brief Forward a kind-specific operation to the live backend.

      The runtime facade owns the public lock / accessor binding and delegates
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
            return m5::stl::make_unexpected(m5::hal::v2::error::error_t::NOT_IMPLEMENTED);
        }
        return fn(*b);
    }

    // Mirror the live backend's query metadata into the facade atomics. Called
    // wherever `_backend` changes (init / release / adopt / swap), under the
    // bus lock for the swap path. With no backend, the safe base defaults apply.
    void _cacheBackendMeta(void)
    {
        if (_backend) {
            _q_kind.store(_backend->backendKind(), std::memory_order_release);
            _q_controller.store(_backend->controllerId(), std::memory_order_release);
            _q_maxfreq.store(_backend->maxFrequency(), std::memory_order_release);
        } else {
            _q_kind.store(bus::IBus::backendKind(), std::memory_order_release);
            _q_controller.store(bus::IBus::controllerId(), std::memory_order_release);
            _q_maxfreq.store(bus::IBus::maxFrequency(), std::memory_order_release);
        }
    }

    std::unique_ptr<IBus> _backend;        // the swappable backend (Phase 3 hot-swap seam)
    std::atomic<uint32_t> _generation{0};  // bumped on every backend swap (poll baseline)

private:
    // Query metadata mirror (read lock-free by the query API; written on swap).
    std::atomic<types::backend_kind_t> _q_kind{types::backend_kind_t::Software};
    std::atomic<int8_t> _q_controller{-1};
    std::atomic<uint32_t> _q_maxfreq{0};
};

/*!
  @brief Master runtime facade with the phase-3 intent + hot-swap surface.

  Extends `FacadeCore<Traits>` (the shared backend spine) with `bus::IManagedBus`
  (the intent + hot-swap surface the resolver touches). `bus::IManagedBus` does
  NOT inherit `bus::IBus`, so there is no duplicate-`IBus` diamond: the facade
  has exactly one `bus::IBus` subobject, owned by `FacadeCore`.

  The concrete backend lives behind `FacadeCore`'s `unique_ptr`; `transfer` is
  forwarded to it while every lock/accessor-binding stays on THIS object. That
  is the hot-swap seam: a later commit can replace the backend under the facade
  lock without rebinding accessors (ADR 034). The backend the swap touches is
  the inherited `FacadeCore` member, reached through `this->_backend` /
  `this->_cacheBackendMeta()` / `this->_generation` (dependent-base names need
  the `this->` qualification under two-phase lookup).

  A kind derives an empty `Bus` from this; SPI adds only its CS-transaction
  forwards (it reaches the backend through `FacadeCore`'s protected `backend()`).
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

    result_t<void> transfer(bus::IAccessor* owner, const MasterAccessConfig& cfg, const TransferDesc& desc,
                            data::Source* src, size_t tx_len, data::Sink* dst, size_t rx_len) override
    {
        return this->forwardBackend([&](IBus& b) { return b.transfer(owner, cfg, desc, src, tx_len, dst, rx_len); });
    }

    /*!
      @brief Adopt a ready (already-`init`-ed) backend for a logical acquire.

      Backs the phase-3 logical path (`BusView::acquire(LogicalBusConfig)`):
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
        this->_backend = std::move(backend);
        Traits::applyAdopt(this->_config, logical);
        _intent  = logical.intent;
        _managed = true;
        this->_cacheBackendMeta();
        return {};
    }

    /*! @brief The acquire intent recorded for this bus (commit-time resolver). */
    const types::AllocationIntent& intent(void) const override
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
      to `requireHardware()` (or demoting to `software()`) is a setIntent
      followed by `commitBuses()`. Re-tagging through the logical path also
      marks the bus intent-managed.
     */
    void setIntent(const LogicalBusConfig& logical)
    {
        _intent  = logical.intent;
        _managed = true;
    }

    /*!
      @brief Rebuild the logical request from the recorded wiring + intent.

      The commit-time resolver hands this to a backend factory to re-make the
      backend (software or hardware) for this bus during a swap.
     */
    LogicalBusConfig logicalConfig(void) const
    {
        LogicalBusConfig logical;
        Traits::fillLogical(logical, this->_config);
        logical.intent = _intent;
        return logical;
    }

    /*!
      @brief Build (under the lock) and adopt a backend via a factory (ADR 034 hot-swap).

      Takes the facade's lock -- the same one accessors contend on -- through a
      stack sentinel, so a swap waits for any in-flight transfer to finish and,
      with a finite `timeout_ms`, returns `TIMEOUT_ERROR` instead of wedging
      when the bus stays busy. `make()` runs INSIDE the lock, so the new
      backend's `init()` (which drives pins / installs a driver) cannot race an
      in-flight transfer on the old backend (D1/F1: the hardware-visible part of
      the swap is now guarded by the same ownership the swap claims). The old
      backend is `release`-d before the new one is adopted; a release failure
      ABORTS the swap with the old backend kept and the error propagated (D1/F7)
      rather than silently dropped, so the controller pool never reclaims an
      unreleased resource. The query metadata is refreshed and the generation
      counter is bumped LAST so a poller that sees the new generation reads
      settled metadata.

      `make()` returns a raw `bus::IBus*`; it is always this kind's `IBus` (this
      kind's factory made it), so it is downcast to the typed backend the facade
      owns. A null result is `OUT_OF_RESOURCE` unless `allow_null` (the
      software-less "pending" detach path).
     */
    template <class MakeBackend>
    result_t<void> swapBackendWith(uint32_t timeout_ms, bool allow_null, MakeBackend&& make)
    {
        MasterAccessConfig sentinel_cfg;
        MasterAccessor sentinel{*this, sentinel_cfg};
        return bus::guarded(
            [&] { return sentinel.beginAccess(timeout_ms); },
            [&]() -> result_t<void> {
                bus::IBus* raw = make();  // init() runs here, under the lock (F1)
                if (raw == nullptr && !allow_null) {
                    return m5::stl::make_unexpected(m5::hal::v2::error::error_t::OUT_OF_RESOURCE);
                }
                std::unique_ptr<IBus> typed{static_cast<IBus*>(raw)};
                if (this->_backend) {
                    auto rel = this->_backend->release();
                    // Propagate a GENUINE release failure (F7): abort the swap
                    // with the old backend kept, so the controller pool never
                    // reclaims an unreleased resource. NOT_IMPLEMENTED is the
                    // base default ("nothing to free" -- a backend that does not
                    // override release), which is not a failure, so the swap
                    // proceeds.
                    if (!rel.has_value() && rel.error() != m5::hal::v2::error::error_t::NOT_IMPLEMENTED) {
                        return rel;
                    }
                }
                this->_backend = std::move(typed);
                this->_cacheBackendMeta();
                this->_generation.fetch_add(1, std::memory_order_release);
                return {};
            },
            [&] { return sentinel.endAccess(); });
    }

    /*!
      @brief Hot-swap to a ready (already-`init`-ed) backend under the bus lock.

      The strict non-null path. The commit-time resolver instead uses
      `IAllocationKind::commitHardware` so `init()` happens under the lock; this
      overload remains for callers that already hold an initialized backend
      (e.g. tests). Release errors propagate as in `swapBackendWith`.
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
    // Phase-3 acquire intent, recorded for the commit-time resolver (ADR 034).
    types::AllocationIntent _intent{};  // capability-based allocation request
    bool _managed = false;              // true => commitBuses() may reassign (logical acquire only)
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_MANAGED_FACADE_HPP_
