// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_HAL_BACKEND_HPP_
#define M5_HAL_BUS_HAL_BACKEND_HPP_

#include "./bus.hpp"       // IBus, IBusConfig, types, error, result_t
#include "./registry.hpp"  // IdentityKey, BusRegistry

#include <cstdint>
#include <memory>

/*!
  @namespace m5::hal::v2::bus
  @brief IHalBackend: the allocation seam a BusView delegates to.
 */
namespace m5::hal::v2::bus {

//-------------------------------------------------------------------------
/*!
  @brief Type-erased logical (intent-driven) acquire request.

  The logical acquire path carries an `AllocationIntent` plus the wiring, which
  the typed `IBusConfig&` path cannot express (a kind's `LogicalBusConfig` is
  NOT derived from `IBusConfig`). `AllocationRequest` decouples the backend
  interface from each kind's `LogicalBusConfig` type: it surfaces the
  kind-neutral fields (`kind`, `identity`, `intent`) directly, and keeps a
  non-owning pointer to the original kind-specific request so a `LocalBackend`
  can hand the whole `LogicalBusConfig` to that kind's software/hardware factory
  without a downcast at this layer.

  The `config` pointer is borrowed: it must outlive the synchronous
  `acquireBusLogical()` call. A BusView builds an `AllocationRequest` on the
  stack from a caller-supplied `LogicalBusConfig&`, mirroring how the current
  `acquire(const LogicalBusConfig&)` forwards that reference down to the factory
  within the same call.
 */
struct AllocationRequest {
    types::bus_kind_t kind;          ///< Bus kind the request targets.
    IdentityKey identity;            ///< Pins-only identity (intern key).
    types::AllocationIntent intent;  ///< Capability-based allocation request.
    const void* config = nullptr;    ///< Borrowed kind-specific `LogicalBusConfig`.

    AllocationRequest(types::bus_kind_t k, const IdentityKey& id, const types::AllocationIntent& in, const void* cfg)
        : kind{k}, identity{id}, intent{in}, config{cfg}
    {
    }
};

//-------------------------------------------------------------------------
/*!
  @brief Allocation seam shared by every BusView.

  A BusView holds an `IHalBackend*` and forwards acquisition to it. The backend
  owns the interning registry, per-kind allocation cores, and the factory
  wiring. `LocalBackend` resolves locally; a future `RemoteBackend` proxies the
  same surface to a peer over a transport.

  The typed acquire path (`BusView::acquire<CfgT>`) uses `busRegistry()`
  directly because it needs the concrete config type (for `BackendFor<CfgT>`
  variant selection) which cannot cross a virtual boundary. This path is
  inherently local; a remote backend provides a separate mechanism.

  The logical path (`acquireBusLogical`) and commit are virtual because they
  operate on type-erased requests, which both local and remote can implement.
 */
struct IHalBackend {
    /*!
      @brief Access the interning registry for the typed acquire path.

      BusView's `acquire<CfgT>` calls this to get the registry, then calls
      `acquireOrFind` with a template make-lambda that creates the kind-specific
      facade and initializes the variant backend. This keeps the concrete config
      type in the BusView template, avoiding type erasure at the interface.

      Both LocalBackend and a future RemoteBackend provide a registry: local for
      real buses, remote for proxy interning. The difference is in what the
      BusView's make-lambda creates (local facade vs remote proxy), which is a
      concern of the make-lambda's construction site and does not affect this
      interface.
     */
    BusRegistry& busRegistry(void)
    {
        return _registry;
    }
    const BusRegistry& busRegistry(void) const
    {
        return _registry;
    }

    /*!
      @brief Typed acquire path: create or intern a bus from a kind-specific config.
      @param kind Bus kind.
      @param id Pins-only identity (intern key).
      @param cfg Kind-specific bus config (concrete type known by the caller).
      @return The interned bus, or NOT_IMPLEMENTED if the backend does not handle
              typed acquires (BusView then falls back to the local busRegistry path).

      RemoteBackend overrides this to create proxy buses on the remote peer.
      LocalBackend uses the default (NOT_IMPLEMENTED), letting BusView's template
      lambda select the variant backend via BackendFor<CfgT>.
     */
    virtual result_t<std::shared_ptr<IBus>> acquireBusTyped(types::bus_kind_t kind, const IdentityKey& id,
                                                            const IBusConfig& cfg)
    {
        (void)kind;
        (void)id;
        (void)cfg;
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }

    /*!
      @brief Logical (intent-driven) acquire used by the master-kind path.
      @param kind Bus kind the request belongs to.
      @param id Pins-only identity (intern key).
      @param req Type-erased intent + borrowed kind-specific logical config.
     */
    virtual result_t<std::shared_ptr<IBus>> acquireBusLogical(types::bus_kind_t kind, const IdentityKey& id,
                                                              const AllocationRequest& req) = 0;

    /*!
      @brief Complete/validate a logical request before it becomes an
             identity (pin auto-fill on a single-element pin domain, early
             `INVALID_ARGUMENT` on a wiring mismatch).
      @param kind Bus kind the request belongs to.
      @param logical_config Borrowed, type-erased kind-specific
             `LogicalBusConfig&`, mutated in place.

      Runs BEFORE identity derivation in the logical acquire path, so a kind
      whose low-power (or otherwise pin-restricted) controller has a fixed
      wiring can fill it in before the pins become the bus's identity.
      Default: no-op success. `LocalBackend` forwards to the registered
      kind's `ILocalKindAdapter::completeLogical`; `RemoteBackend` does not
      override this (remote acquires require explicit pins).
     */
    virtual result_t<void> completeLogicalRequest(types::bus_kind_t kind, void* logical_config)
    {
        (void)kind;
        (void)logical_config;
        return {};
    }

    /*!
      @brief Resolve every live bus of one kind from its pending intent.
      @param kind Bus kind to commit (no-op for kinds without an allocator).
      @param timeout_ms Commit deadline in milliseconds.
     */
    virtual result_t<void> commitBuses(types::bus_kind_t kind, uint32_t timeout_ms) = 0;

    /*!
      @brief Release a bus from the registry.

      The default implementation clears the registry slot for (kind, id),
      reclaiming the capacity immediately. RemoteBackend overrides this to
      also send a BusRelease bytecode to the peer before clearing the slot.

      Returns `INVALID_ARGUMENT` if the exact live instance is not registered,
      and `BUSY` while another owner/accessor exists or the identity is already
      releasing. Success consumes the caller's reference at the BusView layer.
     */
    virtual result_t<void> releaseBus(types::bus_kind_t kind, const IdentityKey& id,
                                      const std::shared_ptr<IBus>& expected)
    {
        auto ticket = _registry.beginRelease(kind, id, expected);
        if (!ticket.has_value()) {
            return m5::stl::make_unexpected(ticket.error());
        }
        auto committed = _registry.commitRelease(ticket.value());
        if (!committed.has_value()) {
            (void)_registry.cancelRelease(ticket.value());
            return m5::stl::make_unexpected(committed.error());
        }
        return {};
    }

    /*!
      @brief Hardware controllers currently leased for a kind (observation/tests).
      @param kind Bus kind to query.
     */
    virtual uint8_t hardwareInUse(types::bus_kind_t kind) const
    {
        (void)kind;
        return 0;
    }

    /*!
      @brief Claim a hardware controller for a caller OUTSIDE the intent
             resolver (e.g. a standalone slave with no `IManagedBus`).
      @param kind Bus kind the controller belongs to.
      @param intent Resolution request; a default-constructed intent is Auto.
      @return The claimed controller index, or an error (see
              `AllocationCore::claimController`).

      Default: `NOT_IMPLEMENTED` (a kind with no `AllocationCore`, or a
      backend with no local allocation at all -- e.g. `RemoteBackend`, where
      claiming a controller on a remote peer for local slave use is
      meaningless). `LocalBackend` forwards to the kind's `AllocationCore`.
     */
    virtual result_t<int8_t> claimController(types::bus_kind_t kind, const types::AllocationIntent& intent)
    {
        (void)kind;
        (void)intent;
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }

    /*!
      @brief Return a controller claimed via `claimController`.
      @param kind Bus kind the controller belongs to.
      @param controller Controller index returned by `claimController`.
     */
    virtual result_t<void> releaseClaimedController(types::bus_kind_t kind, int8_t controller)
    {
        (void)kind;
        (void)controller;
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }

    virtual ~IHalBackend() = default;

protected:
    BusRegistry _registry;
};

}  // namespace m5::hal::v2::bus

#endif
