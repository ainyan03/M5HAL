// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_HAL_BACKEND_HPP_
#define M5_HAL_BUS_HAL_BACKEND_HPP_

#include "./bus.hpp"       // IBus, IBusConfig, types, error, result_t
#include "./registry.hpp"  // ResourceKey, BusRegistry

#include <cstdint>
#include <memory>

/*!
  @namespace m5::hal::v2::bus
  @brief IHalBackend: the allocation seam a BusView delegates to.
 */
namespace m5::hal::v2::bus {

class LocalBackend;

}  // namespace m5::hal::v2::bus

namespace m5::hal::v2 {
class ResourceDomain;
}

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
    ResourceKey identity;            ///< Exact identity projection (portable acquire currently uses Pins).
    types::AllocationIntent intent;  ///< Capability-based allocation request.
    const void* config = nullptr;    ///< Borrowed kind-specific `LogicalBusConfig`.

    AllocationRequest(types::bus_kind_t k, const ResourceKey& id, const types::AllocationIntent& in, const void* cfg)
        : kind{k}, identity{id}, intent{in}, config{cfg}
    {
    }
};

//-------------------------------------------------------------------------
/*!
  @brief Allocation seam shared by every BusView.

  A BusView holds an `IHalBackend*` and forwards acquisition to it. The backend
  selects an interning registry and factory wiring. `LocalBackend` uses its
  ResourceDomain registry; `RemoteBackend` owns a connection-local proxy
  registry and forwards the same surface to a peer.

  The portable acquire path crosses this interface as the kind-level
  `IBusConfig`. LocalBackend selects the registered provider for that kind;
  RemoteBackend may translate the same request into a proxy acquire.

  The logical path (`acquireBusLogical`) and commit are virtual because they
  operate on type-erased requests, which both local and remote can implement.
 */
struct IHalBackend {
    /*!
      @brief Access the backend's interning registry.

      Both LocalBackend and RemoteBackend provide a registry: domain-local for
      real buses and connection-local for remote proxies.
     */
    virtual BusRegistry& busRegistry(void)             = 0;
    virtual const BusRegistry& busRegistry(void) const = 0;

    virtual LocalResourceContext localResources(void) const
    {
        return {};
    }

    /*! @brief Local acquisition namespace, or nullptr for remote backends. */
    virtual const ResourceDomain* localResourceDomain(void) const
    {
        return nullptr;
    }

    /*!
      @brief Portable acquire path shared by local and remote backends.
      @param kind Bus kind.
      @param id Exact target identity projected from `cfg` by the kind Traits.
      @param cfg Variant-independent kind-level configuration.

      Backends override this entry and select a registered provider without
      consulting the dynamic config type.
     */
    virtual result_t<std::shared_ptr<IBus>> acquireBusPortable(types::bus_kind_t kind, const ResourceKey& id,
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
      @param id Exact target identity (portable acquire supplies a Pins projection).
      @param req Type-erased intent + borrowed kind-specific logical config.
     */
    virtual result_t<std::shared_ptr<IBus>> acquireBusLogical(types::bus_kind_t kind, const ResourceKey& id,
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
      @brief Close a bus and retire it from the registry.

      The default implementation first reserves the identity in Closing,
      closes the exact bus while that reservation is held, and only then
      clears the registry slot. RemoteBackend overrides this to send a
      BusRelease bytecode to the peer under the same ordering rule.

      Returns `INVALID_ARGUMENT` if the exact live instance is not registered,
      and `BUSY` while another owner/accessor exists or the identity is already
      releasing. Success consumes the caller's reference at the BusView layer.
     */
    virtual result_t<void> closeBus(types::bus_kind_t kind, const ResourceKey& id,
                                    const std::shared_ptr<IBus>& expected)
    {
        if (id.kind != kind) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        auto& registry = busRegistry();
        auto ticket    = registry.beginRelease(id, expected);
        if (!ticket.has_value()) {
            return m5::stl::make_unexpected(ticket.error());
        }
        const auto outcome = expected->closeWithOutcome();
        if (outcome.disposition == CloseDisposition::NoMutation) {
            auto cancelled = registry.cancelRelease(ticket.value());
            if (!cancelled.has_value()) {
                return m5::stl::make_unexpected(cancelled.error());
            }
            return m5::stl::make_unexpected(outcome.error_code);
        }
        if (outcome.disposition == CloseDisposition::PartialOrUnknown) {
            auto quarantined = registry.quarantineRelease(ticket.value());
            if (!quarantined.has_value()) {
                return m5::stl::make_unexpected(quarantined.error());
            }
            return m5::stl::make_unexpected(outcome.error_code);
        }
        auto committed = registry.commitRelease(ticket.value());
        if (!committed.has_value()) {
            // A ticket mismatch means we can no longer prove that restoring
            // Live is safe. Keep Closing rather than exposing a possibly
            // released backend under the old identity.
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
};

}  // namespace m5::hal::v2::bus

#endif
