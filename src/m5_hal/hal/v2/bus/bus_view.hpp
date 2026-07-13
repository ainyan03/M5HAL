// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_BUS_VIEW_HPP_
#define M5_HAL_BUS_BUS_VIEW_HPP_

#include "./bus.hpp"
#include "./hal_backend.hpp"
#include "./registry.hpp"

#include <memory>
#include <new>
#include <type_traits>

/*!
  @namespace m5::hal::v2::bus
  @brief BusViewCore: the kind-neutral spine shared by every kind's BusView.
 */
namespace m5::hal::v2::bus {

/*!
  @brief Kind-neutral BusView spine, parameterized by a per-kind Traits type.

  Every kind's `BusView` (i2c::BusView, spi::BusView, uart::BusView,
  i2s::BusView) delegates registry and allocation to the HAL backend through
  the identical typed-acquire / logical-acquire / commit / release surface;
  the only per-kind variation is whether `hardwareInUse()` forwards to the
  backend (I2C / SPI) or is always zero (UART / I2S), plus the kind's
  `createBusConfig()` pin overloads. This template holds the shared part;
  each kind derives a thin `BusView` that adds only `createBusConfig()`.

  `Traits` supplies: `IBus`, `IBusConfig`, `LogicalBusConfig`, `BusType`,
  `KIND`, `MANAGED_ALLOCATION`, and the identity / compatibility helpers
  (`identityFromConfig`, `identityFromLogical`, `configCompatible`).
 */
template <class Traits>
class BusViewCore {
public:
    using IBus                              = typename Traits::IBus;
    using IBusConfig                        = typename Traits::IBusConfig;
    using LogicalBusConfig                  = typename Traits::LogicalBusConfig;
    using BusType                           = typename Traits::BusType;
    static constexpr types::bus_kind_t KIND = Traits::KIND;

    BusViewCore() : _backend{nullptr}
    {
    }
    explicit BusViewCore(IHalBackend* backend) : _backend{backend}
    {
    }
    BusViewCore(const BusViewCore&)            = delete;
    BusViewCore& operator=(const BusViewCore&) = delete;

    void setBackend(IHalBackend* backend)
    {
        _backend = backend;
    }

    template <class CfgT>
    result_t<std::shared_ptr<IBus>> acquire(const CfgT& cfg)
    {
        static_assert(std::is_base_of<IBusConfig, CfgT>::value,
                      "BusView::acquire expects a BusConfig of this bus kind");
        if (_backend == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        IdentityKey id = Traits::identityFromConfig(cfg);
        if (!id.valid()) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        {
            auto r = _backend->acquireBusTyped(KIND, id, cfg);
            if (r.has_value()) {
                return std::static_pointer_cast<IBus>(r.value());
            }
            if (r.error() != error::error_t::NOT_IMPLEMENTED) {
                return m5::stl::make_unexpected(r.error());
            }
        }
        // BusRegistry::acquireOrFind is fixed to the kind-neutral base
        // m5::hal::v2::bus::IBus (it interns every kind through one slot
        // table); this class's own `IBus` alias names the kind-specific
        // Traits::IBus instead, so the make-lambda spells the base type out
        // fully to avoid the two colliding.
        auto acquired = _backend->busRegistry().acquireOrFind(
            KIND, id,
            [&cfg](const std::shared_ptr<m5::hal::v2::bus::IBus>& existing) -> result_t<void> {
                if (!Traits::configCompatible(static_cast<const IBusConfig&>(existing->getConfig()), cfg)) {
                    return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
                }
                return {};
            },
            [&cfg]() -> result_t<std::shared_ptr<m5::hal::v2::bus::IBus>> {
                std::shared_ptr<BusType> facade{new (std::nothrow) BusType()};
                if (!facade) {
                    return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
                }
                auto r = facade->init(cfg);
                if (!r.has_value()) {
                    return m5::stl::make_unexpected(r.error());
                }
                return std::shared_ptr<m5::hal::v2::bus::IBus>{facade};
            });
        if (!acquired.has_value()) {
            return m5::stl::make_unexpected(acquired.error());
        }
        return std::static_pointer_cast<IBus>(acquired.value());
    }

    result_t<std::shared_ptr<IBus>> acquire(const LogicalBusConfig& req)
    {
        if (_backend == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        // Let the backend complete/validate the request (e.g. a low-power
        // controller's fixed-pin auto-fill) BEFORE identity derivation, so
        // the completed pins -- not the caller's possibly omitted ones --
        // become the bus's identity.
        LogicalBusConfig eff = req;
        auto rc              = _backend->completeLogicalRequest(KIND, &eff);
        if (!rc.has_value()) {
            return m5::stl::make_unexpected(rc.error());
        }
        IdentityKey id = Traits::identityFromLogical(eff);
        if (!id.valid()) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        if (!eff.intent.valid()) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        AllocationRequest ar{KIND, id, eff.intent, &eff};
        auto acquired = _backend->acquireBusLogical(KIND, id, ar);
        if (!acquired.has_value()) {
            return m5::stl::make_unexpected(acquired.error());
        }
        return std::static_pointer_cast<IBus>(acquired.value());
    }

    result_t<void> commitBuses(uint32_t timeout_ms = types::TIMEOUT_FOREVER)
    {
        if (_backend == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        return _backend->commitBuses(KIND, timeout_ms);
    }

    /*!
      @brief Explicitly release a bus acquired via acquire().

      This is a consuming operation: the caller must be the sole remaining
      strong owner (destroy accessors and aliases first). On success it clears
      the passed handle and immediately reclaims the registry slot. For a
      remote backend it also sends BusRelease to the peer. On failure the
      handle remains unchanged. Pass the `shared_ptr<IBus>` returned by acquire().
      Returns `INVALID_ARGUMENT` if the bus is null, its config pins
      are invalid, or no matching exact instance is registered; returns `BUSY`
      if another strong owner exists or release is already in progress.
     */
    result_t<void> release(std::shared_ptr<IBus>& bus)
    {
        if (_backend == nullptr || !bus) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        IdentityKey id = Traits::identityFromConfig(bus->getConfig());
        if (!id.valid()) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        // Move the caller's ownership into the type-erased backend argument.
        // This avoids creating an extra strong owner that would make the
        // registry's sole-owner check indistinguishable from a real alias.
        std::shared_ptr<m5::hal::v2::bus::IBus> expected{std::move(bus)};
        auto released = _backend->releaseBus(KIND, id, expected);
        if (!released.has_value()) {
            bus = std::static_pointer_cast<IBus>(expected);
            return m5::stl::make_unexpected(released.error());
        }
        expected.reset();
        return {};
    }

    /*!
      @brief Claim a hardware controller of this kind OUTSIDE the intent
             resolver (e.g. for a standalone slave that has no `IManagedBus`
             of its own and never goes through `acquire()`/`commitBuses()`).

      Omitting `intent` (the default) claims Auto (lowest-numbered eligible
      free controller); see `AllocationCore::claimController` for the full
      resolution contract. NOT_IMPLEMENTED on a backend / kind with no local
      allocation core (e.g. RemoteBackend, or a kind such as UART/I2S that
      never manages controllers this way).
     */
    result_t<int8_t> claimController(const types::AllocationIntent& intent = {})
    {
        if (_backend == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        return _backend->claimController(KIND, intent);
    }

    /*! @brief Return a controller claimed via `claimController`. */
    result_t<void> releaseClaimedController(int8_t controller)
    {
        if (_backend == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        return _backend->releaseClaimedController(KIND, controller);
    }

    uint8_t hardwareInUse(void) const
    {
        if constexpr (Traits::MANAGED_ALLOCATION) {
            if (_backend == nullptr) {
                return 0;
            }
            return _backend->hardwareInUse(KIND);
        } else {
            return 0;
        }
    }

protected:
    /// Non-virtual: a BusView is never deleted through a `BusViewCore*`.
    /// `protected` makes that a compile error instead of silent UB
    /// (same policy as `IGPIO` / `IPort`).
    ~BusViewCore() = default;

private:
    IHalBackend* _backend;
};

}  // namespace m5::hal::v2::bus

#endif
