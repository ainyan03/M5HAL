// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_BUS_VIEW_HPP_
#define M5_HAL_BUS_BUS_VIEW_HPP_

#include "./bus.hpp"
#include "./hal_backend.hpp"
#include "./registry.hpp"

#include <memory>
#include <type_traits>
#include <utility>

/*!
  @namespace m5::hal::v2::bus
  @brief BusViewCore: the kind-neutral spine shared by every kind's BusView.
 */
namespace m5::hal::v2::bus {

/*!
  @brief Kind-neutral BusView spine, parameterized by a per-kind Traits type.

  Every kind's `BusView` (i2c::BusView, spi::BusView, uart::BusView,
  i2s::BusView, pdm::BusView) delegates registry and allocation to the HAL backend through
  the identical portable-acquire / logical-acquire / commit / close surface;
  the only per-kind variation is whether `hardwareInUse()` forwards to the
  backend (I2C / SPI) or is always zero (UART / I2S / PDM), plus the kind's
  `createBusConfig()` pin overloads. This template holds the shared part;
  each kind derives a thin `BusView` that adds only `createBusConfig()`. The
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

    /*!
      @brief Acquire from the selected provider using only portable fields.

      The public `BusConfig` denotes the variant-independent `IBusConfig`;
      provider selection is independent of the config's concrete C++ type.
     */
    [[nodiscard]] result_t<std::shared_ptr<IBus>> acquire(const IBusConfig& cfg)
    {
        if (_backend == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        const ResourceKey id = Traits::identityFromConfig(cfg);
        auto acquired        = _backend->acquireBusPortable(KIND, id, cfg);
        if (!acquired.has_value()) {
            return m5::stl::make_unexpected(acquired.error());
        }
        return std::static_pointer_cast<IBus>(acquired.value());
    }

    /*!
      @brief Acquire through an explicit native ownership policy.

      The method name and portable config remain identical to the ordinary
      path. The selected provider accepts only its strongly typed borrowed or
      managed policies; remote providers reject this call before wire I/O.
     */
    template <class Policy>
    [[nodiscard]] result_t<std::shared_ptr<IBus>> acquire(const IBusConfig& cfg, Policy&& policy)
    {
        using PolicyT = typename std::decay<Policy>::type;
        static_assert(native::is_borrowed<PolicyT>::value || native::is_managed<PolicyT>::value,
                      "the second acquire argument must be native::borrowed(...) or native::managed(...)");
        if (_backend == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        return Traits::template NativeProvider<PolicyT>::acquire(*_backend, cfg, std::forward<Policy>(policy));
    }

    [[nodiscard]] result_t<std::shared_ptr<IBus>> acquire(const LogicalBusConfig& req)
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
        ResourceKey id = Traits::identityFromLogical(eff);
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
      @brief Close a bus acquired via acquire().

      This is a consuming operation: the caller must be the sole remaining
      strong owner (destroy accessors and aliases first). On success it clears
      the passed handle and immediately reclaims the registry slot. For a
      remote backend it also sends BusRelease to the peer. On failure the
      handle remains unchanged. Pass the `shared_ptr<IBus>` returned by acquire().
      Returns `NOT_CONNECTED` if this view has no backend,
      `INVALID_ARGUMENT` if the bus is null or no matching exact instance is
      registered, and `BUSY`
      if another strong owner exists or close is already in progress.
     */
    [[nodiscard]] result_t<void> close(std::shared_ptr<IBus>& bus)
    {
        if (_backend == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        if (!bus) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        const ResourceKey* registered = bus->registryResourceKey();
        ResourceKey id = registered != nullptr ? *registered : Traits::identityFromConfig(bus->getConfig());
        // Move the caller's ownership into the type-erased backend argument.
        // This avoids creating an extra strong owner that would make the
        // registry's sole-owner check indistinguishable from a real alias.
        std::shared_ptr<m5::hal::v2::bus::IBus> expected{std::move(bus)};
        auto closed = _backend->closeBus(KIND, id, expected);
        if (!closed.has_value()) {
            bus = std::static_pointer_cast<IBus>(expected);
            return m5::stl::make_unexpected(closed.error());
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
      allocation core (e.g. RemoteBackend, or a kind such as UART/I2S/PDM that
      never manages controllers this way). Call only outside every active
      Access or protocol-frame claim for this bus kind; a concurrent commit may
      hold the allocation lock while waiting for such a window to close.
     */
    result_t<int8_t> claimController(const types::AllocationIntent& intent = {})
    {
        if (_backend == nullptr) {
            return m5::stl::make_unexpected(error::error_t::NOT_CONNECTED);
        }
        return _backend->claimController(KIND, intent);
    }

    /*! @brief Return a controller claimed via `claimController`; same no-access-window rule. */
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
