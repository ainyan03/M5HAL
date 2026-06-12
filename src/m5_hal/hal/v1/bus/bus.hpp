// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_HPP_
#define M5_HAL_BUS_HPP_

#include "../error.hpp"
#include "../types.hpp"

#include <M5Utility.hpp>

/*!
  @namespace m5::hal::v1::bus
  @brief Bus / Accessor abstractions shared by every kind (I2C, SPI, ...).
 */
namespace m5::hal::v1::bus {

struct Bus;
struct AccessConfig;
struct Accessor;

//-------------------------------------------------------------------------
/*!
  @brief Abstract base for per-bus initialization configuration.

  Carries a kind tag (`bus_kind`) as a plain field, set by the derived
  constructor and forwarded to this base constructor. `getBusKind()` is
  non-virtual so the type has no vtable. The field is intentionally
  non-const because variant implementations copy-assign through a
  `_config = static_cast<const FooBusConfig&>(config);` pattern; the
  value is fixed at construction by the derived ctor (the sole entry
  point) by convention.
 */
struct BusConfig {
    types::bus_kind_t bus_kind;  ///< Kind tag (`I2C`, `SPI`, ...). Set by the derived ctor.
    types::bus_kind_t getBusKind(void) const
    {
        return bus_kind;
    }

protected:
    explicit constexpr BusConfig(types::bus_kind_t k) : bus_kind{k}
    {
    }
};

//-------------------------------------------------------------------------
/*!
  @brief Marker base for per-call transfer metadata.

  Each bus kind defines its own derivation (`i2c::TransferDesc`,
  `spi::TransferDesc`, etc.) carrying prefix bytes and per-transfer
  flags. This base has no virtual hook and no `getBusKind` — it exists
  only as a typed anchor.
 */
struct TransferDesc {};

//-------------------------------------------------------------------------
/*!
  @brief Abstract base for accessor-side (per-target) configuration.

  The kind-tag convention matches `BusConfig`: a non-virtual getter
  returns a field that the derived ctor forwards to this base ctor.
 */
struct AccessConfig {
    types::bus_kind_t bus_kind;
    types::bus_kind_t getBusKind(void) const
    {
        return bus_kind;
    }

protected:
    explicit constexpr AccessConfig(types::bus_kind_t k) : bus_kind{k}
    {
    }
};

//-------------------------------------------------------------------------
/*!
  @brief Abstract base for a per-target accessor.

  An `Accessor` is the owner identity for an access window, holds a
  reference to its `Bus`, and exposes lifecycle hooks. Actual I/O is
  defined by kind-specific derivations (`i2c::I2CMasterAccessor`,
  `spi::SPIMasterAccessor`, ...).

  `getConfig()` returns a const reference to the (derived) `AccessConfig`.
  It is named symmetrically with `Bus::getConfig() -> const BusConfig&`.
  Derived classes may narrow the return type covariantly (e.g.
  `const I2CMasterAccessConfig&`); the abstract base returns
  `const AccessConfig&`.

  `getBusKind()` delegates to `getConfig()` (single source of truth).
  Routing the lookup through `AccessConfig` leaves room for a future
  asymmetric setup where the bus and the accessor speak different kinds
  (for example, an I2C protocol accessor running on top of an SPI bus).
 */
struct Accessor {
    virtual ~Accessor(void)                           = default;
    virtual const AccessConfig& getConfig(void) const = 0;
    types::bus_kind_t getBusKind(void) const
    {
        return getConfig().getBusKind();
    }

    Accessor(Bus& bus) : _bus{bus}
    {
    }
    Bus& getBus(void) const
    {
        return _bus;
    };
    const BusConfig& getBusConfig(void) const;

    /*!
      @brief Open an access window on the underlying bus.

      Internally calls `_bus.lock(this, timeout_ms)` (with a paired
      `_bus.unlock(this)` from `endAccess`). The depth counter
      `_access_depth` collapses nested calls (e.g. an outer
      `ScopedAccess` plus an inner sugar method) into a single lock.

      @param timeout_ms Bus-lock acquisition timeout, in milliseconds
                       (currently advisory, see `Bus::lock`).
      @return Empty success, or an error code on lock failure.
     */
    m5::hal::v1::result_t<void> beginAccess(uint32_t timeout_ms = 0);
    /*!
      @brief Close one nesting level of the access window; release the
             bus lock on the outermost call.
     */
    m5::hal::v1::result_t<void> endAccess(void);
    /*! @brief Return whether the accessor currently holds an access window. */
    bool inAccess(void) const
    {
        return _access_depth > 0;
    }

protected:
    Bus& _bus;
    uint32_t _access_depth = 0;
};

//-------------------------------------------------------------------------
/*!
  @brief Abstract base for a communication bus.

  `getBusKind()` delegates to `BusConfig` (single source of truth).
  Only one virtual call (`getConfig()`) is needed; derived classes do
  not override `getBusKind()` because the kind is already forwarded
  into `BusConfig` by their constructor.
 */
struct Bus {
public:
    virtual ~Bus()                                 = default;
    virtual const BusConfig& getConfig(void) const = 0;
    types::bus_kind_t getBusKind(void) const
    {
        return getConfig().getBusKind();
    }

    // Bus initialization is intentionally NOT part of this abstract
    // interface. Initialization inherently needs variant-specific data
    // (a TwoWire*, a port number, a device path, ...), so a kind-generic
    // `init` cannot exist; each concrete bus declares its own
    // non-virtual `init(const <Variant>BusConfig&)` taking exactly the
    // config type it can act on (variants without extra fields take the
    // abstract kind config). The former base virtual only enabled
    // passing a sibling config, which the mandatory downcast turned
    // into UB (S17 E1). Return type matches the other public APIs
    // (`lock` / `unlock` / `transfer` / ...), so callers use
    // `if (auto r = bus.init(cfg); !r) ...` uniformly.

    /*! @brief Release any resources acquired by the concrete bus's `init`. */
    virtual result_t<void> release(void)
    {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }

    /*!
      @brief Acquire mutual exclusion on the bus for an owner.

      `owner` must be a valid `Accessor*`; `nullptr` returns
      `INVALID_ARGUMENT`. If anyone else already holds the lock, returns
      `BUSY`. Re-locking from the same owner also returns `BUSY`:
      `Bus::lock` is invoked at most once per access window; nesting is
      absorbed by the depth counter in `Accessor::beginAccess`.

      @param owner       Locking accessor; required.
      @param timeout_ms  Acquisition timeout (reserved for the future
                         mutex-backed implementation; currently ignored
                         because the native test environment is
                         single-threaded).
      @retval BUSY              The bus is already locked.
      @retval INVALID_ARGUMENT  `owner` is null.
     */
    virtual result_t<void> lock(Accessor* owner, uint32_t timeout_ms = 0)
    {
        (void)timeout_ms;
        if (owner == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        if (_lock_owner != nullptr) {
            return m5::stl::make_unexpected(error::error_t::BUSY);
        }
        _lock_owner = owner;
        return {};
    }

    /*!
      @brief Release the bus lock.

      `owner` must match the accessor that took the lock; a mismatch or
      an unlock without a preceding lock returns `INVALID_ARGUMENT`.
     */
    virtual result_t<void> unlock(Accessor* owner)
    {
        if (owner == nullptr || _lock_owner != owner) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        _lock_owner = nullptr;
        return {};
    }

protected:
    Accessor* _lock_owner = nullptr;  // nullptr = not currently locked
};

//-------------------------------------------------------------------------
// Definitions of Accessor::beginAccess / endAccess. Bus is only
// forward-declared inside Accessor, so the bodies live here after the
// full Bus definition.
inline m5::hal::v1::result_t<void> Accessor::beginAccess(uint32_t timeout_ms)
{
    if (_access_depth == 0) {
        auto r = _bus.lock(this, timeout_ms);
        if (!r.has_value()) {
            return r;
        }
    }
    ++_access_depth;
    return {};
}

inline m5::hal::v1::result_t<void> Accessor::endAccess(void)
{
    if (_access_depth == 0) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    --_access_depth;
    if (_access_depth == 0) {
        return _bus.unlock(this);
    }
    return {};
}

//-------------------------------------------------------------------------
/*!
  @brief RAII helper that wraps `Accessor::beginAccess` / `endAccess`.

  Nested with sugar methods that also call `beginAccess`, the depth
  counter folds the layers naturally. Both move and copy are deleted
  because the scope is not meant to outlive its lexical block. A
  lock-acquisition failure is observed via `has_error()` / `error()`.

  Polarity: success = `!scope.has_error()`. There is deliberately no
  `operator bool` - "truthy scope = acquired" and "truthy = has error"
  are both plausible readings, so the check must spell the polarity
  out. `error()` is `OK` exactly when `has_error()` is false. The
  same applies to `ScopedLock` below.
 */
class ScopedAccess {
public:
    explicit ScopedAccess(Accessor& accessor, uint32_t timeout_ms = 0) : _accessor{&accessor}
    {
        auto r = _accessor->beginAccess(timeout_ms);
        if (!r.has_value()) {
            _error    = r.error();
            _accessor = nullptr;  // dtor will not call endAccess
        }
    }
    ~ScopedAccess()
    {
        if (_accessor != nullptr) {
            (void)_accessor->endAccess();
        }
    }
    ScopedAccess(const ScopedAccess&)            = delete;
    ScopedAccess& operator=(const ScopedAccess&) = delete;
    ScopedAccess(ScopedAccess&&)                 = delete;
    ScopedAccess& operator=(ScopedAccess&&)      = delete;

    bool has_error(void) const
    {
        return _accessor == nullptr;
    }
    m5::hal::v1::error::error_t error(void) const
    {
        return _error;
    }

private:
    Accessor* _accessor                = nullptr;
    m5::hal::v1::error::error_t _error = m5::hal::v1::error::error_t::OK;
};

/*!
  @brief RAII helper that wraps `Bus::lock` / `Bus::unlock`.

  For low-level callers who want to make several
  `bus.transfer(&accessor, ...)` calls atomic. Move and copy are
  deleted; failure is observed via `has_error()` / `error()`.
 */
class ScopedLock {
public:
    ScopedLock(Bus& bus, Accessor* owner, uint32_t timeout_ms = 0) : _bus{&bus}, _owner{owner}
    {
        auto r = _bus->lock(_owner, timeout_ms);
        if (!r.has_value()) {
            _error = r.error();
            _bus   = nullptr;  // dtor will not call unlock
        }
    }
    ~ScopedLock()
    {
        if (_bus != nullptr) {
            (void)_bus->unlock(_owner);
        }
    }
    ScopedLock(const ScopedLock&)            = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
    ScopedLock(ScopedLock&&)                 = delete;
    ScopedLock& operator=(ScopedLock&&)      = delete;

    bool has_error(void) const
    {
        return _bus == nullptr;
    }
    m5::hal::v1::error::error_t error(void) const
    {
        return _error;
    }

private:
    Bus* _bus                          = nullptr;
    Accessor* _owner                   = nullptr;
    m5::hal::v1::error::error_t _error = m5::hal::v1::error::error_t::OK;
};

/*!
  @brief Run `body` bracketed by `begin` / `end` with the shared
         release-error policy.

  The policy every accessor sugar method follows: a `begin` failure
  returns immediately (nothing to release), the `body` error wins over
  an `end` error, but a clean body must not hide a broken release —
  depth-counter corruption would otherwise go unnoticed. `end` always
  runs once `begin` succeeded, even when the body failed.

  All three callables return a `result_t`; the body's result type is
  the call's result type.
 */
template <typename BeginFn, typename BodyFn, typename EndFn>
auto guarded(BeginFn&& begin, BodyFn&& body, EndFn&& end) -> decltype(body())
{
    auto b = begin();
    if (!b.has_value()) {
        return m5::stl::make_unexpected(b.error());
    }
    auto result = body();
    auto e      = end();
    if (!result.has_value()) {
        return result;
    }
    if (!e.has_value()) {
        return m5::stl::make_unexpected(e.error());
    }
    return result;
}

}  // namespace m5::hal::v1::bus

#endif
