// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_HPP_
#define M5_HAL_BUS_HPP_

#include "../assert.hpp"
#include "../error.hpp"
#include "../runtime/runtime.hpp"
#include "../types.hpp"

#include <M5Utility.hpp>

/*!
  @namespace m5::hal::v1::bus
  @brief IBus / IAccessor abstractions shared by every kind (I2C, SPI, ...).
 */
namespace m5::hal::v1::bus {

struct IBus;
struct IAccessConfig;
struct IAccessor;

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
struct IBusConfig {
    types::bus_kind_t bus_kind;  ///< Kind tag (`I2C`, `SPI`, ...). Set by the derived ctor.
    types::bus_kind_t getBusKind(void) const
    {
        return bus_kind;
    }

protected:
    explicit constexpr IBusConfig(types::bus_kind_t k) : bus_kind{k}
    {
    }
};

//-------------------------------------------------------------------------
/*!
  @brief Marker base for per-call transfer metadata.

  Each bus kind defines its own derivation (`i2c::ITransferDesc`,
  `spi::ITransferDesc`, etc.) carrying prefix bytes and per-transfer
  flags. This base has no virtual hook and no `getBusKind` — it exists
  only as a typed anchor.
 */
struct ITransferDesc {};

//-------------------------------------------------------------------------
/*!
  @brief Abstract base for accessor-side (per-target) configuration.

  The kind-tag convention matches `IBusConfig`: a non-virtual getter
  returns a field that the derived ctor forwards to this base ctor.
 */
struct IAccessConfig {
    types::bus_kind_t bus_kind;
    types::bus_kind_t getBusKind(void) const
    {
        return bus_kind;
    }

protected:
    explicit constexpr IAccessConfig(types::bus_kind_t k) : bus_kind{k}
    {
    }
};

//-------------------------------------------------------------------------
/*!
  @brief Abstract base for a per-target accessor.

  An `IAccessor` is the owner identity for an access window, holds a
  pointer to its `IBus`, and exposes lifecycle hooks. Actual I/O is
  defined by kind-specific derivations (`i2c::MasterAccessor`,
  `spi::MasterAccessor`, ...).

  An accessor may be constructed UNBOUND (no bus yet) and bound later
  through the derivation's kind-typed `bind()` — the
  "global driver object, begin(bus) in setup()" pattern. Unbound use
  is a contract violation gated at the access-window entry points
  (`beginAccess` and the kind-specific window openers): debug builds
  assert, release builds return `INVALID_ARGUMENT`. Hot-path calls
  below the gate (transfer and the sugars, which all pass through a
  gate first) skip the check by design.

  `getConfig()` returns a const reference to the (derived) `IAccessConfig`.
  It is named symmetrically with `IBus::getConfig() -> const IBusConfig&`.
  Derived classes may narrow the return type covariantly (e.g.
  `const MasterAccessConfig&`); the abstract base returns
  `const IAccessConfig&`.

  `getBusKind()` delegates to `getConfig()` (single source of truth).
  Routing the lookup through `IAccessConfig` leaves room for a future
  asymmetric setup where the bus and the accessor speak different kinds
  (for example, an I2C protocol accessor running on top of an SPI bus).
 */
struct IAccessor {
    virtual ~IAccessor(void)                           = default;
    virtual const IAccessConfig& getConfig(void) const = 0;
    types::bus_kind_t getBusKind(void) const
    {
        return getConfig().getBusKind();
    }

    IAccessor(IBus& bus) : _bus{&bus}
    {
    }
    /*! @brief Whether a bus is currently bound. */
    bool isBound(void) const
    {
        return _bus != nullptr;
    }
    /*!
      @brief Return the bound bus.

      Calling this on an unbound accessor is a contract violation
      (ungated null dereference); check `isBound()` when in doubt.
     */
    IBus& getBus(void) const
    {
        return *_bus;
    };
    const IBusConfig& getBusConfig(void) const;

    /*!
      @brief Open an access window on the underlying bus.

      Internally calls `_bus.lock(this, timeout_ms)` (with a paired
      `_bus.unlock(this)` from `endAccess`). The depth counter
      `_access_depth` collapses nested calls (e.g. an outer
      `ScopedAccess` plus an inner sugar method) into a single lock.

      @param timeout_ms Bus-lock acquisition timeout, in milliseconds;
                       0 = immediate try-lock, `types::TIMEOUT_FOREVER`
                       (the default) = wait until acquired (semantics:
                       `IBus::lock`). Prefer an explicit budget you can
                       handle on expiry; the infinite default is the
                       sugar for call sites where handling a timeout is
                       more trouble than it is worth.
      @return Empty success, or `TIMEOUT_ERROR` when the bus stayed
              held by another owner for the whole (finite) timeout.
     */
    m5::hal::v1::result_t<void> beginAccess(uint32_t timeout_ms = types::TIMEOUT_FOREVER);
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
    // Unbound construction is protected: only a derivation that also
    // offers the kind-typed bind() may expose it.
    IAccessor(void) = default;
    /*!
      @brief Bind (or rebind) the bus. Backs the derivation's typed bind().

      Must not be called while an access window is open — the open
      window holds the previous bus's lock. Derivations enforce that
      (`bind` returns `INVALID_ARGUMENT`) before delegating here.
     */
    void _bindBus(IBus& bus)
    {
        _bus = &bus;
    }

    IBus* _bus             = nullptr;
    uint32_t _access_depth = 0;
};

//-------------------------------------------------------------------------
/*!
  @brief Abstract base for a communication bus.

  `getBusKind()` delegates to `IBusConfig` (single source of truth).
  Only one virtual call (`getConfig()`) is needed; derived classes do
  not override `getBusKind()` because the kind is already forwarded
  into `IBusConfig` by their constructor.
 */
struct IBus {
public:
    virtual ~IBus()                                 = default;
    virtual const IBusConfig& getConfig(void) const = 0;
    types::bus_kind_t getBusKind(void) const
    {
        return getConfig().getBusKind();
    }

    // IBus initialization is intentionally NOT part of this abstract
    // interface. Initialization inherently needs variant-specific data
    // (a TwoWire*, a port number, a device path, ...), so a kind-generic
    // `init` cannot exist; each concrete bus declares its own
    // non-virtual `init(const <Variant>IBusConfig&)` taking exactly the
    // config type it can act on (variants without extra fields take the
    // abstract kind config). The former base virtual only enabled
    // passing a sibling config, which the mandatory downcast turned
    // into UB. Return type matches the other public APIs
    // (`lock` / `unlock` / `transfer` / ...), so callers use
    // `if (auto r = bus.init(cfg); !r) ...` uniformly.

    /*! @brief Release any resources acquired by the concrete bus's `init`. */
    virtual result_t<void> release(void)
    {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }

    /*!
      @brief Acquire mutual exclusion on the bus for an owner.

      Backed by the always-embedded `runtime::Mutex`: the call
      WAITS for the current holder up to `timeout_ms` and fails with
      `TIMEOUT_ERROR` when the mutex could not be taken; `timeout_ms`
      of 0 is an immediate try-lock and `types::TIMEOUT_FOREVER` (the
      default) blocks until acquired. Non-recursive — `IBus::lock` is
      invoked at most once per access window (nesting is absorbed by
      the depth counter in `IAccessor::beginAccess`), and a re-lock from
      the holding task (same owner or another accessor) also waits
      until the timeout and fails — with TIMEOUT_FOREVER that is a
      deadlock (fail-loud: the task watchdog fires). Task context only;
      never call from an ISR. Timeout granularity follows the
      runtime variant (one FreeRTOS tick — 10 ms by default — on the
      embedded targets).

      `owner` must be a valid `IAccessor*`; `nullptr` returns
      `INVALID_ARGUMENT`. The `_lock_owner` bookkeeping happens with
      the mutex held on both lock and unlock.

      @param owner       Locking accessor; required.
      @param timeout_ms  Acquisition timeout in milliseconds; 0 tries
                         once and returns immediately,
                         `types::TIMEOUT_FOREVER` waits indefinitely.
      @retval TIMEOUT_ERROR     The bus was still held after timeout_ms.
      @retval INVALID_ARGUMENT  `owner` is null.
     */
    virtual result_t<void> lock(IAccessor* owner, uint32_t timeout_ms = types::TIMEOUT_FOREVER)
    {
        if (owner == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        if (!_mutex.lock(timeout_ms)) {
            return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
        }
        _lock_owner = owner;
        return {};
    }

    /*!
      @brief Release the bus lock.

      `owner` must match the accessor that took the lock; a mismatch or
      an unlock without a preceding lock returns `INVALID_ARGUMENT`
      without touching the mutex. Must be called from the task that
      locked (a FreeRTOS mutex requirement).
     */
    virtual result_t<void> unlock(IAccessor* owner)
    {
        if (owner == nullptr || _lock_owner != owner) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        _lock_owner = nullptr;  // cleared while the mutex is still held
        _mutex.unlock();
        return {};
    }

protected:
    runtime::Mutex _mutex;             // always embedded; backs lock()/unlock()
    IAccessor* _lock_owner = nullptr;  // nullptr = not currently locked
};

//-------------------------------------------------------------------------
// Definitions of IAccessor::beginAccess / endAccess. IBus is only
// forward-declared inside IAccessor, so the bodies live here after the
// full IBus definition.
inline m5::hal::v1::result_t<void> IAccessor::beginAccess(uint32_t timeout_ms)
{
    // The unbound gate: every sugar funnels through an access-window
    // opener, so this single check covers the hot paths below it.
    M5HAL_ASSERT(_bus != nullptr, "accessor is not bound to a bus (bind() it first)");
    if (_bus == nullptr) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    if (_access_depth == 0) {
        auto r = _bus->lock(this, timeout_ms);
        if (!r.has_value()) {
            return r;
        }
    }
    ++_access_depth;
    return {};
}

inline m5::hal::v1::result_t<void> IAccessor::endAccess(void)
{
    if (_access_depth == 0) {
        return m5::stl::make_unexpected(m5::hal::v1::error::error_t::INVALID_ARGUMENT);
    }
    --_access_depth;
    if (_access_depth == 0) {
        return _bus->unlock(this);
    }
    return {};
}

//-------------------------------------------------------------------------
/*!
  @brief RAII helper that wraps `IAccessor::beginAccess` / `endAccess`.

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
    explicit ScopedAccess(IAccessor& accessor, uint32_t timeout_ms = types::TIMEOUT_FOREVER) : _accessor{&accessor}
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
    IAccessor* _accessor               = nullptr;
    m5::hal::v1::error::error_t _error = m5::hal::v1::error::error_t::OK;
};

/*!
  @brief RAII helper that wraps `IBus::lock` / `IBus::unlock`.

  For low-level callers who want to make several
  `bus.transfer(&accessor, ...)` calls atomic. Move and copy are
  deleted; failure is observed via `has_error()` / `error()`.
 */
class ScopedLock {
public:
    ScopedLock(IBus& bus, IAccessor* owner, uint32_t timeout_ms = types::TIMEOUT_FOREVER) : _bus{&bus}, _owner{owner}
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
    IBus* _bus                         = nullptr;
    IAccessor* _owner                  = nullptr;
    m5::hal::v1::error::error_t _error = m5::hal::v1::error::error_t::OK;
};

/*!
  @brief Non-owning bus reference registry (slot -> bus).

  The HAL neither creates nor owns buses — the user does (the v1
  ownership model). What a board-support layer still needs is a place
  to PUBLISH its wiring: "slot 1 is the SD bus, slot 2 is the LCD bus".
  A `BusGroup` is that place, one per kind on `M5_Hal` (`M5_Hal.SPI`,
  ...), with the same shape as `GPIOGroup::addGPIO`:

  @code
  static m5::hal::v1::spi::Bus spi_bus;             // user-owned
  M5_Hal.SPI.addBus(&spi_bus, 1);                   // publish
  auto* bus = M5_Hal.SPI.getBus(1);                 // look up (nullptr = empty)
  @endcode

  - **Aliasing is natural**: registering the SAME pointer in several
    slots expresses "slot 1 (SD) and slot 2 (LCD) are one physical
    bus". The registry stores references, not instances, so nothing
    special is needed.
  - **Slot meanings belong to the upper layer** (a board-support
    package names its slots with constants); the HAL only provides the
    table.
  - **Lifetime rule**: registered buses should have static storage
    duration; call `removeBus` before destroying or `release()`-ing a
    registered bus. The registry never deletes.
  - Registration is a startup-time operation; afterwards the table is
    treated as read-only (no locking), like `GPIOGroup`.
 */
template <typename BusT>
class BusGroup {
public:
    /*! @brief Fixed slot count (a few pointers per kind). */
    static constexpr size_t kSlotCount = 8;

    constexpr BusGroup() noexcept = default;

    BusGroup(const BusGroup&)            = delete;
    BusGroup& operator=(const BusGroup&) = delete;
    BusGroup(BusGroup&&)                 = delete;
    BusGroup& operator=(BusGroup&&)      = delete;

    /*!
      @brief Publish `bus` at `slot`.

      Rejected (`INVALID_ARGUMENT`) when `bus == nullptr`,
      `slot >= kSlotCount`, or the slot is already in use. The same
      bus MAY occupy several slots (aliasing).
     */
    [[nodiscard]] result_t<void> addBus(BusT* bus, size_t slot)
    {
        if (bus == nullptr || slot >= kSlotCount || _slots[slot] != nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        _slots[slot] = bus;
        return {};
    }

    /*!
      @brief Withdraw the registration at `slot`.

      Rejected when `slot >= kSlotCount` or the slot is empty. Only the
      table entry is cleared — the bus object is untouched.
     */
    [[nodiscard]] result_t<void> removeBus(size_t slot)
    {
        if (slot >= kSlotCount || _slots[slot] == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        _slots[slot] = nullptr;
        return {};
    }

    /*! @brief Bus registered at `slot`, or `nullptr` (empty / out of range). */
    BusT* getBus(size_t slot) const
    {
        return slot < kSlotCount ? _slots[slot] : nullptr;
    }

    bool hasBus(size_t slot) const
    {
        return getBus(slot) != nullptr;
    }

private:
    BusT* _slots[kSlotCount] = {};
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
