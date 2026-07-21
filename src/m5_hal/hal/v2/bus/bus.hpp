// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_HPP_
#define M5_HAL_BUS_HPP_

#include "../assert.hpp"
#include "../error.hpp"
#include "../runtime/runtime.hpp"
#include "../types.hpp"
#include "capabilities.hpp"
#include "native_policy.hpp"
#include "operation.hpp"

#include <M5Utility.hpp>

#include <atomic>
#include <memory>
#include <utility>

namespace m5::hal::v2::gpio {
class GPIOGroup;
}
namespace m5::hal::v2::service {
class ServiceRunner;
}
namespace m5::hal::v2::memory {
class Allocator;
}

/*!
  @namespace m5::hal::v2::bus
  @brief IBus / IAccessor abstractions shared by every kind (I2C, SPI, ...).
 */
namespace m5::hal::v2::bus {

class BusRegistry;
struct IHalBackend;
struct ResourceKey;

/*! @brief Dependencies captured once when a local Bus is created. */
struct LocalResourceContext {
    gpio::GPIOGroup* gpio            = nullptr;
    service::ServiceRunner* services = nullptr;
    memory::Allocator* memory        = nullptr;
    std::shared_ptr<void> lifetime;

    bool valid(void) const
    {
        return gpio != nullptr && services != nullptr && memory != nullptr && lifetime != nullptr;
    }
};

/*! @brief Compatibility/bootstrap resources used only by directly built local buses. */
const LocalResourceContext& defaultLocalResources(void);

struct IBus;
struct IAccessConfig;
struct IAccessor;

/*! @brief How much backend state a failed close may have changed. */
enum class CloseDisposition : uint8_t { Success, NoMutation, PartialOrUnknown };

/*! @brief Internal close result used to decide registry rollback vs quarantine. */
struct CloseOutcome {
    CloseDisposition disposition = CloseDisposition::Success;
    error::error_t error_code    = error::error_t::OK;

    static CloseOutcome success(void)
    {
        return {};
    }
    static CloseOutcome noMutation(error::error_t e)
    {
        return {CloseDisposition::NoMutation, e};
    }
    static CloseOutcome partialOrUnknown(error::error_t e)
    {
        return {CloseDisposition::PartialOrUnknown, e};
    }
};

/*!
  @brief Shared operation/close gate for a bus whose external identity may
         outlive its concrete proxy.

  Operations hold an `Operation` for their complete use of the external
  resource.  A close first changes Open -> Releasing while holding the same
  mutex, so it waits for earlier operations and excludes later ones.  A failed
  close rolls back to Open; a successful close leaves a permanent Closed
  tombstone.  The registry retains this object after the bus weak_ptr expires,
  preventing an identity (and, for remote buses, its numeric id) from being
  reused while natural destruction is still releasing it.
 */
class BusLifecycle {
public:
    enum class State : uint8_t { Open, Releasing, Quarantined, Closed };

    class Operation {
    public:
        explicit Operation(BusLifecycle& lifecycle, uint32_t timeout_ms = types::TIMEOUT_FOREVER)
            : _lifecycle{&lifecycle}
        {
            const auto locked = lifecycle._mutex.lock(timeout_ms);
            _locked           = locked.has_value();
            if (!_locked) {
                _error = locked.error();
            } else if (lifecycle._state.load(std::memory_order_acquire) != State::Open) {
                _error        = error::error_t::CLOSED;
                auto unlocked = lifecycle._mutex.unlock();
                _locked       = false;
                if (!unlocked.has_value()) {
                    lifecycle.quarantineWithoutLock();
                    _error = unlocked.error();
                }
            }
        }
        ~Operation()
        {
            if (_locked) {
                auto unlocked = _lifecycle->_mutex.unlock();
                if (!unlocked.has_value()) {
                    _lifecycle->quarantineWithoutLock();
                }
            }
        }
        Operation(const Operation&)            = delete;
        Operation& operator=(const Operation&) = delete;

        explicit operator bool() const
        {
            return _locked;
        }
        error::error_t error() const
        {
            return _error;
        }

    private:
        BusLifecycle* _lifecycle = nullptr;
        bool _locked             = false;
        error::error_t _error    = error::error_t::OK;
    };

    class Close {
    public:
        explicit Close(BusLifecycle& lifecycle, uint32_t timeout_ms = types::TIMEOUT_FOREVER) : _lifecycle{&lifecycle}
        {
            const auto locked = lifecycle._mutex.lock(timeout_ms);
            _locked           = locked.has_value();
            _error            = locked.has_value() ? error::error_t::OK : locked.error();
            if (_locked) {
                if (lifecycle._state.load(std::memory_order_acquire) == State::Open) {
                    lifecycle._state.store(State::Releasing, std::memory_order_release);
                } else {
                    auto unlocked = lifecycle._mutex.unlock();
                    _locked       = false;
                    if (!unlocked.has_value()) {
                        lifecycle.quarantineWithoutLock();
                        _error = unlocked.error();
                    } else {
                        _error = error::error_t::BUSY;
                    }
                }
            }
        }
        ~Close()
        {
            if (_locked) {
                // An unfinished close is a failed close.
                _lifecycle->_state.store(State::Open, std::memory_order_release);
                auto unlocked = _lifecycle->_mutex.unlock();
                if (!unlocked.has_value()) {
                    _lifecycle->quarantineWithoutLock();
                }
            }
        }
        Close(const Close&)            = delete;
        Close& operator=(const Close&) = delete;

        explicit operator bool() const
        {
            return _locked;
        }
        error::error_t error() const
        {
            return _locked ? error::error_t::OK
                           : (_error == error::error_t::TIMEOUT_ERROR ? error::error_t::BUSY : _error);
        }
        result_t<void> rollback()
        {
            return finish(State::Open);
        }
        result_t<void> commit()
        {
            return finish(State::Closed);
        }
        result_t<void> quarantine()
        {
            return finish(State::Quarantined);
        }

    private:
        result_t<void> finish(State state)
        {
            if (_locked) {
                _lifecycle->_state.store(state, std::memory_order_release);
                _locked       = false;
                auto unlocked = _lifecycle->_mutex.unlock();
                if (!unlocked.has_value()) {
                    _lifecycle->quarantineWithoutLock();
                }
                return unlocked;
            }
            return {};
        }
        BusLifecycle* _lifecycle = nullptr;
        bool _locked             = false;
        error::error_t _error    = error::error_t::OK;
    };

    State state() const
    {
        return _state.load(std::memory_order_acquire);
    }

    // Destruction-time escape when the current callback already owns the
    // non-recursive operation gate. No id may be reused after this path.
    void quarantineWithoutLock()
    {
        _state.store(State::Quarantined, std::memory_order_release);
    }

private:
    mutable runtime::Mutex _mutex;
    std::atomic<State> _state{State::Open};
};

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

  An `IAccessor` is the owner identity for an access, holds a pointer to
  its `IBus`, and provides the shared implementation used by kind-specific
  lifecycle APIs. Actual `beginAccess` / `endAccess` and I/O are defined by
  concrete derivations (`i2c::MasterAccessor`, `spi::MasterAccessor`, ...),
  because their typed OperationContext and backend hooks differ.

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
    types::bus_kind_t getBusKind(void) const;

    IAccessor(IBus& bus);
    /*!
      @brief Co-owning construction: the accessor shares ownership of the bus.

      The canonical registry-acquire path: `M5_Hal.<kind>.acquire(cfg)` hands
      back an owning `shared_ptr<IBus>`, and constructing an accessor from it
      keeps the bus alive for as long as the accessor lives. This remains safe
      even when the caller does not separately retain the
      `shared_ptr` (e.g. `Accessor dev{acquire(cfg).value(), cfg}` — the
      temporary would otherwise drop and dangle the bus).

      The hot path still goes through the raw `_bus`; `_owner` only pins the
      lifetime. The `IBus&` overload above is the escape for callers that own
      the bus themselves (direct construction), leaving `_owner` empty.
     */
    explicit IAccessor(std::shared_ptr<IBus> owner);
    /*! @brief Whether a bus is currently bound. */
    bool isBound(void) const;
    /*!
      @brief Return the bound bus.

      Calling this on an unbound accessor is a contract violation
      (ungated null dereference); check `isBound()` when in doubt.
     */
    IBus& getBus(void) const;
    const IBusConfig& getBusConfig(void) const;

    /*! @brief Return whether the concrete accessor currently owns an operation. */
    bool inAccess(void) const;

    /*!
      @brief Sibling accessor sharing one logical lock scope with this one
             (combined-accessor plumbing — e.g. the TX/RX children of
             `uart::Accessor`). Reconfiguration gates use it to recognize
             that "the opposite channel's holder" is in fact the caller
             itself. Set once at construction of the combined accessor;
             never mutated afterwards (not thread-safe to change while in
             use).
     */
    IAccessor* lockPeer(void) const
    {
        return _lock_peer;
    }
    void setLockPeer(IAccessor* peer)
    {
        _lock_peer = peer;
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

      `bind` takes the bus by reference, so the accessor borrows it without
      owning it: any prior co-ownership (`_owner`, from a shared_ptr ctor) is
      dropped here. This keeps the invariant "`_owner` is empty, or owns the
      bus `_bus` points to" — otherwise rebinding would leak the old bus and
      leave `_owner` pointing at a different bus than `_bus`.
     */
    void _bindBus(IBus& bus);

    template <class Config>
    OperationContext<Config> makeOperationContext(const Config& config)
    {
        return OperationContext<Config>{*this, config};
    }

    template <class Context, class BeginHook>
    result_t<void> _beginOperationAccess(Context& context, uint32_t timeout_ms, OperationMode mode,
                                         BeginHook&& begin_hook);

    template <class Context, class LockHook, class UnlockHook, class BeginHook>
    result_t<void> _beginOperationAccess(Context& context, uint32_t timeout_ms, OperationMode mode,
                                         LockHook&& lock_hook, UnlockHook&& unlock_hook, BeginHook&& begin_hook);

    template <class Context, class EndHook>
    result_t<void> _endOperationAccess(Context& context, uint32_t timeout_ms, EndHook&& end_hook);

    template <class Context, class EndHook, class UnlockHook>
    result_t<void> _endOperationAccess(Context& context, uint32_t timeout_ms, EndHook&& end_hook,
                                       UnlockHook&& unlock_hook);

    bool _inOperationAccess() const
    {
        return _operation_active;
    }

    IBus* _bus             = nullptr;
    bool _operation_active = false;
    // Empty unless the accessor co-owns its bus (constructed from a
    // shared_ptr). Pins the bus lifetime; `_bus` aliases `_owner.get()`.
    std::shared_ptr<IBus> _owner{};
    // nullptr unless this accessor is one half of a combined accessor (see
    // lockPeer() above).
    IAccessor* _lock_peer = nullptr;
};

//-------------------------------------------------------------------------
/*!
  @brief Abstract base for a communication bus.

  Instances are obtained via `hal.<KIND>.acquire(cfg)` on a `Hal` facade
  (e.g. `M5_Hal.I2C.acquire(cfg)`) — see the per-kind headers' file docs
  for the entry-point story.

  `getBusKind()` delegates to `IBusConfig` (single source of truth).
  Only one virtual call (`getConfig()`) is needed; derived classes do
  not override `getBusKind()` because the kind is already forwarded
  into `IBusConfig` by their constructor.
 */
struct IBus {
public:
    virtual ~IBus()                                 = default;
    virtual const IBusConfig& getConfig(void) const = 0;
    /*! @brief Optional lifetime tombstone used by externally-backed buses. */
    virtual std::shared_ptr<BusLifecycle> lifecycleHandle(void) const
    {
        return {};
    }
    void bindLocalResources(const LocalResourceContext& resources)
    {
        _local_resources = resources;
    }
    const LocalResourceContext& localResourceContext(void) const
    {
        return localResources();
    }
    virtual bool bindRegistryRegistration(BusRegistry&, const ResourceKey&, uint16_t, uint32_t)
    {
        return false;
    }
    virtual const ResourceKey* registryResourceKey(void) const
    {
        return nullptr;
    }
    types::bus_kind_t getBusKind(void) const;

    // IBus initialization is intentionally NOT part of this abstract
    // interface. Each kind has one portable config, while a concrete provider
    // may additionally accept native::borrowed/managed policy arguments.
    // Those overloads cannot be represented by one kind-generic virtual init;
    // direct concrete/facade buses therefore declare non-virtual init methods.
    // The result_t return matches other fallible public commands, so callers
    // use `if (auto r = bus.init(cfg); !r) ...` uniformly.

    // Backend query API. These describe HOW the bus is
    // currently driven, so a holder can react to a hot-swap (e.g. a
    // hardware->software downgrade when a controller is reassigned). They
    // are non-pure with safe defaults: any bus that has not opted into the
    // swappable-backend model answers "software / no controller / unknown
    // ceiling / never swapped", which never misleads a caller into assuming
    // hardware guarantees. The runtime facade forwards these to its live
    // backend so the answers track swaps.

    /*!
      @brief Whether this bus is driven by hardware or software right now.

      Defaults to `Software` (the safe answer: an un-migrated bus never
      claims hardware it does not have).
     */
    virtual types::backend_kind_t backendKind(void) const;

    /*!
      @brief Identifier of the hardware controller backing this bus, or -1.

      -1 = no dedicated controller (software backend, or a kind that does
      not participate in HW allocation). For hardware backends this is the
      peripheral index (e.g. an I2C port number).
     */
    virtual int8_t controllerId(void) const;

    /*!
      @brief Upper frequency the current backend can sustain, in Hz; 0 = unknown.

      Lets a holder notice a capability drop after a downgrade to software
      (declare the ceiling rather than silently slow down).
     */
    virtual uint32_t maxFrequency(void) const;

    /*!
      @brief Monotonic counter bumped on every backend swap (poll baseline).

      A holder that cached `backendKind()` / `maxFrequency()` detects a
      change by comparing this against a previously read value -- no
      callback needed. Stays 0 for buses that never swap.
     */
    virtual uint32_t backendGeneration(void) const;

    /*!
      @brief Allocation-free snapshot of this exact Bus instance generation.

      The returned value is independent of the Bus lifetime. A facade backend
      swap affects only later snapshots; previously returned values do not
      change.
     */
    virtual BusCapabilities capabilities(void) const;

protected:
    /*! @brief Close a directly owned bus and release its backend resources. */
    [[nodiscard]] result_t<void> close(void);

    /*!
      @brief Provider teardown hook with mutation classification.

      Providers that own resources override this hook. The default succeeds
      for resource-free proxies and test doubles.
     */
    virtual CloseOutcome closeBackend(void);

    /*!
      @brief Internal direct-init lifecycle gate.

      A successfully closed directly constructed facade may be initialized
      again. Registry-bound facades are final once closed, and a facade whose
      teardown is in progress or quarantined cannot be reinitialized.
     */
    bool initializationAllowed(bool registry_bound) const;
    result_t<void> markInitializationSucceeded(bool registry_bound);
    static CloseOutcome closeOwnedBackend(IBus& backend)
    {
        return backend.closeWithOutcome();
    }
    void quarantineLifecycleAfterPartialTeardown(void)
    {
        _close_state.store(CloseState::Quarantined, std::memory_order_release);
    }
    bool accessLifecycleOpen(void) const
    {
        return _close_state.load(std::memory_order_acquire) == CloseState::Open;
    }
    virtual result_t<void> tryAcquireCloseBarrier(void);
    virtual result_t<void> releaseCloseBarrier(void);

    const LocalResourceContext& localResources(void) const
    {
        return _local_resources.valid() ? _local_resources : defaultLocalResources();
    }

private:
    friend class BusRegistry;
    friend struct IHalBackend;
    void markRegistryBound(void)
    {
        _registry_bound.store(true, std::memory_order_release);
    }
    CloseOutcome closeWithOutcome(void);
    LocalResourceContext _local_resources;
    // Use a natural-width atomic.  Xtensa LX106 does not provide the byte CAS
    // helper (`__atomic_compare_exchange_1`) in its default Arduino link, while
    // close arbitration requires compare_exchange to remain lock-free and
    // allocation-free on every supported target.
    enum class CloseState : uint32_t { Open, Closing, Quarantined, Closed };
    std::atomic<CloseState> _close_state{CloseState::Open};
    std::atomic<bool> _registry_bound{false};

protected:
    /*!
      @brief Internal lock seam used by concrete Accessor lifecycles.

      This is deliberately not a public low-level access API. Concrete
      Accessors establish typed operation context before acquiring it and
      always pair it with `releaseAccessLock` during rollback or close.
     */
    virtual result_t<void> acquireAccessLock(IAccessor& owner, uint32_t timeout_ms = types::TIMEOUT_FOREVER);
    virtual result_t<void> releaseAccessLock(IAccessor& owner);

    bool accessBroken() const
    {
        return _access_broken.load(std::memory_order_acquire);
    }
    void markAccessBroken()
    {
        _access_broken.store(true, std::memory_order_release);
    }

    runtime::Mutex _mutex;             // always embedded; backs the Access lock
    IAccessor* _lock_owner = nullptr;  // nullptr = not currently locked

private:
    friend struct IAccessor;
    std::atomic<bool> _access_broken{false};
};

template <class Context, class BeginHook>
result_t<void> IAccessor::_beginOperationAccess(Context& context, uint32_t timeout_ms, OperationMode mode,
                                                BeginHook&& begin_hook)
{
    return _beginOperationAccess(
        context, timeout_ms, mode, [&](uint32_t remaining_ms) { return _bus->acquireAccessLock(*this, remaining_ms); },
        [&] { return _bus->releaseAccessLock(*this); }, std::forward<BeginHook>(begin_hook));
}

template <class Context, class LockHook, class UnlockHook, class BeginHook>
result_t<void> IAccessor::_beginOperationAccess(Context& context, uint32_t timeout_ms, OperationMode mode,
                                                LockHook&& lock_hook, UnlockHook&& unlock_hook, BeginHook&& begin_hook)
{
    if (_operation_active) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    M5HAL_ASSERT(_bus != nullptr, "accessor is not bound to a bus (bind() it first)");
    if (_bus == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    context.runtime.begin(runtime::millis(), timeout_ms, mode);
    if (_bus->accessBroken()) {
        context.runtime.state |= OperationStateFlags::Broken;
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    auto locked = lock_hook(remainingTimeout(context.runtime, runtime::millis()));
    if (!locked.has_value()) {
        return locked;
    }
    if (_bus->accessBroken()) {
        auto unlocked = unlock_hook();
        context.runtime.state |= OperationStateFlags::Broken;
        if (!unlocked.has_value()) {
            _bus->markAccessBroken();
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto begun = begin_hook(context);
    if (!begun.has_value()) {
        auto unlocked = unlock_hook();
        if (!unlocked.has_value()) {
            context.runtime.state |= OperationStateFlags::Broken;
            _bus->markAccessBroken();
        }
        return begun;
    }
    context.runtime.state |= OperationStateFlags::BackendStarted;
    _operation_active = true;
    return {};
}

template <class Context, class EndHook>
result_t<void> IAccessor::_endOperationAccess(Context& context, uint32_t timeout_ms, EndHook&& end_hook)
{
    return _endOperationAccess(context, timeout_ms, std::forward<EndHook>(end_hook),
                               [&] { return _bus->releaseAccessLock(*this); });
}

template <class Context, class EndHook, class UnlockHook>
result_t<void> IAccessor::_endOperationAccess(Context& context, uint32_t timeout_ms, EndHook&& end_hook,
                                              UnlockHook&& unlock_hook)
{
    if (!_operation_active) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    context.runtime.beginClose(runtime::millis(), timeout_ms);
    auto ended        = end_hook(context);
    _operation_active = false;
    auto unlocked     = unlock_hook();
    if (!unlocked.has_value()) {
        context.runtime.state |= OperationStateFlags::Broken;
        _bus->markAccessBroken();
    }
    if (!ended.has_value()) {
        return ended;
    }
    return unlocked;
}

//-------------------------------------------------------------------------
/*!
  @brief RAII helper that wraps an accessor's `beginAccess` / `endAccess`.

  Templated on the concrete accessor type so its own `beginAccess` / `endAccess`
  lifecycle is called by static dispatch. `IAccessor` intentionally has no
  type-erased lifecycle because it cannot carry a bus-kind `OperationContext`.
  Use CTAD:
  `ScopedAccess guard{tx_accessor};` deduces the accessor type. Master /
  single-channel accessors deduce to themselves and behave exactly as before.

  Sugar methods borrow an already-active Access instead of opening a nested
  one. Both move and copy are deleted because the scope is not meant to
  outlive its lexical block. An acquisition failure is observed via
  `has_error()` / `error()`.

  Polarity: success = `scope.ok()` (== `!scope.has_error()`). There is
  deliberately no `operator bool` - "truthy scope = acquired" and
  "truthy = has error" are both plausible readings, so the check must
  spell the polarity out; `ok()` makes the positive check explicit.
  `error()` is `OK` exactly when `has_error()` is false.
 */
template <class Accessor>
class ScopedAccess {
public:
    explicit ScopedAccess(Accessor& accessor, uint32_t timeout_ms = types::TIMEOUT_FOREVER) : _accessor{&accessor}
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

    result_t<void> finish(uint32_t timeout_ms = 1000)
    {
        if (_accessor == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        auto ended = endWithTimeout(*_accessor, timeout_ms, 0);
        _accessor  = nullptr;
        if (!ended.has_value()) {
            _error = ended.error();
        }
        return ended;
    }

    bool has_error(void) const
    {
        return error::isError(_error);
    }
    /*! @brief Success view: `true` when the scope acquired (== `!has_error()`). */
    bool ok(void) const
    {
        return !has_error();
    }
    m5::hal::v2::error::error_t error(void) const
    {
        return _error;
    }

private:
    template <class T>
    static auto endWithTimeout(T& accessor, uint32_t timeout_ms, int) -> decltype(accessor.endAccess(timeout_ms))
    {
        return accessor.endAccess(timeout_ms);
    }

    template <class T>
    static result_t<void> endWithTimeout(T& accessor, uint32_t, long)
    {
        return accessor.endAccess();
    }

    Accessor* _accessor                = nullptr;
    m5::hal::v2::error::error_t _error = m5::hal::v2::error::error_t::OK;
};

/*!
  @brief Non-owning bus reference registry (slot -> bus).

  This is NOT the entry point for obtaining a bus — that is
  `hal.<KIND>.acquire(cfg)`. `BusGroup` is a utility for upper layers
  (board-support packages) that want to publish slot-numbered wiring.

  The HAL neither creates nor owns buses — the user does (the v2
  ownership model). What a board-support layer still needs is a place
  to PUBLISH its wiring: "slot 1 is the SD bus, slot 2 is the LCD bus".
  A `BusGroup` is that place, one per kind on `M5_Hal` (`M5_Hal.SPI`,
  ...), with the same shape as `GPIOGroup::addGPIO`:

  @code
  static m5::hal::v2::spi::Bus spi_bus;             // user-owned
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
    duration; call `removeBus` before destroying or `close()`-ing a
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

}  // namespace m5::hal::v2::bus

#endif
