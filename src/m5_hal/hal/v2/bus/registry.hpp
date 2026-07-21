// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_REGISTRY_HPP_
#define M5_HAL_BUS_REGISTRY_HPP_

#include "./bus.hpp"  // IBus, runtime::Mutex, types, error, result_t, M5Utility
#include "./native_binding.hpp"
#include "./resource_key.hpp"  // ResourceKey, RegistryEntryToken

#include <cstdlib>
#include <limits>
#include <memory>
#include <utility>

/*!
  @namespace m5::hal::v2::bus
  @brief All-kind weak interning registry for buses.
 */
namespace m5::hal::v2::bus {

/*!
  @brief All-kind weak registry: interns buses by (kind, identity).

  Within one ResourceDomain, one exact ResourceKey maps to one shared instance,
  so a board-support layer and user code that name the same resource share a single bus (and its
  single lock) -- the correctness requirement the registry enforces. The registry
  holds `weak_ptr`, so a bus is destroyed (its `Bus` dtor runs, freeing the
  backend) once the last `shared_ptr` holder drops it. For an externally
  backed bus whose natural close is still pending or has failed, the slot
  keeps its lifecycle tombstone and identity until close; reacquisition then
  returns `BUSY` rather than aliasing an uncertain peer resource. A single
  mutex serializes reservation/publication. A miss becomes `Constructing`,
  the factory runs outside the mutex, and a competing acquire returns BUSY;
  two concurrent acquires cannot create two instances. Generation-bearing
  registration metadata closes stale-ticket and destructor weak-expiry ABA.
  Lookups never sit on a
  transfer hot path (accessors hold the bus directly), so the lock is free.

  Per-kind access goes through a typed view (e.g. `i2c::BusView`, exposed
  as `M5_Hal.I2C`) that validates the kind and hands back the kind-typed
  `shared_ptr`. The total live-bus budget (`kCapacity`) is shared across
  all kinds; it is a RAM backstop, distinct from any per-kind hardware
  resource pool.
 */
class BusRegistry {
    enum class SlotState : uint8_t { Empty, Constructing, Live, Closing, Quarantined };

public:
    /*! @brief Total live buses across all kinds (RAM budget). */
    static constexpr size_t kCapacity = 16;

    struct ReleaseTicket {
        RegistryEntryToken entry;
        ResourceKey key;
        const IBus* expected     = nullptr;
        bool retrying_quarantine = false;

        ReleaseTicket(void) = default;
        ReleaseTicket(RegistryEntryToken token, const ResourceKey& identity, const IBus* expected_bus,
                      bool retrying = false)
            : entry{token}, key{identity}, expected{expected_bus}, retrying_quarantine{retrying}
        {
        }

        bool valid(void) const
        {
            return entry.valid() && entry.slot < kCapacity && expected != nullptr;
        }
    };

    BusRegistry(void)                          = default;
    BusRegistry(const BusRegistry&)            = delete;
    BusRegistry& operator=(const BusRegistry&) = delete;

    /*!
      @brief Return the bus for (kind, id), creating it via `make` on a miss.

      Reservation and publication are atomic under the registry mutex. The
      potentially allocating or platform-calling `make` callback runs outside
      that mutex while the matching identity remains `Constructing`; another
      acquire of that identity returns `BUSY`. A hit returns the existing
      instance only after its exact provider/native binding and the caller's
      kind-specific configuration validator both accept it. A managed
      reassignment changes a live bus through its separate commit path. A miss
      calls `make` and interns the result. `make` is
      `() -> result_t<shared_ptr<IBus>>`; its error is
      propagated WITHOUT interning (a failed bus is never registered).
      Returns `OUT_OF_RESOURCE` when all `kCapacity` slots hold live buses.
     */
    template <class MakeFn>
    result_t<std::shared_ptr<IBus>> acquireOrFind(const ResourceKey& key, MakeFn&& make)
    {
        return acquireOrFind(
            key, [](const std::shared_ptr<IBus>&) -> result_t<void> { return {}; }, std::forward<MakeFn>(make));
    }

    /*!
      @brief `acquireOrFind` with an atomic live-hit validator.

      `validate(live)` runs under the same registry critical section that
      identifies the hit.  This is for configuration compatibility checks
      that must not race a release/reacquire of the same identity.
     */
    template <class ValidateFn, class MakeFn>
    result_t<std::shared_ptr<IBus>> acquireOrFind(const ResourceKey& key, ValidateFn&& validate, MakeFn&& make)
    {
        return acquireOrFind(key, BindingDescriptor{}, std::forward<ValidateFn>(validate), std::forward<MakeFn>(make));
    }

    /*!
      @brief Acquire with an exact provider/native binding descriptor.

      A live identity may be reused only when the complete binding matches.
      This comparison is performed under the registry lock before the
      kind-specific portable-config validator.
     */
    template <class ValidateFn, class MakeFn>
    result_t<std::shared_ptr<IBus>> acquireOrFind(const ResourceKey& key, const BindingDescriptor& binding,
                                                  ValidateFn&& validate, MakeFn&& make)
    {
        if (!key.isValid()) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        RegistryEntryToken reservation;
        {
            Guard guard{_mutex};
            int free_slot = -1;
            for (size_t i = 0; i < kCapacity; ++i) {
                Slot& s = _slots[i];
                if (s.state != SlotState::Empty && s.key == key) {
                    if (s.state != SlotState::Live) {
                        return m5::stl::make_unexpected(error::error_t::BUSY);
                    }
                    auto live = s.bus.lock();
                    if (!live) {
                        // A final owner can already have released its strong
                        // count while its destructor has not yet reserved
                        // Closing. Never create a second instance in that gap.
                        if (!s.destructor_managed &&
                            (!s.lifecycle || s.lifecycle->state() == BusLifecycle::State::Closed)) {
                            clearSlot(s);
                            if (s.generation != std::numeric_limits<uint32_t>::max() && free_slot < 0) {
                                free_slot = static_cast<int>(i);
                            }
                            continue;
                        }
                        return m5::stl::make_unexpected(error::error_t::BUSY);
                    }
                    if (!(s.binding == binding)) {
                        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
                    }
                    auto valid = validate(live);
                    if (!valid.has_value()) {
                        return m5::stl::make_unexpected(valid.error());
                    }
                    return live;  // hit: the first backend wins
                }

                if (s.state == SlotState::Empty) {
                    if (s.generation != std::numeric_limits<uint32_t>::max() && free_slot < 0) {
                        free_slot = static_cast<int>(i);
                    }
                    continue;
                }

                // A non-matching ordinary local entry can be reclaimed once
                // its owner is gone. Externally-backed entries retain their
                // tombstone until lifecycle close is certain.
                if (s.state == SlotState::Live && !s.destructor_managed && s.bus.expired() &&
                    (!s.lifecycle || s.lifecycle->state() == BusLifecycle::State::Closed)) {
                    clearSlot(s);
                    if (s.generation != std::numeric_limits<uint32_t>::max() && free_slot < 0) {
                        free_slot = static_cast<int>(i);
                    }
                }
            }
            if (free_slot < 0) {
                return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
            }
            Slot& s = _slots[static_cast<size_t>(free_slot)];
            ++s.generation;
            s.key       = key;
            s.binding   = binding;
            s.state     = SlotState::Constructing;
            reservation = {static_cast<uint16_t>(free_slot), 0, s.generation};
        }

        ReservationRollback rollback{*this, reservation, key};
        auto made = make();

        Guard guard{_mutex};
        Slot* s = reservedSlot(reservation, key, SlotState::Constructing);
        if (s == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        if (!made.has_value()) {
            const auto err = made.error();
            clearSlot(*s);
            rollback.dismiss();
            return m5::stl::make_unexpected(err);
        }
        if (!made.value()) {
            clearSlot(*s);
            rollback.dismiss();
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        auto lifecycle = made.value()->lifecycleHandle();
        const bool destructor_managed =
            made.value()->bindRegistryRegistration(*this, key, reservation.slot, reservation.generation);
        // weak_ptr expires before the object's destructor begins. Publishing a
        // bus with neither a destructor callback nor an external lifecycle
        // tombstone would therefore permit same-resource recreation during
        // teardown. Refuse that unsafe extension shape.
        if (!destructor_managed && (!lifecycle || lifecycle->state() == BusLifecycle::State::Closed)) {
            clearSlot(*s);
            rollback.dismiss();
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        made.value()->markRegistryBound();
        s->destructor_managed = destructor_managed;
        s->bus                = made.value();  // stored as weak_ptr
        s->expected           = made.value().get();
        s->lifecycle          = std::move(lifecycle);
        s->state              = SlotState::Live;
        rollback.dismiss();
        return made.value();
    }

    /*!
      @brief Invoke `fn(shared_ptr<IBus>)` for every live bus of `kind`.

      Snapshots the matching live buses under the registry mutex (locking each
      `weak_ptr` to a strong `shared_ptr`), then calls `fn` for each OUTSIDE
      the lock -- so `fn` may safely take a bus's own lock (e.g. swap its
      backend) or otherwise act on the bus without risking a lock-order issue
      against the registry. Each bus is kept alive for the duration of its
      `fn` call. Used by the commit-time resolver to walk the live buses of a
      kind. The snapshot is bounded by `kCapacity`.
     */
    template <class Fn>
    void forEachLive(types::bus_kind_t kind, Fn&& fn)
    {
        std::shared_ptr<IBus> snapshot[kCapacity];
        size_t count = 0;
        {
            Guard guard{_mutex};
            for (size_t i = 0; i < kCapacity; ++i) {
                Slot& s = _slots[i];
                if (s.key.kind != kind) {
                    continue;
                }
                if (s.state != SlotState::Live) {
                    continue;
                }
                if (auto live = s.bus.lock()) {
                    snapshot[count++] = std::move(live);
                }
            }
        }
        for (size_t i = 0; i < count; ++i) {
            fn(snapshot[i]);
        }
    }

    /*!
      @brief Find a live bus by (kind, id) without creating one.

      Returns a strong `shared_ptr` if a live bus matching (kind, id)
      exists in the registry, or `nullptr` if no match is found. Used by
      `closeBus` in RemoteBackend to retrieve the proxy object and read
      back its remote bus_id before clearing the slot.
     */
    std::shared_ptr<IBus> findByIdentity(const ResourceKey& key) const
    {
        if (!key.isValid()) {
            return nullptr;
        }
        Guard guard{_mutex};
        for (size_t i = 0; i < kCapacity; ++i) {
            const Slot& s = _slots[i];
            if (s.key == key) {
                if (s.state != SlotState::Live) {
                    return nullptr;
                }
                return s.bus.lock();
            }
        }
        return nullptr;
    }

    result_t<ReleaseTicket> beginRelease(const ResourceKey& key, const std::shared_ptr<IBus>& expected)
    {
        if (!key.isValid() || !expected) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        Guard guard{_mutex};
        for (size_t i = 0; i < kCapacity; ++i) {
            Slot& s = _slots[i];
            if (s.key == key) {
                if (s.state != SlotState::Live && s.state != SlotState::Quarantined) {
                    return m5::stl::make_unexpected(error::error_t::BUSY);
                }
                const bool retrying_quarantine = s.state == SlotState::Quarantined;
                auto live                      = s.bus.lock();
                if (!live || live.get() != expected.get()) {
                    return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
                }
                // `live` is the one temporary owner created by this check.
                // Anything beyond it and `expected` is an external co-owner.
                if (live.use_count() != 2) {
                    return m5::stl::make_unexpected(error::error_t::BUSY);
                }
                s.state = SlotState::Closing;
                return ReleaseTicket{
                    {static_cast<uint16_t>(i), 0, s.generation}, key, expected.get(), retrying_quarantine};
            }
        }
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    result_t<void> commitRelease(const ReleaseTicket& ticket)
    {
        Guard guard{_mutex};
        Slot* s = releaseSlot(ticket);
        if (s == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        clearSlot(*s);
        return {};
    }

    result_t<void> cancelRelease(const ReleaseTicket& ticket)
    {
        Guard guard{_mutex};
        Slot* s = releaseSlot(ticket);
        if (s == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        s->state = ticket.retrying_quarantine ? SlotState::Quarantined : SlotState::Live;
        return {};
    }

    result_t<void> quarantineRelease(const ReleaseTicket& ticket)
    {
        Guard guard{_mutex};
        Slot* s = releaseSlot(ticket);
        if (s == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        s->state = SlotState::Quarantined;
        return {};
    }

    bool beginAbandon(RegistryEntryToken token, const ResourceKey& key, const IBus* expected)
    {
        if (!token.valid() || !key.isValid() || expected == nullptr || token.slot >= kCapacity) {
            return false;
        }
        Guard guard{_mutex};
        Slot& s = _slots[token.slot];
        // A destructor-managed local bus may be dropped after an explicit
        // partial close without the caller retrying. Its destructor gets one
        // final teardown attempt; reserve both Live and Quarantined slots so
        // confirmed success can reclaim the identity. External-lifecycle
        // tombstones have destructor_managed=false and remain quarantined.
        if (s.generation != token.generation || (s.state != SlotState::Live && s.state != SlotState::Quarantined) ||
            !(s.key == key) || s.expected != expected || !s.destructor_managed) {
            return false;
        }
        s.state = SlotState::Closing;
        return true;
    }

    void finishAbandon(RegistryEntryToken token, const ResourceKey& key, const IBus* expected, bool success)
    {
        if (!token.valid() || !key.isValid() || expected == nullptr || token.slot >= kCapacity) {
            return;
        }
        Guard guard{_mutex};
        Slot& s = _slots[token.slot];
        if (s.generation != token.generation || s.state != SlotState::Closing || !(s.key == key) ||
            s.expected != expected || !s.destructor_managed) {
            return;
        }
        if (success) {
            clearSlot(s);
        } else {
            s.bus.reset();
            s.expected = nullptr;
            s.state    = SlotState::Quarantined;
        }
    }

    /*!
      @brief Number of capacity slots occupied by a live bus or tombstone.

      A Releasing/Quarantined external lifecycle continues to consume its
      slot after the bus weak_ptr expires. Closed tombstones and ordinary
      expired weak slots are reported free even before lazy reclamation.
     */
    size_t liveCount(void) const
    {
        Guard guard{_mutex};  // read _slots under the same lock writers hold
        size_t n = 0;
        for (size_t i = 0; i < kCapacity; ++i) {
            const Slot& s = _slots[i];
            if (s.state != SlotState::Empty &&
                (s.state != SlotState::Live || s.destructor_managed || !s.bus.expired() ||
                 (s.lifecycle && s.lifecycle->state() != BusLifecycle::State::Closed))) {
                ++n;
            }
        }
        return n;
    }

private:
    friend struct BusRegistryTestAccess;

    struct Slot {
        std::weak_ptr<IBus> bus;
        std::shared_ptr<BusLifecycle> lifecycle;
        ResourceKey key;
        BindingDescriptor binding;
        const IBus* expected    = nullptr;
        uint32_t generation     = 0;
        SlotState state         = SlotState::Empty;
        bool destructor_managed = false;
    };

    static void clearSlot(Slot& s)
    {
        s.bus.reset();
        s.lifecycle.reset();
        s.key                = {};
        s.binding            = {};
        s.expected           = nullptr;
        s.state              = SlotState::Empty;
        s.destructor_managed = false;
    }

    class ReservationRollback {
    public:
        ReservationRollback(BusRegistry& registry, RegistryEntryToken token, const ResourceKey& key)
            : _registry{&registry}, _token{token}, _key{key}
        {
        }
        ~ReservationRollback()
        {
            if (_registry != nullptr) {
                _registry->rollbackReservation(_token, _key);
            }
        }
        void dismiss(void)
        {
            _registry = nullptr;
        }

    private:
        BusRegistry* _registry;
        RegistryEntryToken _token;
        ResourceKey _key;
    };

    void rollbackReservation(RegistryEntryToken token, const ResourceKey& key)
    {
        Guard guard{_mutex};
        if (auto* s = reservedSlot(token, key, SlotState::Constructing)) {
            clearSlot(*s);
        }
    }

    Slot* reservedSlot(RegistryEntryToken token, const ResourceKey& key, SlotState state)
    {
        if (!token.valid() || token.slot >= kCapacity) {
            return nullptr;
        }
        Slot& s = _slots[token.slot];
        if (s.generation != token.generation || s.state != state || !(s.key == key)) {
            return nullptr;
        }
        return &s;
    }

    Slot* releaseSlot(const ReleaseTicket& ticket)
    {
        if (!ticket.valid()) {
            return nullptr;
        }
        Slot& s = _slots[ticket.entry.slot];
        if (s.generation != ticket.entry.generation || s.state != SlotState::Closing || !(s.key == ticket.key)) {
            return nullptr;
        }
        auto live = s.bus.lock();
        if (!live || live.get() != ticket.expected) {
            return nullptr;
        }
        return &s;
    }

    using Guard = runtime::MutexGuard;

    Slot _slots[kCapacity];
    mutable runtime::Mutex _mutex;  // mutable: const observers (liveCount) take it too
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_REGISTRY_HPP_
