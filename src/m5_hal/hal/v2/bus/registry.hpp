// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_REGISTRY_HPP_
#define M5_HAL_BUS_REGISTRY_HPP_

#include "./bus.hpp"  // IBus, runtime::Mutex, types, error, result_t, M5Utility

#include <initializer_list>
#include <memory>
#include <utility>

/*!
  @namespace m5::hal::v2::bus
  @brief All-kind weak interning registry for buses.
 */
namespace m5::hal::v2::bus {

/*!
  @brief Physical-wiring identity tag (pins only).

  Two configs that name the same pins denote the same physical bus and
  intern to one instance (identity = the wiring, NOT which
  backend / controller drives it, NOT the per-accessor frequency). The
  roles are positional and fixed, so no order normalization is needed --
  swapping pins is a different bus. Each kind puts its two essential
  wires in pins[0]/pins[1] (so `valid()` is kind-agnostic) and any extra
  identifying wires after them: I2C = {SCL, SDA}; SPI = {CLK, MOSI, MISO}
  (the 3-wire core; quad/octal data lines and DC are NOT identity -- the
  same core wires are the same physical bus); UART = {TX, RX}; I2S =
  {BCLK, WS, DOUT, DIN}. `kMaxPins` covers the widest of these.

  Limiting identity to the core wires is deliberate: including the extra
  bus-level pins a backend may also drive (SPI quad data, UART RTS/CTS, I2S
  MCLK) would split the same core wiring used in two modes into two buses
  with two locks -- breaking the "one physical bus, one lock" guarantee. The
  trade-off is that those extra pins are FIRST-CONFIG-WINS: a later acquire
  of the same core wiring shares the first bus and ignores any differing
  extra-pin config (set them on the first acquire).
 */
struct IdentityKey {
    static constexpr size_t kMaxPins = 6;
    // Keep the initializer element count equal to kMaxPins (the -1 sentinel
    // must reach every slot; a short brace list would zero the rest, and 0
    // is a valid pin number).
    types::gpio_number_t pins[kMaxPins] = {-1, -1, -1, -1, -1, -1};

    /*!
      @brief Build a key from a kind's identity pins in role order.

      The single shared identity projection: each kind passes its essential
      wires first (pins[0]/pins[1], required by `valid()`) then any extra
      identifying wires, exactly as the per-kind layout doc above. Unset
      trailing roles stay at the `-1` sentinel (e.g. an I2S RX-only DOUT), so
      "wire absent" remains observable in the key (it must, for interning).
      A list longer than `kMaxPins` is clamped (a kind never exceeds it).
     */
    static IdentityKey fromPins(std::initializer_list<types::gpio_number_t> role_pins)
    {
        IdentityKey key;
        size_t i = 0;
        for (types::gpio_number_t pin : role_pins) {
            if (i >= kMaxPins) {
                break;
            }
            key.pins[i++] = pin;
        }
        return key;
    }

    constexpr bool valid(void) const
    {
        return pins[0] >= 0 && pins[1] >= 0;
    }
    bool operator==(const IdentityKey& other) const
    {
        for (size_t i = 0; i < kMaxPins; ++i) {
            if (pins[i] != other.pins[i]) {
                return false;
            }
        }
        return true;
    }
};

/*!
  @brief All-kind weak registry: interns buses by (kind, identity).

  One physical wiring maps to one shared instance, so a board-support
  layer and user code that name the same pins share a single bus (and its
  single lock) -- the correctness requirement the registry enforces. The registry
  holds `weak_ptr`, so a bus is released (its `Bus` dtor runs, freeing the
  backend) once the last `shared_ptr` holder drops it. For an externally
  backed bus whose natural release is still pending or has failed, the slot
  keeps its lifecycle tombstone and identity until close; reacquisition then
  returns `BUSY` rather than aliasing an uncertain peer resource. A single
  mutex serializes `acquireOrFind` so two concurrent acquires
  of one identity cannot create two instances. Lookups never sit on a
  transfer hot path (accessors hold the bus directly), so the lock is free.

  Per-kind access goes through a typed view (e.g. `i2c::BusView`, exposed
  as `M5_Hal.I2C`) that validates the kind and hands back the kind-typed
  `shared_ptr`. The total live-bus budget (`kCapacity`) is shared across
  all kinds; it is a RAM backstop, distinct from any per-kind hardware
  resource pool.
 */
class BusRegistry {
    enum class SlotState : uint8_t { Live, Releasing };

public:
    /*! @brief Total live buses across all kinds (RAM budget). */
    static constexpr size_t kCapacity = 16;

    struct ReleaseTicket {
        size_t slot            = kCapacity;
        types::bus_kind_t kind = types::bus_kind_t::Unknown;
        IdentityKey id;
        const IBus* expected = nullptr;

        ReleaseTicket(void) = default;
        ReleaseTicket(size_t slot_index, types::bus_kind_t bus_kind, const IdentityKey& identity,
                      const IBus* expected_bus)
            : slot{slot_index}, kind{bus_kind}, id{identity}, expected{expected_bus}
        {
        }

        bool valid(void) const
        {
            return slot < kCapacity && expected != nullptr;
        }
    };

    BusRegistry(void)                          = default;
    BusRegistry(const BusRegistry&)            = delete;
    BusRegistry& operator=(const BusRegistry&) = delete;

    /*!
      @brief Return the bus for (kind, id), creating it via `make` on a miss.

      Atomic under the registry mutex. A hit returns the existing instance,
      so a second acquire of the same wiring shares it -- the FIRST backend
      choice wins and a later differing config is ignored (a managed reassign
      changes a live bus's backend instead). A miss calls `make` and interns
      the result. `make` is `() -> result_t<shared_ptr<IBus>>`; its error is
      propagated WITHOUT interning (a failed bus is never registered).
      Returns `OUT_OF_RESOURCE` when all `kCapacity` slots hold live buses.
     */
    template <class MakeFn>
    result_t<std::shared_ptr<IBus>> acquireOrFind(types::bus_kind_t kind, const IdentityKey& id, MakeFn&& make)
    {
        return acquireOrFind(
            kind, id, [](const std::shared_ptr<IBus>&) -> result_t<void> { return {}; }, std::forward<MakeFn>(make));
    }

    /*!
      @brief `acquireOrFind` with an atomic live-hit validator.

      `validate(live)` runs under the same registry critical section that
      identifies the hit.  This is for configuration compatibility checks
      that must not race a release/reacquire of the same identity.
     */
    template <class ValidateFn, class MakeFn>
    result_t<std::shared_ptr<IBus>> acquireOrFind(types::bus_kind_t kind, const IdentityKey& id, ValidateFn&& validate,
                                                  MakeFn&& make)
    {
        Guard guard{_mutex};
        int free_slot = -1;
        for (size_t i = 0; i < kCapacity; ++i) {
            Slot& s = _slots[i];
            if (auto live = s.bus.lock()) {
                if (s.kind == kind && s.id == id) {
                    if (s.state == SlotState::Releasing) {
                        return m5::stl::make_unexpected(error::error_t::BUSY);
                    }
                    auto valid = validate(live);
                    if (!valid.has_value()) {
                        return m5::stl::make_unexpected(valid.error());
                    }
                    return live;  // hit: the first backend wins
                }
                continue;
            }
            // An externally-backed bus may already have lost its last strong
            // owner while its destructor is still releasing the peer object.
            // Keep the identity tombstoned until that lifecycle is Closed.
            if (s.lifecycle && s.lifecycle->state() != BusLifecycle::State::Closed) {
                if (s.kind == kind && s.id == id) {
                    return m5::stl::make_unexpected(error::error_t::BUSY);
                }
                continue;
            }
            // Expired (or never used): reclaim lazily and remember as free.
            s.lifecycle.reset();
            s.kind  = types::bus_kind_t::Unknown;
            s.state = SlotState::Live;
            if (free_slot < 0) {
                free_slot = static_cast<int>(i);
            }
        }
        if (free_slot < 0) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        auto made = make();
        if (!made.has_value()) {
            return m5::stl::make_unexpected(made.error());
        }
        Slot& s     = _slots[static_cast<size_t>(free_slot)];
        s.bus       = made.value();  // stored as weak_ptr
        s.lifecycle = made.value()->lifecycleHandle();
        s.kind      = kind;
        s.id        = id;
        s.state     = SlotState::Live;
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
                if (s.kind != kind) {
                    continue;
                }
                if (s.state == SlotState::Releasing) {
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
      `releaseBus` in RemoteBackend to retrieve the proxy object and read
      back its remote bus_id before clearing the slot.
     */
    std::shared_ptr<IBus> findByIdentity(types::bus_kind_t kind, const IdentityKey& id) const
    {
        Guard guard{_mutex};
        for (size_t i = 0; i < kCapacity; ++i) {
            const Slot& s = _slots[i];
            if (s.kind == kind && s.id == id) {
                if (s.state == SlotState::Releasing) {
                    return nullptr;
                }
                return s.bus.lock();
            }
        }
        return nullptr;
    }

    result_t<ReleaseTicket> beginRelease(types::bus_kind_t kind, const IdentityKey& id,
                                         const std::shared_ptr<IBus>& expected)
    {
        if (!expected) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
        Guard guard{_mutex};
        for (size_t i = 0; i < kCapacity; ++i) {
            Slot& s = _slots[i];
            if (s.kind == kind && s.id == id) {
                if (s.state == SlotState::Releasing) {
                    return m5::stl::make_unexpected(error::error_t::BUSY);
                }
                auto live = s.bus.lock();
                if (!live || live.get() != expected.get()) {
                    return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
                }
                // `live` is the one temporary owner created by this check.
                // Anything beyond it and `expected` is an external co-owner.
                if (live.use_count() != 2) {
                    return m5::stl::make_unexpected(error::error_t::BUSY);
                }
                s.state = SlotState::Releasing;
                return ReleaseTicket{i, kind, id, expected.get()};
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
        s->bus.reset();
        s->lifecycle.reset();
        s->kind  = types::bus_kind_t::Unknown;
        s->id    = IdentityKey{};
        s->state = SlotState::Live;
        return {};
    }

    result_t<void> cancelRelease(const ReleaseTicket& ticket)
    {
        Guard guard{_mutex};
        Slot* s = releaseSlot(ticket);
        if (s == nullptr) {
            return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
        }
        s->state = SlotState::Live;
        return {};
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
            if (!s.bus.expired() || (s.lifecycle && s.lifecycle->state() != BusLifecycle::State::Closed)) {
                ++n;
            }
        }
        return n;
    }

private:
    struct Slot {
        std::weak_ptr<IBus> bus;
        std::shared_ptr<BusLifecycle> lifecycle;
        types::bus_kind_t kind = types::bus_kind_t::Unknown;
        IdentityKey id;
        SlotState state = SlotState::Live;
    };

    Slot* releaseSlot(const ReleaseTicket& ticket)
    {
        if (!ticket.valid()) {
            return nullptr;
        }
        Slot& s = _slots[ticket.slot];
        if (s.state != SlotState::Releasing || s.kind != ticket.kind || !(s.id == ticket.id)) {
            return nullptr;
        }
        auto live = s.bus.lock();
        if (!live || live.get() != ticket.expected) {
            return nullptr;
        }
        return &s;
    }

    // Minimal RAII guard over runtime::Mutex (bool lock(timeout) / void unlock()).
    struct Guard {
        runtime::Mutex& m;
        explicit Guard(runtime::Mutex& mtx) : m{mtx}
        {
            (void)m.lock(types::TIMEOUT_FOREVER);
        }
        ~Guard(void)
        {
            m.unlock();
        }
        Guard(const Guard&)            = delete;
        Guard& operator=(const Guard&) = delete;
    };

    Slot _slots[kCapacity];
    mutable runtime::Mutex _mutex;  // mutable: const observers (liveCount) take it too
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_REGISTRY_HPP_
