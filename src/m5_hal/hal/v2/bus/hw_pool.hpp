// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_HW_POOL_HPP_
#define M5_HAL_BUS_HW_POOL_HPP_

#include "../runtime/runtime.hpp"
#include "../types.hpp"

#include <cstdint>

/*!
  @namespace m5::hal::v2::bus
  @brief Per-kind hardware controller lease pool.
 */
namespace m5::hal::v2::bus {

/*!
  @brief Fixed-size lease pool for a kind's hardware controllers.

  The "silicon budget", distinct from the registry's RAM budget (total live
  bus count, `BusRegistry::kCapacity`). A bus driven in hardware leases one
  controller index here; a software (bit-bang) bus leases nothing. The intent
  resolver (`commitBuses`) hands out leases highest-priority-first and the
  pool just tracks which indices are free with a bitmask. Lease ids are
  abstract `0 .. capacity-1`; a variant maps an index to its concrete
  peripheral (an ESP-IDF `i2c_port`, an `spi_host_device_t`, ...).

  One pool exists per kind (owned by that kind's `BusView`). Capacity comes
  from the build's hardware variant (e.g. `SOC_I2C_NUM`). A software-only or
  host build has capacity 0, so every `RequireHardware` fails and everything
  lands in software -- exactly the native-test condition.

  Each operation is mutex-guarded for atomicity; the resolver is expected to
  drive a whole bin-packing pass from one (task) context. Capacity is bounded
  by the bitmask width (`kMaxControllers`).
 */
class HwControllerPool {
public:
    static constexpr int8_t kNone            = -1;
    static constexpr uint8_t kMaxControllers = 32;  // bounded by the mask width

    explicit HwControllerPool(uint8_t capacity = 0) : _capacity{capacity > kMaxControllers ? kMaxControllers : capacity}
    {
    }

    HwControllerPool(const HwControllerPool&)            = delete;
    HwControllerPool& operator=(const HwControllerPool&) = delete;

    /*! @brief Number of controllers this pool can hand out. */
    uint8_t capacity(void) const
    {
        return _capacity;
    }

    /*! @brief Lease the lowest free controller index, or `kNone` when full. */
    int8_t acquire(void)
    {
        Guard guard{_mutex};
        for (uint8_t i = 0; i < _capacity; ++i) {
            const uint32_t bit = 1u << i;
            if ((_in_use_mask & bit) == 0) {
                _in_use_mask |= bit;
                return static_cast<int8_t>(i);
            }
        }
        return kNone;
    }

    /*!
      @brief Lease a SPECIFIC controller index (the `PinController` escape hatch).

      Returns false when the index is out of range or already leased.
     */
    bool acquireSpecific(int8_t controller)
    {
        if (controller < 0 || static_cast<uint8_t>(controller) >= _capacity) {
            return false;
        }
        Guard guard{_mutex};
        const uint32_t bit = 1u << static_cast<uint8_t>(controller);
        if (_in_use_mask & bit) {
            return false;
        }
        _in_use_mask |= bit;
        return true;
    }

    /*!
      @brief Return a leased controller to the pool.

      A no-op for an out-of-range index or one that was not leased, so a
      double release is harmless. Also a no-op for an externally-claimed
      controller: each ownership class releases only through its own path
      (`releaseExternal` for claims), so a stray ordinary release can never
      free a controller out from under its external holder.
     */
    void release(int8_t controller)
    {
        if (controller < 0 || static_cast<uint8_t>(controller) >= _capacity) {
            return;
        }
        Guard guard{_mutex};
        const uint32_t bit = 1u << static_cast<uint8_t>(controller);
        _in_use_mask &= ~(bit & ~_external_mask);
    }

    /*! @brief Whether `controller` is currently leased. */
    bool isLeased(int8_t controller) const
    {
        if (controller < 0 || static_cast<uint8_t>(controller) >= _capacity) {
            return false;
        }
        Guard guard{_mutex};  // read _in_use_mask under the same lock writers hold
        return (_in_use_mask & (1u << static_cast<uint8_t>(controller))) != 0;
    }

    /*!
      @brief Claim a controller for a caller OUTSIDE the intent resolver
             (e.g. a standalone slave that never goes through `commitBuses`).

      Marks `controller` both leased (so `acquire`/`acquireSpecific` skip it)
      and external (so `releaseAll()` -- the commit-time pool rebuild --
      leaves it alone instead of reclaiming it as a leak). Returns false when
      the index is out of range or already leased (by either an intent-driven
      bus or a prior external claim); a double claim of the same index fails.
     */
    bool claimExternal(int8_t controller)
    {
        if (controller < 0 || static_cast<uint8_t>(controller) >= _capacity) {
            return false;
        }
        Guard guard{_mutex};
        const uint32_t bit = 1u << static_cast<uint8_t>(controller);
        if (_in_use_mask & bit) {
            return false;
        }
        _in_use_mask |= bit;
        _external_mask |= bit;
        return true;
    }

    /*!
      @brief Return a controller claimed via `claimExternal`.

      Returns true when `controller` was externally claimed and is now free.
      Returns false -- leaving the pool untouched -- for an out-of-range
      index, an unleased one, or one held by an ORDINARY (resolver) lease:
      the check and the clear happen under one lock, so this can never free
      a controller belonging to the other ownership class, and the caller
      gets an authoritative answer (no separate isExternal pre-check needed).
      A double release simply returns false.
     */
    bool releaseExternal(int8_t controller)
    {
        if (controller < 0 || static_cast<uint8_t>(controller) >= _capacity) {
            return false;
        }
        Guard guard{_mutex};
        const uint32_t bit = 1u << static_cast<uint8_t>(controller);
        if ((_external_mask & bit) == 0) {
            return false;
        }
        _in_use_mask &= ~bit;
        _external_mask &= ~bit;
        return true;
    }

    /*! @brief Whether `controller` is currently held by an external claim. */
    bool isExternal(int8_t controller) const
    {
        if (controller < 0 || static_cast<uint8_t>(controller) >= _capacity) {
            return false;
        }
        Guard guard{_mutex};
        return (_external_mask & (1u << static_cast<uint8_t>(controller))) != 0;
    }

    /*! @brief Count of leased controllers (observation: tests / capacity checks). */
    uint8_t inUse(void) const
    {
        Guard guard{_mutex};  // read _in_use_mask under the same lock writers hold
        uint8_t n = 0;
        for (uint8_t i = 0; i < _capacity; ++i) {
            if (_in_use_mask & (1u << i)) {
                ++n;
            }
        }
        return n;
    }

    /*! @brief Free controllers remaining. */
    uint8_t available(void) const
    {
        return static_cast<uint8_t>(_capacity - inUse());
    }

    /*!
      @brief Drop all leases, EXCEPT externally-claimed controllers.

      Used by the commit-time resolver to rebuild the pool from the live
      hardware buses: a bus that expired (its last holder dropped) tore down
      its hardware in the backend dtor but could not return its lease, so the
      resolver clears the pool and re-establishes leases for the buses that
      are actually still live. An external claim (`claimExternal`) is not a
      resolver lease -- its holder is outside `commitBuses`' view entirely, so
      rebuilding from the live-bus list must not drop it (it would otherwise
      be handed back out from under the external holder on the next commit).
     */
    void releaseAll(void)
    {
        Guard guard{_mutex};
        _in_use_mask = _external_mask;
    }

private:
    // Minimal RAII guard over runtime::Mutex (matches registry.hpp).
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

    uint8_t _capacity     = 0;
    uint32_t _in_use_mask = 0;
    // Subset of _in_use_mask held by claimExternal (see releaseAll). Always
    // a subset: a bit is set here only alongside the matching _in_use_mask
    // bit, and cleared no later than it.
    uint32_t _external_mask = 0;
    mutable runtime::Mutex _mutex;  // mutable: const observers (isLeased/inUse) take it too
};

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_HW_POOL_HPP_
