// SPDX-License-Identifier: MIT
#ifndef M5_HAL_GPIO_GROUP_HPP_
#define M5_HAL_GPIO_GROUP_HPP_

// GPIOGroup — bundles multiple IGPIO instances and resolves a global
// `gpio_number_t` to a concrete `Pin`. Role split: IGPIO owns the local
// pin space, GPIOGroup is the global resolver.
// Authoritative spec: spec/design/gpio.md §GPIOGroup.

#include "../assert.hpp"
#include "../error.hpp"
#include "../runtime/runtime.hpp"
#include "../service/service.hpp"
#include "../types.hpp"
#include "gpio.hpp"
#include "port.hpp"

#include <M5Utility.hpp>

#include <atomic>
#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v2::gpio {

/*!
  @brief Slot-based registry that resolves a global `gpio_number_t`
         to a `Pin`.

  Registration happens during startup only; afterwards the registry
  is treated as immutable and read-only (no locking).
 */
class GPIOGroup : private service::IService {
public:
    /*! @brief Upper bound on the slot key space (bits 14-8 of
        `gpio_number_t`, values 0..127). */
    static constexpr size_t kSlotCount = 128;
    /*! @brief Backing-storage capacity (maximum number of IGPIOs
        registered at once). Slot keys are sparse (0..127) but the
        storage is dense. */
    static constexpr size_t kMaxEntries = 16;

    /*! @brief Default poll period used by `setWatchSink` when the
        caller passes 0. */
    static constexpr uint32_t kDefaultWatchIntervalUs = 1000;

    enum class Edge : uint8_t {
        Rising,
        Falling,
    };

    /*!
      @brief Sink contract: called on the service-runner context (poll-detected
             edges) or on the thread pumping remote frames (push-fed edges).

      Must be short and non-blocking. May call watch()/unwatch()/
      setWatchSink() re-entrantly (a self-unregister does not wait, R6).
      Must NOT call notifyPinStateChanged(), runOnce(), or start bus /
      remote transactions. A push callback runs inside the remote session
      lease, so it also must not destroy or reconnect the owning Hal/session.
      Dropping ordinary proxy references is allowed.
     */
    using WatchSink = void (*)(void* ctx, types::gpio_number_t pin, bool level, Edge edge);

    GPIOGroup() noexcept;

    /*! @brief Load the MCU GPIO into slot 0 (or build an empty group
        when `mcu_gpio == nullptr`). */
    explicit GPIOGroup(const IGPIO* mcu_gpio) noexcept;

    GPIOGroup(const GPIOGroup&)            = delete;
    GPIOGroup& operator=(const GPIOGroup&) = delete;
    GPIOGroup(GPIOGroup&&)                 = delete;
    GPIOGroup& operator=(GPIOGroup&&)      = delete;

    /*!
      @brief Register an `IGPIO` at `slot` (startup-only operation).

      Rejected (returns `INVALID_ARGUMENT`) when: `gpio == nullptr`,
      `slot >= kSlotCount`, pin count not in `[1, 256]`, the slot is
      already in use, or storage is full. See spec §addGPIO for the
      full rule set.
     */
    [[nodiscard]] result_t<void> addGPIO(const IGPIO* gpio, types::gpio_slot_t slot);

    /*!
      @brief Unregister the IGPIO at `slot` (startup-only operation).

      Rejected when `slot >= kSlotCount` or the slot is not currently
      registered. The hole is filled by moving the last entry over it
      (swap-and-pop; the entry order is not preserved).
     */
    [[nodiscard]] result_t<void> removeGPIO(types::gpio_slot_t slot);

    void bindServiceRunner(service::ServiceRunner* runner);

    /*!
      @brief Register / replace / unregister the group-wide watch sink
             (`sink == nullptr` unregisters).

      Also sets the group-wide poll period (`poll_interval_us == 0`
      rounds to `kDefaultWatchIntervalUs`). Registering, replacing and
      unregistering are serialized against each other through an
      internal mutex plus an in-flight wait: once this call returns,
      the previous sink is guaranteed not to be invoked again (a
      self-unregister called from inside the sink itself does not
      wait on its own in-flight callback, R6). Concurrent calls to
      `setWatchSink` from different threads are NOT supported (result
      undefined); `watch()`/`unwatch()` may run concurrently with it
      from any thread.

      Failure (`OUT_OF_RESOURCE`, service-runner table full) leaves a
      deterministic state: no sink registered and no poll service — a
      failed REPLACE does not restore the previous sink. Callers may
      retry.
     */
    [[nodiscard]] result_t<void> setWatchSink(WatchSink sink, void* ctx,
                                              uint32_t poll_interval_us = kDefaultWatchIntervalUs);

    /*!
      @brief Enable watching of `gpio_num`. Lock-free (a single RMW on
             an atomic port mask); safe to call from any thread
             concurrently with other watch()/unwatch() calls.

      The current level is seeded into the edge-detection shadow
      before the mask bit is published, so a transition that
      straddles the call may be folded into the very next observed
      edge, but no spurious edge is reported for the call itself.
      Rejected with `INVALID_ARGUMENT` when `gpio_num` is invalid /
      denied, or `IGPIO::locatePin()` maps it outside logical port
      0/1.
     */
    [[nodiscard]] result_t<void> watch(types::gpio_number_t gpio_num);

    /*!
      @brief Disable watching of `gpio_num`. Lock-free, idempotent
             (unwatching an already-unwatched pin still succeeds).

      Does not wait for an in-flight callback for this pin to finish:
      one more event for `gpio_num` may still be delivered right
      after `unwatch` returns (only sink teardown via `setWatchSink`
      waits for in-flight completion).
     */
    result_t<void> unwatch(types::gpio_number_t gpio_num);

    /*! @brief Clear every watch mask and unregister the sink (teardown). */
    void clearWatchers();

    /*!
      @brief Feed a push-sourced pin-state edge (remote GPIO event path).

      Contract: only valid for pins whose owning `IGPIO::hasPushEvents()`
      reports true. Calling it for a poll-fed pin breaks the
      single-source rule the shadow-XOR edge detection relies on
      (not asserted; documented UB).
     */
    [[nodiscard]] result_t<void> notifyPinStateChanged(types::gpio_number_t gpio_num, bool level);

    const IGPIO* getGPIO(types::gpio_slot_t slot) const;

    bool hasGPIO(types::gpio_slot_t slot) const;

    /*!
      @brief Set a per-port deny bitmask for the IGPIO at `slot`.

      Bits set in `mask` prevent access to the corresponding pin
      (e.g. Flash SPI pins on ESP32).  Multiple calls replace the
      mask for the same (slot, port_index) pair. `port_index` must be
      an existing IGPIO logical port inside GPIOGroup's supported
      port 0/1 window; pin-to-port mapping follows `IGPIO::locatePin()`.
     */
    result_t<void> setDenyMask(types::gpio_slot_t slot, uint8_t port_index, uint32_t mask);

    struct PortAccess {
        IPort* port;
        uint32_t deny_mask;
    };

    result_t<PortAccess> getPort(types::gpio_slot_t slot, uint8_t port_index) const;

    /*!
      @brief Return whether `gpio_num` is in range (slot registered,
             local pin within the IGPIO's pin count, and not denied).
     */
    bool isValid(types::gpio_number_t gpio_num) const;

    /*!
      @brief Checked resolution. Invalid or denied input recovers
             through the `expected` error path.
     */
    [[nodiscard]] result_t<Pin> tryGetPin(types::gpio_number_t gpio_num) const;

    /*!
      @brief Unchecked fast-path resolution. Contract violations
             assert / are UB; use `tryGetPin` when the caller needs
             to recover.
     */
    Pin getPin(types::gpio_number_t gpio_num) const;

private:
    static constexpr size_t kMaxPortsPerEntry   = 2;
    static constexpr size_t kMaxSinkDispatchers = 4;

    struct Entry {
        const IGPIO* gpio                     = nullptr;
        types::gpio_slot_t slot               = 0;
        uint32_t deny_mask[kMaxPortsPerEntry] = {};
        bool push_events                      = false;  // cached gpio->hasPushEvents()
    };

    const Entry* _find(types::gpio_slot_t slot) const;
    Entry* _findMut(types::gpio_slot_t slot);

    // _find's index form (kMaxEntries when not found).
    size_t entryIndexOf(types::gpio_slot_t slot) const;

    // Zeroes every watch-state array (both ctors call this instead of
    // relying on aggregate `= {}` — see the C++17 note below).
    void initWatchState();

    static service::fast_tick_t usToTicks(uint32_t us);

    // -- watch-sink dispatch bookkeeping (see spec/design/gpio.md
    // §thread safety / lifetime for the full contract) --------------
    bool enterSinkDispatch(WatchSink& cb, void*& ctx);  // false = no sink registered
    void exitSinkDispatch();
    void waitSinkIdle();  // does not wait if the calling task is itself mid-dispatch (R6)

    // RAII over enterSinkDispatch()/exitSinkDispatch() so a throwing
    // sink (host builds) cannot leak the in-flight count or the
    // dispatcher task slot (same rationale as ServiceRunner's PassEnd).
    struct SinkDispatchScope {
        explicit SinkDispatchScope(GPIOGroup& group) : g{group}
        {
            active = g.enterSinkDispatch(cb, ctx);
        }
        ~SinkDispatchScope()
        {
            if (active) {
                g.exitSinkDispatch();
            }
        }
        SinkDispatchScope(const SinkDispatchScope&)            = delete;
        SinkDispatchScope& operator=(const SinkDispatchScope&) = delete;

        GPIOGroup& g;
        WatchSink cb = nullptr;
        void* ctx    = nullptr;
        bool active  = false;
    };

    service::ServicePoll serviceImpl(const service::ServiceContext& ctx) override;

    Entry _entries[kMaxEntries] = {};
    size_t _count               = 0;

    service::ServiceRunner* _service_runner = nullptr;

    // Watch state: parallel to _entries (index-aligned). Kept outside
    // Entry so Entry stays trivially copyable for swap-and-pop.
    //
    // C++17 note: std::atomic's default constructor leaves the value
    // indeterminate (P0883 — value-initialization to 0 is C++20 only),
    // so these arrays are NOT relied on for zero-init via `= {}`; both
    // constructors call initWatchState() to store 0 explicitly.
    std::atomic<uint32_t> _watch_mask[kMaxEntries][kMaxPortsPerEntry];
    std::atomic<uint32_t> _watch_shadow[kMaxEntries][kMaxPortsPerEntry];

    runtime::Mutex _watch_mutex;                           // sink fields + registration flag
    WatchSink _watch_sink                      = nullptr;  // under _watch_mutex
    void* _watch_sink_ctx                      = nullptr;  // under _watch_mutex
    bool _watch_service_registered             = false;    // under _watch_mutex
    service::fast_tick_t _watch_interval_ticks = 0;        // under _watch_mutex (poll side reads it once per pass)
    service::fast_tick_t _watch_next_tick      = 0;        // runner-context private (virtual axis)
    service::fast_tick_t _watch_svc_now        = 0;        // runner-context private virtual clock
    std::atomic<uint32_t> _sink_inflight{0};
    std::atomic<void*> _sink_dispatch_tasks[kMaxSinkDispatchers];
};

}  // namespace m5::hal::v2::gpio

#endif
