// SPDX-License-Identifier: MIT
#ifndef M5_HAL_GPIO_GROUP_HPP_
#define M5_HAL_GPIO_GROUP_HPP_

// GPIOGroup — bundles multiple IGPIO instances and resolves a global
// `gpio_number_t` to a concrete `Pin`. Role split: IGPIO owns the local
// pin space, GPIOGroup is the global resolver.
// Authoritative spec: spec/design/gpio.md §GPIOGroup.

#include "../assert.hpp"
#include "../error.hpp"
#include "../service/service.hpp"
#include "../types.hpp"
#include "gpio.hpp"
#include "port.hpp"

#include <M5Utility.hpp>

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
    static constexpr size_t kMaxEntries     = 16;
    static constexpr size_t kMaxWatchers    = 16;
    static constexpr size_t kMaxWatchEvents = 32;

    using watch_id_t = uint16_t;

    enum class Edge : uint8_t {
        Rising,
        Falling,
        Change,
    };

    using WatchCallback = void (*)(void* ctx, types::gpio_number_t pin, bool level, Edge edge);

    struct WatchConfig {
        uint32_t poll_interval_us = 1000;
        uint32_t debounce_us      = 0;
    };

    GPIOGroup() noexcept = default;

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

    [[nodiscard]] result_t<watch_id_t> watch(types::gpio_number_t gpio_num, Edge edge, WatchCallback callback);

    [[nodiscard]] result_t<watch_id_t> watch(types::gpio_number_t gpio_num, Edge edge, WatchCallback callback,
                                             void* ctx);

    [[nodiscard]] result_t<watch_id_t> watch(types::gpio_number_t gpio_num, Edge edge, WatchCallback callback,
                                             void* ctx, const WatchConfig& cfg);

    bool unwatch(watch_id_t id);

    void clearWatchers();

    [[nodiscard]] result_t<void> notifyPinStateChanged(types::gpio_number_t gpio_num, bool level);

    const IGPIO* getGPIO(types::gpio_slot_t slot) const;

    bool hasGPIO(types::gpio_slot_t slot) const;

    /*!
      @brief Set a per-port deny bitmask for the IGPIO at `slot`.

      Bits set in `mask` prevent access to the corresponding pin
      (e.g. Flash SPI pins on ESP32).  Multiple calls replace the
      mask for the same (slot, port_index) pair.
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
    static constexpr size_t kMaxPortsPerEntry = 2;

    struct Entry {
        const IGPIO* gpio                     = nullptr;
        types::gpio_slot_t slot               = 0;
        uint32_t deny_mask[kMaxPortsPerEntry] = {};
    };

    const Entry* _find(types::gpio_slot_t slot) const;
    Entry* _findMut(types::gpio_slot_t slot);

    Entry _entries[kMaxEntries] = {};
    size_t _count               = 0;

    struct Watcher {
        types::gpio_number_t pin             = -1;
        Edge edge                            = Edge::Change;
        WatchCallback callback               = nullptr;
        void* ctx                            = nullptr;
        service::fast_tick_t poll_ticks      = 0;
        service::fast_tick_t debounce_ticks  = 0;
        service::fast_tick_t next_poll_tick  = 0;
        service::fast_tick_t last_event_tick = 0;
        uint8_t generation                   = 0;
        bool last_level                      = false;
        bool has_level                       = false;
        bool used                            = false;
    };

    struct WatchEvent {
        types::gpio_number_t pin = -1;
        Edge edge                = Edge::Change;
        bool level               = false;
        uint8_t watcher_index    = 0;
        uint8_t generation       = 0;
    };

    static watch_id_t makeWatchId(size_t index, uint8_t generation);

    static size_t watchIndex(watch_id_t id);

    static uint8_t watchGeneration(watch_id_t id);

    static service::fast_tick_t usToTicks(uint32_t us);

    static bool edgeMatches(Edge watch_edge, Edge observed);

    bool anyWatcherUsed() const;

    void ensureWatchService();

    void maybeRemoveWatchService();

    result_t<void> enqueueEvent(size_t watcher_index, Edge edge, bool level);

    result_t<void> observeWatcher(size_t index, bool level, service::fast_tick_t now);

    void dropQueuedEventsFor(size_t watcher_index, uint8_t generation);

    bool dispatchOneEvent();

    service::ServicePoll serviceImpl(const service::ServiceContext& ctx) override;

    service::ServiceRunner* _service_runner = nullptr;
    Watcher _watchers[kMaxWatchers]         = {};
    WatchEvent _events[kMaxWatchEvents]     = {};
    size_t _event_head                      = 0;
    size_t _event_tail                      = 0;
    size_t _event_count                     = 0;
    bool _watch_service_registered          = false;
};

}  // namespace m5::hal::v2::gpio

#endif
