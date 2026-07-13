// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_GPIO_GROUP_INL_
#define M5_HAL_HAL_V2_GPIO_GROUP_INL_

#include "group.hpp"

#include "../service/completion_gate.hpp"

namespace m5::hal::v2::gpio {

namespace group_detail {

// Minimal RAII guard over runtime::Mutex (matches bus/registry.hpp,
// bus/hw_pool.hpp: bool lock(timeout) / void unlock()).
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

}  // namespace group_detail

void GPIOGroup::initWatchState()
{
    for (size_t i = 0; i < kMaxEntries; ++i) {
        for (size_t p = 0; p < kMaxPortsPerEntry; ++p) {
            _watch_mask[i][p].store(0, std::memory_order_relaxed);
            _watch_shadow[i][p].store(0, std::memory_order_relaxed);
        }
    }
    for (size_t i = 0; i < kMaxSinkDispatchers; ++i) {
        _sink_dispatch_tasks[i].store(nullptr, std::memory_order_relaxed);
    }
}

GPIOGroup::GPIOGroup() noexcept
{
    initWatchState();
}

GPIOGroup::GPIOGroup(const IGPIO* mcu_gpio) noexcept
{
    initWatchState();
    if (mcu_gpio != nullptr) {
        _entries[0]             = Entry{mcu_gpio, 0};
        _entries[0].push_events = mcu_gpio->hasPushEvents();
        _count                  = 1;
    }
}

result_t<void> GPIOGroup::addGPIO(const IGPIO* gpio, types::gpio_slot_t slot)
{
    if (gpio == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (slot >= kSlotCount) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const uint16_t pin_count = gpio->getPinCount();
    if (pin_count == 0 || pin_count > 256) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    // Reject duplicate slot (single linear scan).
    for (size_t i = 0; i < _count; ++i) {
        if (_entries[i].slot == slot) {
            return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
        }
    }
    // Storage cap (checked after we confirm there's no duplicate).
    if (_count >= kMaxEntries) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    for (size_t p = 0; p < kMaxPortsPerEntry; ++p) {
        _watch_mask[_count][p].store(0, std::memory_order_relaxed);
        _watch_shadow[_count][p].store(0, std::memory_order_relaxed);
    }
    _entries[_count]             = Entry{gpio, slot};
    _entries[_count].push_events = gpio->hasPushEvents();
    ++_count;
    return {};
}

result_t<void> GPIOGroup::removeGPIO(types::gpio_slot_t slot)
{
    if (slot >= kSlotCount) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    for (size_t i = 0; i < _count; ++i) {
        if (_entries[i].slot == slot) {
            const size_t last = _count - 1;
            _entries[i]       = _entries[last];
            // startup-only contract: no concurrent watcher access, so
            // plain relaxed load/store is enough to move the row.
            for (size_t p = 0; p < kMaxPortsPerEntry; ++p) {
                _watch_mask[i][p].store(_watch_mask[last][p].load(std::memory_order_relaxed),
                                        std::memory_order_relaxed);
                _watch_shadow[i][p].store(_watch_shadow[last][p].load(std::memory_order_relaxed),
                                          std::memory_order_relaxed);
                _watch_mask[last][p].store(0, std::memory_order_relaxed);
                _watch_shadow[last][p].store(0, std::memory_order_relaxed);
            }
            _entries[last] = Entry{};
            _count         = last;
            return {};
        }
    }
    return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
}

void GPIOGroup::bindServiceRunner(service::ServiceRunner* runner)
{
    _service_runner = runner;
}

result_t<void> GPIOGroup::setWatchSink(WatchSink sink, void* ctx, uint32_t poll_interval_us)
{
    {
        group_detail::Guard guard{_watch_mutex};
        _watch_sink     = nullptr;
        _watch_sink_ctx = nullptr;
    }
    waitSinkIdle();

    // Deregister the poll service in every path (unregister AND
    // register/replace): ServiceRunner keeps the next_due returned by
    // the previous pass, so a re-register with a shorter interval must
    // go through remove + add to reset the runner-visible due state —
    // updating _watch_interval_ticks alone would leave the service
    // suppressed until the OLD interval elapses.
    bool was_registered = false;
    {
        group_detail::Guard guard{_watch_mutex};
        was_registered            = _watch_service_registered;
        _watch_service_registered = false;
    }
    if (was_registered && _service_runner != nullptr) {
        (void)_service_runner->remove(*this);  // R7: never call this while holding _watch_mutex
    }

    if (sink == nullptr) {
        return {};
    }

    // Safe without the runner lock: remove() above is synchronous (no
    // serviceImpl pass is in flight for this service anymore), so the
    // pass-private due tick has no concurrent writer here.
    _watch_next_tick = 0;

    const auto ticks = usToTicks(poll_interval_us == 0 ? kDefaultWatchIntervalUs : poll_interval_us);
    bool need_add    = false;
    {
        group_detail::Guard guard{_watch_mutex};
        _watch_sink           = sink;
        _watch_sink_ctx       = ctx;
        _watch_interval_ticks = ticks;
        need_add              = (_service_runner != nullptr);
        if (need_add) {
            _watch_service_registered = true;
        }
    }
    if (need_add && !_service_runner->add(*this)) {
        // Deterministic failure state: NO sink and NO poll service. A
        // failed REPLACE does not restore the previous sink — its
        // service was already removed above and re-adding it could
        // fail the same way, so "restore" cannot be guaranteed either.
        // Callers see OUT_OF_RESOURCE and may retry.
        group_detail::Guard guard{_watch_mutex};
        _watch_sink               = nullptr;
        _watch_sink_ctx           = nullptr;
        _watch_service_registered = false;
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    return {};
}

result_t<void> GPIOGroup::watch(types::gpio_number_t gpio_num)
{
    if (!isValid(gpio_num)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const auto slot    = types::extractSlot(gpio_num);
    const auto local   = types::extractLocalPin(gpio_num);
    const uint8_t p    = local >> 5;
    const uint32_t bit = 1u << (local & 31);
    if (p >= kMaxPortsPerEntry) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const size_t i = entryIndexOf(slot);
    if (i >= kMaxEntries) {
        // Defensive: isValid() already guarantees the slot resolves,
        // so this should be unreachable; treat it as invalid input
        // rather than indexing out of bounds.
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    // Seed the shadow with the current level BEFORE publishing the
    // mask bit: the opposite order would let a poll/notify pass that
    // races the mask store observe an unseeded shadow and report a
    // spurious edge on the very first sample.
    const bool level = getPin(gpio_num).read();
    if (level) {
        _watch_shadow[i][p].fetch_or(bit);
    } else {
        _watch_shadow[i][p].fetch_and(~bit);
    }
    _watch_mask[i][p].fetch_or(bit, std::memory_order_release);
    return {};
}

result_t<void> GPIOGroup::unwatch(types::gpio_number_t gpio_num)
{
    if (gpio_num < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const auto slot    = types::extractSlot(gpio_num);
    const auto local   = types::extractLocalPin(gpio_num);
    const uint8_t p    = local >> 5;
    const uint32_t bit = 1u << (local & 31);
    const size_t i     = entryIndexOf(slot);
    if (i >= kMaxEntries || p >= kMaxPortsPerEntry) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _watch_mask[i][p].fetch_and(~bit);  // idempotent: unwatching an unwatched pin still succeeds
    return {};
}

void GPIOGroup::clearWatchers()
{
    for (size_t i = 0; i < kMaxEntries; ++i) {
        for (size_t p = 0; p < kMaxPortsPerEntry; ++p) {
            _watch_mask[i][p].store(0, std::memory_order_relaxed);
        }
    }
    (void)setWatchSink(nullptr, nullptr);
}

result_t<void> GPIOGroup::notifyPinStateChanged(types::gpio_number_t gpio_num, bool level)
{
    if (gpio_num < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const auto slot    = types::extractSlot(gpio_num);
    const auto local   = types::extractLocalPin(gpio_num);
    const uint8_t p    = local >> 5;
    const uint32_t bit = 1u << (local & 31);
    const size_t i     = entryIndexOf(slot);
    if (i >= kMaxEntries || p >= kMaxPortsPerEntry) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    const uint32_t old = level ? _watch_shadow[i][p].fetch_or(bit) : _watch_shadow[i][p].fetch_and(~bit);
    if (((old & bit) != 0) == level) {
        return {};  // no transition: duplicate notification for the same level
    }
    if ((_watch_mask[i][p].load(std::memory_order_acquire) & bit) == 0) {
        return {};  // not watched: state recorded, no listener to notify
    }

    SinkDispatchScope scope{*this};
    if (scope.active) {
        scope.cb(scope.ctx, gpio_num, level, level ? Edge::Rising : Edge::Falling);
    }
    return {};
}

const IGPIO* GPIOGroup::getGPIO(types::gpio_slot_t slot) const
{
    const Entry* e = _find(slot);
    return e != nullptr ? e->gpio : nullptr;
}

bool GPIOGroup::hasGPIO(types::gpio_slot_t slot) const
{
    return _find(slot) != nullptr;
}

result_t<void> GPIOGroup::setDenyMask(types::gpio_slot_t slot, uint8_t port_index, uint32_t mask)
{
    Entry* e = _findMut(slot);
    if (e == nullptr || port_index >= kMaxPortsPerEntry) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    e->deny_mask[port_index] = mask;
    return {};
}

result_t<GPIOGroup::PortAccess> GPIOGroup::getPort(types::gpio_slot_t slot, uint8_t port_index) const
{
    const Entry* e = _find(slot);
    if (e == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (port_index >= e->gpio->getPortCount()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    uint32_t deny = port_index < kMaxPortsPerEntry ? e->deny_mask[port_index] : 0;
    return PortAccess{e->gpio->getPort(port_index), deny};
}

bool GPIOGroup::isValid(types::gpio_number_t gpio_num) const
{
    if (gpio_num < 0) {
        return false;
    }
    const Entry* e = _find(types::extractSlot(gpio_num));
    if (e == nullptr) {
        return false;
    }
    const auto local = types::extractLocalPin(gpio_num);
    if (!e->gpio->isValid(local)) {
        return false;
    }
    const uint8_t port_idx = local >> 5;
    if (port_idx < kMaxPortsPerEntry && (e->deny_mask[port_idx] & (1u << (local & 31)))) {
        return false;
    }
    return true;
}

result_t<Pin> GPIOGroup::tryGetPin(types::gpio_number_t gpio_num) const
{
    if (!isValid(gpio_num)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return getPin(gpio_num);
}

Pin GPIOGroup::getPin(types::gpio_number_t gpio_num) const
{
    M5HAL_ASSERT(gpio_num >= 0, "GPIOGroup::getPin: invalid sentinel (negative gpio_number_t)");
    const Entry* e = _find(types::extractSlot(gpio_num));
    M5HAL_ASSERT(e != nullptr, "GPIOGroup::getPin: slot unregistered");
    const types::gpio_local_pin_t local = types::extractLocalPin(gpio_num);
    M5HAL_ASSERT(e->gpio->isValid(local), "GPIOGroup::getPin: local pin out of range");
    const uint8_t port_idx = local >> 5;
    M5HAL_ASSERT(port_idx >= kMaxPortsPerEntry || !(e->deny_mask[port_idx] & (1u << (local & 31))),
                 "GPIOGroup::getPin: pin denied");
    return e->gpio->getPin(local);
}

const GPIOGroup::Entry* GPIOGroup::_find(types::gpio_slot_t slot) const
{
    for (size_t i = 0; i < _count; ++i) {
        if (_entries[i].slot == slot) {
            return &_entries[i];
        }
    }
    return nullptr;
}

GPIOGroup::Entry* GPIOGroup::_findMut(types::gpio_slot_t slot)
{
    for (size_t i = 0; i < _count; ++i) {
        if (_entries[i].slot == slot) {
            return &_entries[i];
        }
    }
    return nullptr;
}

size_t GPIOGroup::entryIndexOf(types::gpio_slot_t slot) const
{
    for (size_t i = 0; i < _count; ++i) {
        if (_entries[i].slot == slot) {
            return i;
        }
    }
    return kMaxEntries;
}

service::fast_tick_t GPIOGroup::usToTicks(uint32_t us)
{
    if (us == 0) {
        return 0;
    }
    const uint64_t nsec    = static_cast<uint64_t>(us) * 1000u;
    const uint32_t clamped = nsec > static_cast<uint64_t>(UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(nsec);
    return service::nsecToFastTickCeil(static_cast<service::tick_nsec_t>(clamped), service::fastTickFrequencyHz());
}

bool GPIOGroup::enterSinkDispatch(WatchSink& cb, void*& ctx)
{
    {
        group_detail::Guard guard{_watch_mutex};
        if (_watch_sink == nullptr) {
            return false;
        }
        cb  = _watch_sink;
        ctx = _watch_sink_ctx;
        _sink_inflight.fetch_add(1, std::memory_order_relaxed);
    }
    const auto id = runtime::currentTaskId();
    service::SpinBackoff backoff;
    for (;;) {
        bool claimed = false;
        for (size_t i = 0; i < kMaxSinkDispatchers; ++i) {
            void* expected = nullptr;
            if (_sink_dispatch_tasks[i].compare_exchange_strong(expected, id)) {
                claimed = true;
                break;
            }
        }
        if (claimed) {
            break;
        }
        backoff.step();
    }
    return true;
}

void GPIOGroup::exitSinkDispatch()
{
    const auto id = runtime::currentTaskId();
    for (size_t i = 0; i < kMaxSinkDispatchers; ++i) {
        if (_sink_dispatch_tasks[i].load(std::memory_order_relaxed) == id) {
            _sink_dispatch_tasks[i].store(nullptr, std::memory_order_relaxed);
            break;
        }
    }
    _sink_inflight.fetch_sub(1, std::memory_order_release);
}

void GPIOGroup::waitSinkIdle()
{
    const auto id = runtime::currentTaskId();
    for (size_t i = 0; i < kMaxSinkDispatchers; ++i) {
        if (_sink_dispatch_tasks[i].load(std::memory_order_acquire) == id) {
            return;  // self-unregister (R6): the calling task is itself mid-dispatch
        }
    }
    service::SpinBackoff backoff;
    while (_sink_inflight.load(std::memory_order_acquire) != 0) {
        backoff.step();
    }
}

service::ServicePoll GPIOGroup::serviceImpl(const service::ServiceContext& ctx)
{
    // Private virtual timeline: advances only by caller-vouched elapsed,
    // so the schedule below never compares ticks from different cores.
    _watch_svc_now += ctx.elapsed;
    if (_watch_next_tick != 0 && !service::hasReached(_watch_svc_now, _watch_next_tick)) {
        return {service::ServiceResult::Idle, static_cast<service::fast_tick_t>(_watch_next_tick - _watch_svc_now)};
    }

    service::fast_tick_t interval;
    {
        // Read once per pass; a mid-pass setWatchSink() change is
        // picked up starting with the NEXT pass.
        group_detail::Guard guard{_watch_mutex};
        interval = _watch_interval_ticks;
    }

    bool progressed = false;

    for (size_t i = 0; i < _count; ++i) {
        const auto& e = _entries[i];
        if (e.push_events) {
            continue;  // single-source rule: push-fed entries are never polled
        }
        const uint8_t port_count = e.gpio->getPortCount();
        const size_t ports       = port_count < kMaxPortsPerEntry ? port_count : kMaxPortsPerEntry;
        for (size_t p = 0; p < ports; ++p) {
            const uint32_t mask = _watch_mask[i][p].load(std::memory_order_acquire) & ~e.deny_mask[p];
            if (mask == 0) {
                continue;
            }
            IPort* port = e.gpio->getPort(static_cast<uint8_t>(p));
            if (port == nullptr) {
                continue;
            }
            const uint32_t value   = port->readPort();
            const uint32_t old     = _watch_shadow[i][p].exchange(value);
            const uint32_t changed = (old ^ value) & mask;
            if (changed == 0) {
                continue;
            }
            progressed = true;
            for (uint8_t bit_idx = 0; bit_idx < 32; ++bit_idx) {
                const uint32_t bit_mask = 1u << bit_idx;
                if ((changed & bit_mask) == 0) {
                    continue;
                }
                // One enter/exit per callback (not per pass): a sink
                // that unregisters/replaces itself from inside the
                // callback must not receive the remaining events of
                // the same pass — the next enter re-reads the sink
                // and stops the whole dispatch when it went away.
                SinkDispatchScope scope{*this};
                if (!scope.active) {
                    // No sink (anymore): shadows already exchanged stay
                    // current, undispatched events are simply dropped
                    // (nobody is listening).
                    goto pass_done;
                }
                const auto pin =
                    types::makeGpioNumber(e.slot, static_cast<types::gpio_local_pin_t>((p << 5) | bit_idx));
                const bool lvl = (value & bit_mask) != 0;
                scope.cb(scope.ctx, pin, lvl, lvl ? Edge::Rising : Edge::Falling);
            }
        }
    }
pass_done:
    _watch_next_tick = static_cast<service::fast_tick_t>(_watch_svc_now + interval);
    return {progressed ? service::ServiceResult::Progress : service::ServiceResult::Idle, interval};
}

}  // namespace m5::hal::v2::gpio

#endif  // M5_HAL_HAL_V2_GPIO_GROUP_INL_
