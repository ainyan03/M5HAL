// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_GPIO_GROUP_INL_
#define M5_HAL_HAL_V2_GPIO_GROUP_INL_

#include "group.hpp"

namespace m5::hal::v2::gpio {

GPIOGroup::GPIOGroup(const IGPIO* mcu_gpio) noexcept
{
    if (mcu_gpio != nullptr) {
        _entries[0] = Entry{mcu_gpio, 0};
        _count      = 1;
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
    _entries[_count++] = Entry{gpio, slot};
    return {};
}

result_t<void> GPIOGroup::removeGPIO(types::gpio_slot_t slot)
{
    if (slot >= kSlotCount) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    for (size_t i = 0; i < _count; ++i) {
        if (_entries[i].slot == slot) {
            _entries[i]        = _entries[_count - 1];
            _entries[--_count] = Entry{};
            return {};
        }
    }
    return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
}

void GPIOGroup::bindServiceRunner(service::ServiceRunner* runner)
{
    _service_runner = runner;
}

result_t<GPIOGroup::watch_id_t> GPIOGroup::watch(types::gpio_number_t gpio_num, Edge edge, WatchCallback callback)
{
    WatchConfig cfg;
    return watch(gpio_num, edge, callback, nullptr, cfg);
}

result_t<GPIOGroup::watch_id_t> GPIOGroup::watch(types::gpio_number_t gpio_num, Edge edge, WatchCallback callback,
                                                 void* ctx)
{
    WatchConfig cfg;
    return watch(gpio_num, edge, callback, ctx, cfg);
}

result_t<GPIOGroup::watch_id_t> GPIOGroup::watch(types::gpio_number_t gpio_num, Edge edge, WatchCallback callback,
                                                 void* ctx, const WatchConfig& cfg)
{
    if (callback == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto pin = tryGetPin(gpio_num);
    if (!pin.has_value()) {
        return m5::stl::make_unexpected(pin.error());
    }
    size_t index = kMaxWatchers;
    for (size_t i = 0; i < kMaxWatchers; ++i) {
        if (!_watchers[i].used) {
            index = i;
            break;
        }
    }
    if (index == kMaxWatchers) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    auto& w           = _watchers[index];
    w.used            = true;
    w.pin             = gpio_num;
    w.edge            = edge;
    w.callback        = callback;
    w.ctx             = ctx;
    w.last_level      = pin.value().read();
    w.has_level       = true;
    w.last_event_tick = 0;
    w.poll_ticks      = usToTicks(cfg.poll_interval_us == 0 ? 1000 : cfg.poll_interval_us);
    w.debounce_ticks  = usToTicks(cfg.debounce_us);
    w.next_poll_tick  = 0;
    w.generation      = static_cast<uint8_t>(w.generation + 1u);
    if (w.generation == 0) {
        w.generation = 1;
    }

    ensureWatchService();
    return makeWatchId(index, w.generation);
}

bool GPIOGroup::unwatch(watch_id_t id)
{
    const size_t index = watchIndex(id);
    if (index >= kMaxWatchers) {
        return false;
    }
    auto& w = _watchers[index];
    if (!w.used || w.generation != watchGeneration(id)) {
        return false;
    }
    w.used     = false;
    w.callback = nullptr;
    dropQueuedEventsFor(index, w.generation);
    maybeRemoveWatchService();
    return true;
}

void GPIOGroup::clearWatchers()
{
    if (_watch_service_registered && _service_runner != nullptr) {
        (void)_service_runner->remove(*this);
    }
    _watch_service_registered = false;
    for (size_t i = 0; i < kMaxWatchers; ++i) {
        _watchers[i].used     = false;
        _watchers[i].callback = nullptr;
    }
    _event_head  = 0;
    _event_tail  = 0;
    _event_count = 0;
}

result_t<void> GPIOGroup::notifyPinStateChanged(types::gpio_number_t gpio_num, bool level)
{
    const auto now = service::defaultNowTick();
    bool matched   = false;
    for (size_t i = 0; i < kMaxWatchers; ++i) {
        auto& w = _watchers[i];
        if (!w.used || w.pin != gpio_num) {
            continue;
        }
        matched = true;
        auto r  = observeWatcher(i, level, now);
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
    }
    if (matched) {
        ensureWatchService();
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

GPIOGroup::watch_id_t GPIOGroup::makeWatchId(size_t index, uint8_t generation)
{
    return static_cast<watch_id_t>((static_cast<watch_id_t>(generation) << 8) | static_cast<watch_id_t>(index + 1u));
}

size_t GPIOGroup::watchIndex(watch_id_t id)
{
    const uint8_t raw = static_cast<uint8_t>(id & 0xFFu);
    return raw == 0 ? kMaxWatchers : static_cast<size_t>(raw - 1u);
}

uint8_t GPIOGroup::watchGeneration(watch_id_t id)
{
    return static_cast<uint8_t>((id >> 8) & 0xFFu);
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

bool GPIOGroup::edgeMatches(Edge watch_edge, Edge observed)
{
    return watch_edge == Edge::Change || watch_edge == observed;
}

bool GPIOGroup::anyWatcherUsed() const
{
    for (size_t i = 0; i < kMaxWatchers; ++i) {
        if (_watchers[i].used) {
            return true;
        }
    }
    return false;
}

void GPIOGroup::ensureWatchService()
{
    if (!_watch_service_registered && _service_runner != nullptr && anyWatcherUsed()) {
        _watch_service_registered = _service_runner->add(*this);
    }
}

void GPIOGroup::maybeRemoveWatchService()
{
    if (_watch_service_registered && !anyWatcherUsed() && _service_runner != nullptr) {
        (void)_service_runner->remove(*this);
        _watch_service_registered = false;
    }
}

result_t<void> GPIOGroup::enqueueEvent(size_t watcher_index, Edge edge, bool level)
{
    if (_event_count >= kMaxWatchEvents) {
        return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
    }
    const auto& w        = _watchers[watcher_index];
    _events[_event_tail] = WatchEvent{w.pin, edge, level, static_cast<uint8_t>(watcher_index), w.generation};
    _event_tail          = (_event_tail + 1u) % kMaxWatchEvents;
    ++_event_count;
    return {};
}

result_t<void> GPIOGroup::observeWatcher(size_t index, bool level, service::fast_tick_t now)
{
    auto& w = _watchers[index];
    if (!w.has_level) {
        w.last_level = level;
        w.has_level  = true;
        return {};
    }
    if (w.last_level == level) {
        return {};
    }
    const Edge observed = level ? Edge::Rising : Edge::Falling;
    w.last_level        = level;
    if (!edgeMatches(w.edge, observed)) {
        return {};
    }
    if (w.debounce_ticks != 0 && w.last_event_tick != 0 &&
        service::elapsedTicks(now, w.last_event_tick) < w.debounce_ticks) {
        return {};
    }
    w.last_event_tick = now;
    return enqueueEvent(index, observed, level);
}

void GPIOGroup::dropQueuedEventsFor(size_t watcher_index, uint8_t generation)
{
    size_t keep_count = 0;
    WatchEvent keep[kMaxWatchEvents];
    while (_event_count != 0) {
        const auto ev = _events[_event_head];
        _event_head   = (_event_head + 1u) % kMaxWatchEvents;
        --_event_count;
        if (ev.watcher_index == watcher_index && ev.generation == generation) {
            continue;
        }
        keep[keep_count++] = ev;
    }
    _event_head = 0;
    _event_tail = 0;
    for (size_t i = 0; i < keep_count; ++i) {
        _events[_event_tail] = keep[i];
        _event_tail          = (_event_tail + 1u) % kMaxWatchEvents;
    }
    _event_count = keep_count;
}

bool GPIOGroup::dispatchOneEvent()
{
    if (_event_count == 0) {
        return false;
    }
    const auto ev = _events[_event_head];
    _event_head   = (_event_head + 1u) % kMaxWatchEvents;
    --_event_count;

    if (ev.watcher_index >= kMaxWatchers) {
        return true;
    }
    const auto& w = _watchers[ev.watcher_index];
    if (!w.used || w.generation != ev.generation || w.callback == nullptr) {
        return true;
    }
    auto cb            = w.callback;
    auto* callback_ctx = w.ctx;
    cb(callback_ctx, ev.pin, ev.level, ev.edge);
    return true;
}

service::ServicePoll GPIOGroup::serviceImpl(const service::ServiceContext& ctx)
{
    bool progressed               = false;
    service::fast_tick_t next_due = 0;

    for (size_t i = 0; i < kMaxWatchers; ++i) {
        auto& w = _watchers[i];
        if (!w.used) {
            continue;
        }
        if (w.next_poll_tick != 0 && !service::hasReached(ctx.now_tick, w.next_poll_tick)) {
            if (next_due == 0 || service::hasReached(next_due, w.next_poll_tick)) {
                next_due = w.next_poll_tick;
            }
            continue;
        }
        auto pin = tryGetPin(w.pin);
        if (!pin.has_value()) {
            w.used     = false;
            progressed = true;
            continue;
        }
        auto observed = observeWatcher(i, pin.value().read(), ctx.now_tick);
        if (!observed.has_value()) {
            return {service::ServiceResult::Error};
        }
        w.next_poll_tick = static_cast<service::fast_tick_t>(ctx.now_tick + w.poll_ticks);
        if (next_due == 0 || service::hasReached(next_due, w.next_poll_tick)) {
            next_due = w.next_poll_tick;
        }
    }

    while (dispatchOneEvent()) {
        progressed = true;
    }
    maybeRemoveWatchService();
    return {progressed ? service::ServiceResult::Progress : service::ServiceResult::Idle, next_due};
}

}  // namespace m5::hal::v2::gpio

#endif  // M5_HAL_HAL_V2_GPIO_GROUP_INL_
