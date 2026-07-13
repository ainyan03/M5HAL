// SPDX-License-Identifier: MIT
#ifndef M5_HAL_REMOTE_SERVER_HANDLER_INL_
#define M5_HAL_REMOTE_SERVER_HANDLER_INL_

#include "server_handler.hpp"

#include "../diag.hpp"

#include <M5Utility.hpp>

namespace m5::hal::v2::remote {

result_t<void> RemoteServerHandler::handler(void* ctx, frame::Kind kind, uint8_t seq, data::ConstDataSpan payload,
                                            data::MuxFrameEncoder& enc, data::MuxFrameDecoder& dec)
{
    auto* h = static_cast<RemoteServerHandler*>(ctx);
    if (h == nullptr) {
        return writeError(enc, seq, error::error_t::INVALID_STATE);
    }

    switch (kind) {
        case frame::Kind::Control:
            // Reset is fire-and-forget per spec/design/remote.md §Control:
            // "host API は応答を待たない" — the host never reads this Response
            // or otherwise observes whether a restart actually happened, so a
            // platform that cannot restart itself writing this same success
            // Response and simply not restarting is within contract (already
            // the case for posix hosts, which have never called esp_restart
            // here either; RP2040/SAMD51 now share that same behavior).
            enc.writeFrame(frame::Kind::Response, seq, {});
#if defined(ESP_PLATFORM)
            // ESPIDF and arduino-esp32 both resolve to ESP_PLATFORM here;
            // other Arduino cores (RP2040 / SAMD51) have no FreeRTOS/esp_restart
            // and fall through to the plain `return {}` below, same as posix.
            vTaskDelay(pdMS_TO_TICKS(50));
            M5HAL_DIAG("restart requested seq=%u", static_cast<unsigned>(seq));
            esp_restart();
#endif
            return {};

        case frame::Kind::Ping:
            enc.writeFrame(frame::Kind::Pong, seq, {});
            return {};

        case frame::Kind::HelloReq: {
            M5HAL_DIAG("hello reset gpio=%d pool=%d", h->gpio != nullptr, h->pool != nullptr);
            if (h->pool != nullptr) {
                h->pool->releaseAll();
            }
            if (h->server != nullptr) {
                h->server->abortPendingStream();
            }
            h->clearGpioSubscriptions();
            h->clearGpioMonitorMasks();
            h->clearGpioSnapshot();

            // HelloResp: [proto_ver][flags][n]([bus_kind][bus_id])*n [gpio_port][gpio_pin:u16]?
            // (spec/design/remote.md §hello). n mirrors the server's statically
            // registered bus capabilities; the host decodes with decodeHelloCaps.
            uint8_t hello_body[3 + Capabilities::kMaxEntries * 2 + 3];
            uint8_t flags = h->hello_flags;
            if (h->pool != nullptr) {
                flags |= 0x02u;
            }
            if (h->gpio != nullptr) {
                flags |= 0x01u;
            }
            hello_body[0] = kProtocolVersion;
            hello_body[1] = flags;

            size_t cap_count = h->server != nullptr ? h->server->capabilityCount() : 0;
            if (cap_count > Capabilities::kMaxEntries) {
                cap_count = Capabilities::kMaxEntries;
            }
            hello_body[2]    = static_cast<uint8_t>(cap_count);
            size_t hello_len = 3;
            for (size_t i = 0; i < cap_count; ++i) {
                const Capabilities::BusEntry& entry = h->server->capabilityAt(i);
                hello_body[hello_len]               = static_cast<uint8_t>(entry.kind);
                hello_body[hello_len + 1]           = entry.bus_id;
                hello_len += 2;
            }
            if (h->gpio != nullptr) {
                uint16_t pin_cnt          = h->gpio->getPinCount();
                hello_body[hello_len]     = h->gpio->getPortCount();
                hello_body[hello_len + 1] = static_cast<uint8_t>(pin_cnt & 0xFFu);
                hello_body[hello_len + 2] = static_cast<uint8_t>((pin_cnt >> 8) & 0xFFu);
                hello_len += 3;
            }
            enc.writeFrame(frame::Kind::HelloResp, seq, {hello_body, hello_len});
            return {};
        }

        case frame::Kind::Request: {
            if (h->server == nullptr) {
                return writeError(enc, seq, error::error_t::INVALID_STATE);
            }
            h->clearGpioSnapshot();
            h->server->beginFrameRequest(seq, enc, dec);
            uint8_t resp_buf[frame::kMaxPayload];
            data::MemorySink resp_sink(resp_buf, sizeof(resp_buf));
            auto status         = h->server->processScript(payload, resp_sink);
            const bool deferred = h->server->responseDeferred();
            h->server->endFrameRequest();

            if (status.has_value()) {
                auto snapshot = h->writeGpioSnapshotEvent(enc);
                h->clearGpioSnapshot();
                if (!snapshot.has_value()) {
                    return m5::stl::make_unexpected(snapshot.error());
                }
            } else {
                h->clearGpioSnapshot();
            }

            if (deferred) {
                return {};
            }
            if ((seq & 0x80) != 0) {
                return {};
            }
            if (!status.has_value()) {
                return writeError(enc, seq, status.error());
            }
            enc.writeFrame(frame::Kind::Response, seq, {resp_buf, resp_sink.written()});
            return {};
        }

        default:
            return {};
    }
}

result_t<void> RemoteServerHandler::gpioSubscribe(void* ctx, bool subscribe, const types::gpio_number_t* pins,
                                                  size_t count)
{
    auto* h = static_cast<RemoteServerHandler*>(ctx);
    if (h == nullptr || h->gpio_group == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
    if (pins == nullptr && count != 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (count == 0) {
        if (!subscribe) {
            h->clearGpioSubscriptions();
        }
        return {};
    }

    if (!subscribe) {
        for (size_t i = 0; i < count; ++i) {
            h->removeGpioSubscription(pins[i]);
        }
        return {};
    }

    PendingGpioBit new_bits[kMaxGpioSubscriptions];
    size_t needed_bits  = 0;
    size_t needed_ports = 0;
    for (size_t i = 0; i < count; ++i) {
        GpioPinRef ref;
        if (!h->resolveGpioPin(pins[i], ref)) {
            continue;
        }
        auto* sub = h->findGpioSubscription(ref.slot, ref.port_index);
        if (sub != nullptr && (sub->subscribed_mask & ref.bit_mask) != 0) {
            continue;
        }
        if (h->hasPendingGpioBit(new_bits, needed_bits, ref.slot, ref.port_index, ref.bit_mask)) {
            continue;
        }
        if (needed_bits >= kMaxGpioSubscriptions) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        if (sub == nullptr && !h->hasPendingGpioPort(new_bits, needed_bits, ref.slot, ref.port_index)) {
            ++needed_ports;
        }
        new_bits[needed_bits++] = PendingGpioBit{ref.slot, ref.port_index, ref.bit_mask};
    }
    if (needed_bits > h->freeGpioSubscriptionCount() || needed_ports > h->freeGpioSubscriptionPortCount()) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    for (size_t i = 0; i < count; ++i) {
        GpioPinRef ref;
        if (!h->resolveGpioPin(pins[i], ref)) {
            continue;
        }
        auto* sub     = h->findGpioSubscription(ref.slot, ref.port_index);
        uint32_t now  = 0;
        bool have_now = false;
        if (sub == nullptr) {
            now      = ref.pa.port->readPort() & ~ref.pa.deny_mask;
            have_now = true;
            sub      = h->firstFreeGpioSubscription();
            if (sub != nullptr) {
                sub->slot            = ref.slot;
                sub->port_index      = ref.port_index;
                sub->subscribed_mask = 0;
                sub->last_value      = now;
                sub->used            = true;
            }
        }
        if (sub == nullptr) {
            return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
        }
        const uint32_t new_mask = ref.bit_mask & ~sub->subscribed_mask;
        if (!have_now) {
            now = ref.pa.port->readPort() & ~ref.pa.deny_mask;
        }
        if (new_mask != 0) {
            sub->last_value = (sub->last_value & ~new_mask) | (now & new_mask);
            sub->subscribed_mask |= ref.bit_mask;
        }
        auto snap = h->appendGpioSnapshot(ref.pin, (now & ref.bit_mask) != 0);
        if (!snap.has_value()) {
            return m5::stl::make_unexpected(snap.error());
        }
    }
    return {};
}

result_t<void> RemoteServerHandler::gpioModeSet(void* ctx, types::gpio_number_t pin, types::gpio_mode_t mode)
{
    (void)mode;
    auto* h = static_cast<RemoteServerHandler*>(ctx);
    if (h == nullptr) {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }

    types::gpio_slot_t slot = 0;
    uint8_t port_index      = 0;
    uint32_t bit_mask       = 0;
    if (!decodeGpioPin(pin, slot, port_index, bit_mask)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

    auto* mon = h->findGpioMonitorMask(slot, port_index);
    if (mon == nullptr) {
        mon = h->firstFreeGpioMonitorMask();
        if (mon == nullptr) {
            return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
        }
        mon->slot       = slot;
        mon->port_index = port_index;
        mon->mask       = 0;
        mon->used       = true;
    }
    mon->mask |= bit_mask;
    return {};
}

void RemoteServerHandler::gpioPinsClaimed(void* ctx, const types::gpio_number_t* pins, size_t count)
{
    auto* h = static_cast<RemoteServerHandler*>(ctx);
    if (h == nullptr || (pins == nullptr && count != 0)) {
        return;
    }
    for (size_t i = 0; i < count; ++i) {
        types::gpio_slot_t slot = 0;
        uint8_t port_index      = 0;
        uint32_t bit_mask       = 0;
        if (!decodeGpioPin(pins[i], slot, port_index, bit_mask)) {
            continue;
        }
        auto* mon = h->findGpioMonitorMask(slot, port_index);
        if (mon == nullptr) {
            continue;
        }
        mon->mask &= ~bit_mask;
        if (mon->mask == 0) {
            *mon = GpioMonitorMask{};
        }
    }
}

result_t<void> RemoteServerHandler::poll(void* ctx, data::MuxFrameEncoder& enc)
{
    auto* h = static_cast<RemoteServerHandler*>(ctx);
    if (h == nullptr) {
        return {};
    }
    if (h->server != nullptr) {
        auto stream = h->server->poll(enc, static_cast<uint32_t>(m5::utility::millis()));
        if (!stream.has_value()) {
            return m5::stl::make_unexpected(stream.error());
        }
    }
    if (h->gpio_group == nullptr) {
        return {};
    }

    types::gpio_number_t changed_pins[kMaxGpioSubscriptions];
    bool changed_levels[kMaxGpioSubscriptions];
    GpioSubscription* changed_subs[kMaxGpioSubscriptionPorts];
    uint32_t changed_values[kMaxGpioSubscriptionPorts];
    size_t changed_count     = 0;
    size_t changed_sub_count = 0;

    for (size_t i = 0; i < kMaxGpioSubscriptionPorts; ++i) {
        auto& sub = h->gpio_subscriptions[i];
        if (!sub.used || sub.subscribed_mask == 0) {
            continue;
        }
        auto pa = h->gpio_group->getPort(sub.slot, sub.port_index);
        if (!pa.has_value()) {
            sub = GpioSubscription{};
            continue;
        }
        const uint32_t value        = pa.value().port->readPort() & ~pa.value().deny_mask;
        auto* mon                   = h->findGpioMonitorMask(sub.slot, sub.port_index);
        const uint32_t monitor_mask = mon != nullptr ? mon->mask : 0;
        const uint32_t changed_mask = (value ^ sub.last_value) & sub.subscribed_mask & monitor_mask;
        if (changed_mask == 0) {
            sub.last_value = value;
            continue;
        }
        if (changed_sub_count >= kMaxGpioSubscriptionPorts) {
            return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
        }
        changed_subs[changed_sub_count]     = &sub;
        changed_values[changed_sub_count++] = value;
        for (uint8_t bit = 0; bit < 32; ++bit) {
            const uint32_t mask = 1u << bit;
            if ((changed_mask & mask) == 0) {
                continue;
            }
            if (changed_count >= kMaxGpioSubscriptions) {
                return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
            }
            changed_pins[changed_count] = types::makeGpioNumber(
                sub.slot, static_cast<types::gpio_local_pin_t>((static_cast<uint16_t>(sub.port_index) << 5) | bit));
            changed_levels[changed_count] = (value & mask) != 0;
            ++changed_count;
        }
    }

    if (changed_count == 0) {
        return {};
    }

    uint8_t script_buf[kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder script_enc{script};
    auto r = script_enc.evtGpioState(changed_pins, changed_levels, changed_count);
    if (r.has_value()) {
        r = script_enc.end();
    }
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    if (!enc.writeFrame(frame::Kind::Event, h->gpio_event_seq++, {script_buf, script.written()})) {
        return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
    }
    for (size_t i = 0; i < changed_sub_count; ++i) {
        changed_subs[i]->last_value = changed_values[i];
    }
    return {};
}

void RemoteServerHandler::clearGpioSubscriptions()
{
    for (size_t i = 0; i < kMaxGpioSubscriptionPorts; ++i) {
        gpio_subscriptions[i] = GpioSubscription{};
    }
    gpio_event_seq = 0;
}

void RemoteServerHandler::clearGpioMonitorMasks()
{
    for (size_t i = 0; i < kMaxGpioSubscriptionPorts; ++i) {
        gpio_monitor_masks[i] = GpioMonitorMask{};
    }
}

void RemoteServerHandler::clearGpioSnapshot()
{
    gpio_snapshot_count = 0;
}

result_t<void> RemoteServerHandler::appendGpioSnapshot(types::gpio_number_t pin, bool level)
{
    for (size_t i = 0; i < gpio_snapshot_count; ++i) {
        if (gpio_snapshot_pins[i] == pin) {
            gpio_snapshot_levels[i] = level;
            return {};
        }
    }
    if (gpio_snapshot_count >= kMaxGpioSubscriptions) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    gpio_snapshot_pins[gpio_snapshot_count]   = pin;
    gpio_snapshot_levels[gpio_snapshot_count] = level;
    ++gpio_snapshot_count;
    return {};
}

result_t<void> RemoteServerHandler::writeGpioSnapshotEvent(data::MuxFrameEncoder& enc)
{
    if (gpio_snapshot_count == 0) {
        return {};
    }
    uint8_t script_buf[kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder script_enc{script};
    auto r = script_enc.evtGpioState(gpio_snapshot_pins, gpio_snapshot_levels, gpio_snapshot_count);
    if (r.has_value()) {
        r = script_enc.end();
    }
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    if (!enc.writeFrame(frame::Kind::Event, gpio_event_seq++, {script_buf, script.written()})) {
        return m5::stl::make_unexpected(error::error_t::BUFFER_OVERFLOW);
    }
    return {};
}

bool RemoteServerHandler::resolveGpioPin(types::gpio_number_t pin, GpioPinRef& ref)
{
    if (pin < 0 || gpio_group == nullptr) {
        return false;
    }
    auto resolved = gpio_group->tryGetPin(pin);
    if (!resolved.has_value()) {
        return false;
    }
    ref.pin        = pin;
    ref.slot       = types::extractSlot(pin);
    ref.local      = types::extractLocalPin(pin);
    ref.port_index = ref.local >> 5;
    ref.bit_mask   = 1u << (ref.local & 31);
    auto pa        = gpio_group->getPort(ref.slot, ref.port_index);
    if (!pa.has_value()) {
        return false;
    }
    ref.pa = pa.value();
    return true;
}

bool RemoteServerHandler::decodeGpioPin(types::gpio_number_t pin, types::gpio_slot_t& slot, uint8_t& port_index,
                                        uint32_t& bit_mask)
{
    if (pin < 0) {
        return false;
    }
    const auto local = types::extractLocalPin(pin);
    slot             = types::extractSlot(pin);
    port_index       = local >> 5;
    bit_mask         = 1u << (local & 31);
    return true;
}

RemoteServerHandler::GpioSubscription* RemoteServerHandler::findGpioSubscription(types::gpio_slot_t slot,
                                                                                 uint8_t port_index)
{
    for (size_t i = 0; i < kMaxGpioSubscriptionPorts; ++i) {
        if (gpio_subscriptions[i].used && gpio_subscriptions[i].slot == slot &&
            gpio_subscriptions[i].port_index == port_index) {
            return &gpio_subscriptions[i];
        }
    }
    return nullptr;
}

RemoteServerHandler::GpioSubscription* RemoteServerHandler::firstFreeGpioSubscription()
{
    for (size_t i = 0; i < kMaxGpioSubscriptionPorts; ++i) {
        if (!gpio_subscriptions[i].used) {
            return &gpio_subscriptions[i];
        }
    }
    return nullptr;
}

RemoteServerHandler::GpioMonitorMask* RemoteServerHandler::findGpioMonitorMask(types::gpio_slot_t slot,
                                                                               uint8_t port_index)
{
    for (size_t i = 0; i < kMaxGpioSubscriptionPorts; ++i) {
        if (gpio_monitor_masks[i].used && gpio_monitor_masks[i].slot == slot &&
            gpio_monitor_masks[i].port_index == port_index) {
            return &gpio_monitor_masks[i];
        }
    }
    return nullptr;
}

RemoteServerHandler::GpioMonitorMask* RemoteServerHandler::firstFreeGpioMonitorMask()
{
    for (size_t i = 0; i < kMaxGpioSubscriptionPorts; ++i) {
        if (!gpio_monitor_masks[i].used) {
            return &gpio_monitor_masks[i];
        }
    }
    return nullptr;
}

size_t RemoteServerHandler::freeGpioSubscriptionCount() const
{
    return kMaxGpioSubscriptions - subscribedGpioBitCount();
}

size_t RemoteServerHandler::freeGpioSubscriptionPortCount() const
{
    size_t count = 0;
    for (size_t i = 0; i < kMaxGpioSubscriptionPorts; ++i) {
        if (!gpio_subscriptions[i].used) {
            ++count;
        }
    }
    return count;
}

size_t RemoteServerHandler::subscribedGpioBitCount() const
{
    size_t count = 0;
    for (size_t i = 0; i < kMaxGpioSubscriptionPorts; ++i) {
        if (gpio_subscriptions[i].used) {
            count += countBits(gpio_subscriptions[i].subscribed_mask);
        }
    }
    return count;
}

void RemoteServerHandler::removeGpioSubscription(types::gpio_number_t pin)
{
    types::gpio_slot_t slot = 0;
    uint8_t port_index      = 0;
    uint32_t bit_mask       = 0;
    if (!decodeGpioPin(pin, slot, port_index, bit_mask)) {
        return;
    }
    auto* sub = findGpioSubscription(slot, port_index);
    if (sub != nullptr) {
        sub->subscribed_mask &= ~bit_mask;
        if (sub->subscribed_mask == 0) {
            *sub = GpioSubscription{};
        }
    }
}

bool RemoteServerHandler::hasPendingGpioBit(const PendingGpioBit* bits, size_t count, types::gpio_slot_t slot,
                                            uint8_t port_index, uint32_t bit_mask)
{
    for (size_t i = 0; i < count; ++i) {
        if (bits[i].slot == slot && bits[i].port_index == port_index && bits[i].bit_mask == bit_mask) {
            return true;
        }
    }
    return false;
}

bool RemoteServerHandler::hasPendingGpioPort(const PendingGpioBit* bits, size_t count, types::gpio_slot_t slot,
                                             uint8_t port_index)
{
    for (size_t i = 0; i < count; ++i) {
        if (bits[i].slot == slot && bits[i].port_index == port_index) {
            return true;
        }
    }
    return false;
}

size_t RemoteServerHandler::countBits(uint32_t value)
{
    size_t count = 0;
    while (value != 0) {
        value &= value - 1u;
        ++count;
    }
    return count;
}

result_t<void> RemoteServerHandler::writeError(data::MuxFrameEncoder& enc, uint8_t seq, error::error_t code)
{
    int8_t err = static_cast<int8_t>(code);
    M5HAL_DIAG("protocol error seq=%u code=%d", static_cast<unsigned>(seq), static_cast<int>(code));
    enc.writeFrame(frame::Kind::Control, seq, {reinterpret_cast<const uint8_t*>(&err), 1});
    return {};
}

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_SERVER_HANDLER_INL_
