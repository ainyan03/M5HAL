// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_GPIO_GPIO_INL_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_GPIO_GPIO_INL_

#include "gpio.hpp"

#include "../../../../../hal/v2/bytecode/bytecode.hpp"
#include "../../../../../hal/v2/remote/remote.hpp"
#include "../../detail_helpers.hpp"

namespace m5::hal::v2::gpio {

result_t<void> Port_remote::syncRead()
{
    if (_session == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto r = enc.gpioPortRead(0, _device_slot, _port_index);
    if (r.has_value()) {
        r = enc.end();
    }
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    remote::RemoteSessionHandle::Lease lease{*_session};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }
    auto& session = lease.session();
    auto req      = session.request({script_buf, script.written()});
    if (!req.has_value()) {
        return m5::stl::make_unexpected(req.error());
    }
    bytecode::BytecodeRunner runner{memory::defaultAllocator()};
    auto resp    = session.lastResponse();
    auto decoded = remote::detail::decodeResponseStatus(resp, &runner);
    if (!decoded.has_value()) {
        return m5::stl::make_unexpected(decoded.error());
    }
    auto stored = runner.storedData(0);
    if (stored.size < 4) {
        return m5::stl::make_unexpected(error::error_t::PROTOCOL_ERROR);
    }
    _cached.store(static_cast<uint32_t>(stored.data[0]) | (static_cast<uint32_t>(stored.data[1]) << 8) |
                      (static_cast<uint32_t>(stored.data[2]) << 16) | (static_cast<uint32_t>(stored.data[3]) << 24),
                  std::memory_order_relaxed);
    return {};
}

void Port_remote::_writePinEncoded(uint32_t pin_mask, bool v)
{
    if (v) {
        _writePortMasked(pin_mask, 0);
    } else {
        _writePortMasked(0, pin_mask);
    }
}

void Port_remote::_writePinEncodedHigh(uint32_t pin_mask)
{
    _writePortMasked(pin_mask, 0);
}

void Port_remote::_writePinEncodedLow(uint32_t pin_mask)
{
    _writePortMasked(0, pin_mask);
}

bool Port_remote::_readPinEncoded(uint32_t pin_mask)
{
    return (_cached.load(std::memory_order_relaxed) & pin_mask) != 0;
}

void Port_remote::_setPinModeEncoded(uint32_t pin_mask, types::gpio_mode_t mode)
{
    if (_session == nullptr) {
        return;
    }
    auto local    = _toLocalPin(pin_mask);
    auto gpio_num = types::makeGpioNumber(_device_slot, local);

    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto r = enc.gpioSetMode(mode, &gpio_num, 1);
    if (r.has_value()) {
        r = enc.end();
    }
    if (!r.has_value()) {
        return;
    }
    remote::RemoteSessionHandle::Lease lease{*_session};
    if (lease) {
        (void)lease.session().request({script_buf, script.written()});
    }
}

uint32_t Port_remote::_readPortAll()
{
    return _cached.load(std::memory_order_relaxed);
}

void Port_remote::_writePortMasked(uint32_t set_mask, uint32_t clear_mask)
{
    if (_session == nullptr) {
        return;
    }
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto r = enc.gpioPortWrite(_device_slot, _port_index, set_mask, clear_mask);
    if (r.has_value()) {
        r = enc.end();
    }
    if (!r.has_value()) {
        return;
    }
    remote::RemoteSessionHandle::Lease lease{*_session};
    if (lease) {
        uint32_t cached = _cached.load(std::memory_order_relaxed);
        while (!_cached.compare_exchange_weak(cached, (cached | set_mask) & ~clear_mask, std::memory_order_relaxed)) {
        }
        (void)lease.session().requestNoResponse({script_buf, script.written()});
    }
}

types::gpio_local_pin_t Port_remote::_toLocalPin(uint32_t pin_mask) const
{
    uint8_t bit = 0;
    for (; bit < 32; ++bit) {
        if (pin_mask & (1u << bit)) {
            break;
        }
    }
    return static_cast<types::gpio_local_pin_t>(bit + (_port_index << 5));
}

uint32_t Port_remote::_fromLocalPin(types::gpio_local_pin_t pin_index) const
{
    M5HAL_ASSERT((pin_index >> 5) == _port_index, "pin_index not in this port");
    return 1u << (pin_index & 31);
}

GPIO_remote::GPIO_remote(RemoteSession& session, types::gpio_slot_t device_slot, uint8_t port_count, uint16_t pin_count)
    : GPIO_remote{remote::makeBorrowedSessionHandle(session), device_slot, port_count, pin_count}
{
}

GPIO_remote::GPIO_remote(std::shared_ptr<remote::RemoteSessionHandle> session, types::gpio_slot_t device_slot,
                         uint8_t port_count, uint16_t pin_count)
    : _port_count{port_count > kMaxPorts ? static_cast<uint8_t>(kMaxPorts) : port_count},
      _pin_count{static_cast<uint16_t>(
          pin_count > static_cast<uint16_t>((port_count > kMaxPorts ? kMaxPorts : port_count) * 32u)
              ? static_cast<uint16_t>((port_count > kMaxPorts ? kMaxPorts : port_count) * 32u)
              : pin_count)}
{
    for (uint8_t i = 0; i < _port_count; ++i) {
        _ports[i].init(session, device_slot, i);
    }
}

gpio::IPort* GPIO_remote::portForPin(types::gpio_local_pin_t pin_index) const
{
    M5HAL_ASSERT(pin_index < _pin_count, "pin_index out of range");
    return &_ports[pin_index >> 5];
}

gpio::IPort* GPIO_remote::getPort(uint8_t port_number) const
{
    M5HAL_ASSERT(port_number < _port_count, "port_number out of range");
    return &_ports[port_number];
}

uint16_t GPIO_remote::getPinCount() const
{
    return _pin_count;
}

uint8_t GPIO_remote::getPortCount() const
{
    return _port_count;
}

result_t<void> GPIO_remote::seedCache()
{
    for (uint8_t p = 0; p < _port_count; ++p) {
        auto r = _ports[p].syncRead();
        if (!r.has_value()) {
            return m5::stl::make_unexpected(r.error());
        }
    }
    return {};
}

result_t<void> GPIO_remote::subscribeAll()
{
    if (_pin_count == 0 || _ports[0]._session == nullptr) {
        return {};
    }
    types::gpio_number_t pins[kMaxPorts * 32];
    auto slot = _ports[0]._device_slot;
    for (uint16_t i = 0; i < _pin_count; ++i) {
        pins[i] = types::makeGpioNumber(slot, static_cast<types::gpio_local_pin_t>(i));
    }
    uint8_t script_buf[remote::kMaxScriptSize];
    data::MemorySink script{script_buf, sizeof(script_buf)};
    bytecode::BytecodeEncoder enc{script};
    auto r = enc.gpioSubscribe(pins, _pin_count);
    if (r.has_value()) {
        r = enc.end();
    }
    if (!r.has_value()) {
        return m5::stl::make_unexpected(r.error());
    }
    remote::RemoteSessionHandle::Lease lease{*_ports[0]._session};
    if (!lease) {
        return m5::stl::make_unexpected(lease.error());
    }
    auto& session = lease.session();
    auto req      = session.request({script_buf, script.written()});
    if (!req.has_value()) {
        return m5::stl::make_unexpected(req.error());
    }
    return session.checkResponse();
}

void GPIO_remote::onGpioEvent(void* ctx, types::gpio_number_t pin, bool level)
{
    auto* self       = static_cast<GPIO_remote*>(ctx);
    auto local       = types::extractLocalPin(pin);
    uint8_t port_idx = local >> 5;
    if (port_idx >= self->_port_count) {
        return;
    }
    uint32_t mask = 1u << (local & 31);
    if (level) {
        self->_ports[port_idx].setCachedBit(mask);
    } else {
        self->_ports[port_idx].clearCachedBit(mask);
    }
    if (self->_event_group != nullptr) {
        auto global = types::makeGpioNumber(self->_event_slot, local);
        (void)self->_event_group->notifyPinStateChanged(global, level);
    }
}

void GPIO_remote::onSessionEvent(void* ctx, uint8_t seq, data::ConstDataSpan body)
{
    (void)seq;
    auto* self = static_cast<GPIO_remote*>(ctx);
    bytecode::BytecodeRunner runner{memory::defaultAllocator()};
    runner.setReceiveOnly(true);
    runner.setGpioEventHandler(&GPIO_remote::onGpioEvent, self);
    (void)runner.runEvent(body);
}

}  // namespace m5::hal::v2::gpio

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_HAL_GPIO_GPIO_INL_
