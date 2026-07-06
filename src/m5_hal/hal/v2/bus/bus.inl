// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_BUS_BUS_INL_
#define M5_HAL_HAL_V2_BUS_BUS_INL_

#include "bus.hpp"

namespace m5::hal::v2::bus {

const IBusConfig& IAccessor::getBusConfig(void) const
{
    return _bus->getConfig();
}

m5::hal::v2::result_t<void> IAccessor::beginAccess(uint32_t timeout_ms)
{
    // The unbound gate: every sugar funnels through an access-window
    // opener, so this single check covers the hot paths below it.
    M5HAL_ASSERT(_bus != nullptr, "accessor is not bound to a bus (bind() it first)");
    if (_bus == nullptr) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_ARGUMENT);
    }
    if (_access_depth == 0) {
        auto r = _bus->lock(this, timeout_ms);
        if (!r.has_value()) {
            return r;
        }
    }
    ++_access_depth;
    return {};
}

m5::hal::v2::result_t<void> IAccessor::endAccess(void)
{
    if (_access_depth == 0) {
        return m5::stl::make_unexpected(m5::hal::v2::error::error_t::INVALID_STATE);
    }
    --_access_depth;
    if (_access_depth == 0) {
        return _bus->unlock(this);
    }
    return {};
}

types::bus_kind_t IAccessor::getBusKind(void) const
{
    return getConfig().getBusKind();
}

IAccessor::IAccessor(IBus& bus) : _bus{&bus}
{
}

IAccessor::IAccessor(std::shared_ptr<IBus> owner) : _bus{owner.get()}, _owner{std::move(owner)}
{
}

bool IAccessor::isBound(void) const
{
    return _bus != nullptr;
}

IBus& IAccessor::getBus(void) const
{
    return *_bus;
}

bool IAccessor::inAccess(void) const
{
    return _access_depth > 0;
}

void IAccessor::_bindBus(IBus& bus)
{
    _bus = &bus;
    _owner.reset();
}

types::bus_kind_t IBus::getBusKind(void) const
{
    return getConfig().getBusKind();
}

result_t<void> IBus::release(void)
{
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
}

result_t<void> IBus::lock(IAccessor* owner, uint32_t timeout_ms)
{
    if (owner == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    if (!_mutex.lock(timeout_ms)) {
        return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
    }
    _lock_owner = owner;
    return {};
}

result_t<void> IBus::unlock(IAccessor* owner)
{
    if (owner == nullptr || _lock_owner != owner) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _lock_owner = nullptr;  // cleared while the mutex is still held
    _mutex.unlock();
    return {};
}

types::backend_kind_t IBus::backendKind(void) const
{
    return types::backend_kind_t::Software;
}

int8_t IBus::controllerId(void) const
{
    return -1;
}

uint32_t IBus::maxFrequency(void) const
{
    return 0;
}

uint32_t IBus::backendGeneration(void) const
{
    return 0;
}

ScopedLock::ScopedLock(IBus& bus, IAccessor* owner, uint32_t timeout_ms) : _bus{&bus}, _owner{owner}
{
    auto r = _bus->lock(_owner, timeout_ms);
    if (!r.has_value()) {
        _error = r.error();
        _bus   = nullptr;  // dtor will not call unlock
    }
}

ScopedLock::~ScopedLock()
{
    if (_bus != nullptr) {
        (void)_bus->unlock(_owner);
    }
}

bool ScopedLock::has_error(void) const
{
    return _bus == nullptr;
}

bool ScopedLock::ok(void) const
{
    return !has_error();
}

m5::hal::v2::error::error_t ScopedLock::error(void) const
{
    return _error;
}

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_HAL_V2_BUS_BUS_INL_
