// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_BUS_BUS_INL_
#define M5_HAL_HAL_V2_BUS_BUS_INL_

#include "bus.hpp"

namespace m5::hal::v2::bus {

const IBusConfig& IAccessor::getBusConfig(void) const
{
    return _bus->getConfig();
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
    return _operation_active;
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

CloseOutcome IBus::closeBackend(void)
{
    return CloseOutcome::success();
}

result_t<void> IBus::close(void)
{
    // Registry retirement must reserve and update the slot around teardown.
    // A permanent non-virtual marker prevents custom buses from bypassing
    // that ordering by declining bindRegistryRegistration().
    if (_registry_bound.load(std::memory_order_acquire)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    auto outcome = closeWithOutcome();
    if (outcome.disposition == CloseDisposition::Success) {
        return {};
    }
    return m5::stl::make_unexpected(outcome.error_code);
}

CloseOutcome IBus::closeWithOutcome(void)
{
    auto previous = _close_state.load(std::memory_order_acquire);
    for (;;) {
        if (previous == CloseState::Closing) {
            return CloseOutcome::noMutation(error::error_t::BUSY);
        }
        if (previous == CloseState::Closed) {
            return CloseOutcome::noMutation(error::error_t::CLOSED);
        }
        // compare_exchange updates `previous` after a race, so the next
        // iteration reclassifies Closing/Closed without invoking teardown.
        if (_close_state.compare_exchange_weak(previous, CloseState::Closing, std::memory_order_acq_rel,
                                               std::memory_order_acquire)) {
            break;
        }
    }

    // Prove that no Access owns the bus before touching backend state. The
    // zero-timeout lock keeps close non-blocking when an Access is active.
    auto barrier = tryAcquireCloseBarrier();
    if (!barrier.has_value()) {
        const auto mapped   = barrier.error() == error::error_t::TIMEOUT_ERROR ? error::error_t::BUSY : barrier.error();
        CloseState expected = CloseState::Closing;
        if (!_close_state.compare_exchange_strong(expected, previous, std::memory_order_acq_rel,
                                                  std::memory_order_acquire)) {
            // A multi-lock barrier may acquire one channel and then fail to
            // release it while rolling back. Its override marks the lifecycle
            // quarantined before returning so this base must not reopen it.
            return CloseOutcome::partialOrUnknown(mapped);
        }
        return CloseOutcome::noMutation(mapped);
    }
    auto outcome = closeBackend();
    switch (outcome.disposition) {
        case CloseDisposition::Success:
            _close_state.store(CloseState::Closed, std::memory_order_release);
            break;
        case CloseDisposition::NoMutation:
            _close_state.store(previous == CloseState::Quarantined ? CloseState::Quarantined : CloseState::Open,
                               std::memory_order_release);
            break;
        case CloseDisposition::PartialOrUnknown:
            _close_state.store(CloseState::Quarantined, std::memory_order_release);
            break;
    }
    auto released = releaseCloseBarrier();
    if (!released.has_value()) {
        _close_state.store(CloseState::Quarantined, std::memory_order_release);
        return CloseOutcome::partialOrUnknown(released.error());
    }
    return outcome;
}

result_t<void> IBus::tryAcquireCloseBarrier(void)
{
    return _mutex.lock(0);
}

result_t<void> IBus::releaseCloseBarrier(void)
{
    return _mutex.unlock();
}

bool IBus::initializationAllowed(bool registry_bound) const
{
    const auto state = _close_state.load(std::memory_order_acquire);
    if (state == CloseState::Open) {
        return true;
    }
    return state == CloseState::Closed && !registry_bound;
}

result_t<void> IBus::markInitializationSucceeded(bool registry_bound)
{
    auto state = _close_state.load(std::memory_order_acquire);
    if (state == CloseState::Open) {
        return {};
    }
    if (registry_bound || state != CloseState::Closed ||
        !_close_state.compare_exchange_strong(state, CloseState::Open, std::memory_order_acq_rel,
                                              std::memory_order_acquire)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    return {};
}

result_t<void> IBus::acquireAccessLock(IAccessor& owner, uint32_t timeout_ms)
{
    if (_close_state.load(std::memory_order_acquire) != CloseState::Open) {
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }
    auto locked = _mutex.lock(timeout_ms);
    if (!locked.has_value()) {
        return m5::stl::make_unexpected(locked.error());
    }
    if (_close_state.load(std::memory_order_acquire) != CloseState::Open) {
        auto unlocked = _mutex.unlock();
        if (!unlocked.has_value()) {
            markAccessBroken();
        }
        return m5::stl::make_unexpected(error::error_t::CLOSED);
    }
    _lock_owner = &owner;
    return {};
}

result_t<void> IBus::releaseAccessLock(IAccessor& owner)
{
    if (_lock_owner != &owner) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _lock_owner   = nullptr;  // cleared while the mutex is still held
    auto unlocked = _mutex.unlock();
    if (!unlocked.has_value()) {
        markAccessBroken();
    }
    return unlocked;
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

BusCapabilities IBus::capabilities(void) const
{
    detail::BusCapabilitiesBuilder builder;
    builder.setGeneration(backendGeneration());
    if (backendKind() == types::backend_kind_t::Hardware) {
        builder.enable(BusFeature::HardwareBackend);
    }
    if (const auto frequency = maxFrequency(); frequency != 0) {
        builder.setLimit(BusLimit::MaxFrequencyHz, frequency);
    }
    return builder.build();
}

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_HAL_V2_BUS_BUS_INL_
