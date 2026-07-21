// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_SLAVE_EVENT_INL_
#define M5_HAL_HAL_V2_SLAVE_EVENT_INL_

#include "event.hpp"

namespace m5::hal::v2::slave {

constexpr SlaveEvent operator|(SlaveEvent lhs, SlaveEvent rhs)
{
    return static_cast<SlaveEvent>(static_cast<uint16_t>(lhs) | static_cast<uint16_t>(rhs));
}

constexpr SlaveEvent operator&(SlaveEvent lhs, SlaveEvent rhs)
{
    return static_cast<SlaveEvent>(static_cast<uint16_t>(lhs) & static_cast<uint16_t>(rhs));
}

constexpr SlaveEvent operator~(SlaveEvent value)
{
    return static_cast<SlaveEvent>(~static_cast<uint16_t>(value));
}

constexpr SlaveEvent& operator|=(SlaveEvent& lhs, SlaveEvent rhs)
{
    lhs = lhs | rhs;
    return lhs;
}

constexpr bool any(SlaveEvent value)
{
    return value != SlaveEvent::None;
}

inline result_t<void> SlaveEventEndpoint::setEventCallback(SlaveEventCallback callback, void* user)
{
    if (active_.load(std::memory_order_acquire)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    callback_      = callback;
    callback_user_ = user;
    return {};
}

inline void SlaveEventEndpoint::setLevelProbe(SlaveEventLevelProbe probe, void* user)
{
    probe_      = probe;
    probe_user_ = user;
}

inline result_t<void> SlaveEventEndpoint::begin(uint32_t new_generation)
{
    if (new_generation == 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    bool expected = false;
    if (!active_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    pending_.store(0, std::memory_order_relaxed);
    delivered_.store(0, std::memory_order_relaxed);
    rx_available_.store(0, std::memory_order_relaxed);
    tx_available_.store(0, std::memory_order_relaxed);
    generation_.store(new_generation, std::memory_order_release);
    return {};
}

inline void SlaveEventEndpoint::end(uint32_t expected_generation)
{
    if (generation_.load(std::memory_order_acquire) != expected_generation) {
        return;
    }
    active_.store(false, std::memory_order_release);
    pending_.store(0, std::memory_order_release);
    delivered_.store(0, std::memory_order_release);
}

inline bool SlaveEventEndpoint::active() const
{
    return active_.load(std::memory_order_acquire);
}

inline uint32_t SlaveEventEndpoint::generation() const
{
    return generation_.load(std::memory_order_acquire);
}

inline void SlaveEventEndpoint::publish(SlaveEvent events, size_t rx_available, size_t tx_available,
                                        uint32_t event_generation)
{
    if (!any(events) || !active_.load(std::memory_order_acquire) ||
        generation_.load(std::memory_order_acquire) != event_generation) {
        return;
    }
    rx_available_.store(rx_available, std::memory_order_relaxed);
    tx_available_.store(tx_available, std::memory_order_relaxed);
    pending_.fetch_or(static_cast<uint16_t>(events), std::memory_order_release);
}

inline result_t<void> SlaveEventEndpoint::dispatchEvents()
{
    if (!active_.load(std::memory_order_acquire)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    if (dispatching_.test_and_set(std::memory_order_acquire)) {
        return m5::stl::make_unexpected(error::error_t::BUSY);
    }
    struct DispatchGuard {
        std::atomic_flag& flag;
        ~DispatchGuard()
        {
            flag.clear(std::memory_order_release);
        }
    } guard{dispatching_};

    const uint16_t already_delivered = delivered_.load(std::memory_order_acquire);
    uint16_t observed                = pending_.load(std::memory_order_acquire);
    uint16_t ready                   = 0;
    for (;;) {
        ready = observed & static_cast<uint16_t>(~already_delivered);
        if (ready == 0) {
            break;
        }
        const uint16_t remaining = observed & static_cast<uint16_t>(~ready);
        if (pending_.compare_exchange_weak(observed, remaining, std::memory_order_acq_rel, std::memory_order_acquire)) {
            break;
        }
    }
    if (ready == 0) {
        return {};
    }
    delivered_.fetch_or(ready, std::memory_order_release);
    if (callback_ != nullptr) {
        SlaveEventInfo info;
        info.events       = static_cast<SlaveEvent>(ready);
        info.rx_available = rx_available_.load(std::memory_order_acquire);
        info.tx_available = tx_available_.load(std::memory_order_acquire);
        info.generation   = generation_.load(std::memory_order_acquire);
        callback_(callback_user_, info);
    }
    return {};
}

inline void SlaveEventEndpoint::publishLevel(const SlaveEventInfo& info, uint32_t expected_generation)
{
    if (info.generation != 0 && info.generation != expected_generation) {
        return;
    }
    publish(info.events, info.rx_available, info.tx_available, expected_generation);
}

inline result_t<void> SlaveEventEndpoint::acknowledgeEvents(SlaveEvent events)
{
    if (!active_.load(std::memory_order_acquire)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }
    const uint16_t bits = static_cast<uint16_t>(events);
    delivered_.fetch_and(static_cast<uint16_t>(~bits), std::memory_order_acq_rel);

    const uint32_t current_generation = generation_.load(std::memory_order_acquire);
    if (probe_ != nullptr) {
        publishLevel(probe_(probe_user_, current_generation), current_generation);
    }
    return {};
}

inline SlaveEvent SlaveEventEndpoint::pendingEvents() const
{
    return static_cast<SlaveEvent>(pending_.load(std::memory_order_acquire));
}

inline SlaveEvent SlaveEventEndpoint::deliveredEvents() const
{
    return static_cast<SlaveEvent>(delivered_.load(std::memory_order_acquire));
}

inline service::IService& SlaveEventEndpoint::eventService()
{
    return *this;
}

inline service::ServicePoll SlaveEventEndpoint::serviceImpl(const service::ServiceContext&)
{
    if (!active()) {
        return {service::ServiceResult::Idle};
    }
    const bool had_pending = any(pendingEvents() & ~deliveredEvents());
    auto dispatched        = dispatchEvents();
    if (!dispatched.has_value()) {
        return {service::ServiceResult::Error};
    }
    return {had_pending ? service::ServiceResult::Progress : service::ServiceResult::Idle};
}

}  // namespace m5::hal::v2::slave

#endif  // M5_HAL_HAL_V2_SLAVE_EVENT_INL_
