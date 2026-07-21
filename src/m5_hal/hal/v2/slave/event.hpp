// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_SLAVE_EVENT_HPP_
#define M5_HAL_HAL_V2_SLAVE_EVENT_HPP_

#include "../error.hpp"
#include "../service/service.hpp"

#include <atomic>
#include <stddef.h>
#include <stdint.h>

namespace m5::hal::v2::slave {

enum class SlaveEvent : uint16_t {
    None           = 0,
    RxAvailable    = 1u << 0,
    FrameCompleted = 1u << 1,
    TxSpace        = 1u << 2,
    Underrun       = 1u << 3,
    Overflow       = 1u << 4,
    FrameAborted   = 1u << 5,
    BusBroken      = 1u << 6,
};

constexpr SlaveEvent operator|(SlaveEvent lhs, SlaveEvent rhs);
constexpr SlaveEvent operator&(SlaveEvent lhs, SlaveEvent rhs);
constexpr SlaveEvent operator~(SlaveEvent value);
constexpr SlaveEvent& operator|=(SlaveEvent& lhs, SlaveEvent rhs);
constexpr bool any(SlaveEvent value);

struct SlaveEventInfo {
    SlaveEvent events   = SlaveEvent::None;
    size_t rx_available = 0;
    size_t tx_available = 0;
    uint32_t generation = 0;
};

using SlaveEventCallback   = void (*)(void* user, const SlaveEventInfo& event);
using SlaveEventLevelProbe = SlaveEventInfo (*)(void* user, uint32_t generation);

/*!
  @brief Level-triggered task-context notification endpoint for slave queues.

  Backends publish event bits from their single producer context. Delivery is
  coalesced by bit and never invokes user code directly. `dispatchEvents()` or
  the embedded service moves pending bits to the delivered set and invokes the
  callback in caller/service task context. A delivered bit is not dispatched
  again until the owner acknowledges it. A level probe run by acknowledge then
  republishes conditions that remain true, closing the clear-versus-producer
  lost-wakeup window without lending queue payload to the callback.
 */
class SlaveEventEndpoint : private service::IService {
public:
    SlaveEventEndpoint()                                     = default;
    SlaveEventEndpoint(const SlaveEventEndpoint&)            = delete;
    SlaveEventEndpoint& operator=(const SlaveEventEndpoint&) = delete;
    SlaveEventEndpoint(SlaveEventEndpoint&&)                 = delete;
    SlaveEventEndpoint& operator=(SlaveEventEndpoint&&)      = delete;

    result_t<void> setEventCallback(SlaveEventCallback callback, void* user);
    void setLevelProbe(SlaveEventLevelProbe probe, void* user);

    result_t<void> begin(uint32_t generation);
    void end(uint32_t generation);
    bool active() const;
    uint32_t generation() const;

    void publish(SlaveEvent events, size_t rx_available, size_t tx_available, uint32_t generation);
    result_t<void> dispatchEvents();
    result_t<void> acknowledgeEvents(SlaveEvent events);
    SlaveEvent pendingEvents() const;
    SlaveEvent deliveredEvents() const;

    service::IService& eventService();

private:
    service::ServicePoll serviceImpl(const service::ServiceContext& ctx) override;
    void publishLevel(const SlaveEventInfo& info, uint32_t generation);

    std::atomic<uint16_t> pending_{0};
    std::atomic<uint16_t> delivered_{0};
    std::atomic<size_t> rx_available_{0};
    std::atomic<size_t> tx_available_{0};
    std::atomic<uint32_t> generation_{0};
    std::atomic<bool> active_{false};
    std::atomic_flag dispatching_ = ATOMIC_FLAG_INIT;

    SlaveEventCallback callback_ = nullptr;
    void* callback_user_         = nullptr;
    SlaveEventLevelProbe probe_  = nullptr;
    void* probe_user_            = nullptr;
};

}  // namespace m5::hal::v2::slave

#include "event.inl"

#endif  // M5_HAL_HAL_V2_SLAVE_EVENT_HPP_
