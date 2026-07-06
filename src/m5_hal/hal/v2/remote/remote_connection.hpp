// SPDX-License-Identifier: MIT

#ifndef M5_HAL_REMOTE_REMOTE_CONNECTION_HPP_
#define M5_HAL_REMOTE_REMOTE_CONNECTION_HPP_

#include "../gpio/group.hpp"
#include "../runtime/runtime.hpp"
#include "../service/service.hpp"
#include "./remote.hpp"

namespace m5::hal::v2::remote {

class RemoteBackend;
class RemoteSession;

struct PumpConfig {
    bool keepalive                 = false;
    uint32_t keepalive_interval_ms = 20;
};

struct RemoteConnectionState {
    virtual ~RemoteConnectionState()                 = default;
    virtual RemoteSession& session()                 = 0;
    virtual RemoteBackend& backend()                 = 0;
    virtual const Capabilities& capabilities() const = 0;
    virtual service::IService* service()
    {
        return nullptr;
    }
    virtual const gpio::IGPIO* gpio() const
    {
        return nullptr;
    }
    virtual void bindGpioEvents(gpio::GPIOGroup&, types::gpio_slot_t)
    {
    }

    result_t<size_t> pollSession();
    result_t<void> keepalive(const PumpConfig& cfg);

private:
    uint32_t _last_keepalive_ms = 0;
};

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_REMOTE_REMOTE_CONNECTION_HPP_
