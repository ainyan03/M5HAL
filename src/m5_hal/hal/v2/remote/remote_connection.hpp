// SPDX-License-Identifier: MIT

#ifndef M5_HAL_REMOTE_REMOTE_CONNECTION_HPP_
#define M5_HAL_REMOTE_REMOTE_CONNECTION_HPP_

#include "../gpio/group.hpp"
#include "../runtime/runtime.hpp"
#include "../service/service.hpp"
#include "./remote.hpp"
#include "./session_handle.hpp"

#include <memory>
#include <utility>

namespace m5::hal::v2::remote {

class RemoteBackend;
class RemoteSession;

/*! @brief Type-erased owner for a connection's remote GPIO object.

  `gpio::Pin` is a small value handle containing a raw `IPort*`.  A Hal that
  replaces a remote connection therefore transfers this owner to its retired
  list instead of destroying the GPIO together with the transport.  The
  transport and its buffers are not retained: ports only share the separately
  closed `RemoteSessionHandle`.
 */
class RemoteGpioOwner {
public:
    virtual ~RemoteGpioOwner()              = default;
    virtual const gpio::IGPIO* gpio() const = 0;

    std::unique_ptr<RemoteGpioOwner> next;
};

template <typename T>
class RemoteGpioOwnerModel final : public RemoteGpioOwner {
public:
    template <typename... Args>
    explicit RemoteGpioOwnerModel(Args&&... args) : _gpio(std::forward<Args>(args)...)
    {
    }

    const gpio::IGPIO* gpio() const override
    {
        return &_gpio;
    }

    T& value()
    {
        return _gpio;
    }

private:
    T _gpio;
};

struct PumpConfig {
    bool keepalive                 = false;
    uint32_t keepalive_interval_ms = 20;
};

struct RemoteConnectionState {
    virtual ~RemoteConnectionState()                             = default;
    virtual RemoteSession& session()                             = 0;
    virtual std::shared_ptr<RemoteSessionHandle> sessionHandle() = 0;
    virtual RemoteBackend& backend()                             = 0;
    virtual const Capabilities& capabilities() const             = 0;
    virtual service::IService* service()
    {
        return nullptr;
    }
    virtual const gpio::IGPIO* gpio() const
    {
        return nullptr;
    }
    virtual std::unique_ptr<RemoteGpioOwner> releaseGpioOwnership()
    {
        return {};
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
