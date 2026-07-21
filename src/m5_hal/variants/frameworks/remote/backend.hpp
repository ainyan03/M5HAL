// SPDX-License-Identifier: MIT

#ifndef M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_BACKEND_HPP_
#define M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_BACKEND_HPP_

#include "../../../hal/v2/bus/hal_backend.hpp"
#include "../../../hal/v2/data/memory.hpp"
#include "../../../hal/v2/remote/remote.hpp"
#include "bus_lease.hpp"
#include "hal/i2c/i2c.hpp"
#include "hal/i2s/i2s.hpp"
#include "hal/pdm/pdm.hpp"
#include "hal/spi/spi.hpp"
#include "hal/uart/uart.hpp"
#include "session.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace m5::hal::v2::remote {

// ---- RemoteBackend ---------------------------------------------------------
// IHalBackend implementation for remote (peer MCU) connections.
// Takes an established RemoteSession; transport setup is the caller's concern.
// Platform-independent: builds on ESP32, POSIX, and native test targets.

class RemoteBackend : public bus::IHalBackend {
public:
    explicit RemoteBackend(std::shared_ptr<RemoteSessionHandle> session) : _session{std::move(session)}
    {
    }

    explicit RemoteBackend(RemoteSession& session) : RemoteBackend{makeBorrowedSessionHandle(session)}
    {
    }

    void setCapabilities(const Capabilities& caps)
    {
        _caps = caps;
    }
    const Capabilities& capabilities() const
    {
        return _caps;
    }

    bus::BusRegistry& busRegistry(void) override
    {
        return _registry;
    }
    const bus::BusRegistry& busRegistry(void) const override
    {
        return _registry;
    }

    result_t<std::shared_ptr<bus::IBus>> acquireBusPortable(types::bus_kind_t kind, const bus::ResourceKey& id,
                                                            const bus::IBusConfig& cfg) override;
    result_t<std::shared_ptr<bus::IBus>> acquireBusLogical(types::bus_kind_t kind, const bus::ResourceKey& id,
                                                           const bus::AllocationRequest& req) override;
    result_t<void> commitBuses(types::bus_kind_t kind, uint32_t timeout_ms) override;
    result_t<void> closeBus(types::bus_kind_t kind, const bus::ResourceKey& id,
                            const std::shared_ptr<bus::IBus>& expected) override;

private:
    result_t<bus::ResourceKey> sessionResourceKey(types::bus_kind_t kind, const bus::ResourceKey& target) const;
    static uint8_t extractBusId(types::bus_kind_t kind, const std::shared_ptr<bus::IBus>& bus);
    static std::shared_ptr<RemoteBusLease> extractBusLease(types::bus_kind_t kind,
                                                           const std::shared_ptr<bus::IBus>& bus);
    static bool sameBaseConfig(types::bus_kind_t kind, const bus::IBusConfig& current,
                               const bus::IBusConfig& requested);
    result_t<void> sendReleaseBus(types::bus_kind_t kind, uint8_t bus_id);
    result_t<std::shared_ptr<bus::IBus>> createRemoteBus(types::bus_kind_t kind, data::ConstDataSpan pin_config,
                                                         const bus::IBusConfig& cfg);
    result_t<std::shared_ptr<bus::IBus>> createRemoteBusFromLogical(types::bus_kind_t kind,
                                                                    data::ConstDataSpan pin_config);
    result_t<bus::BusCapabilities> sendCreateBus(types::bus_kind_t kind, uint8_t bus_id,
                                                 data::ConstDataSpan pin_config);
    void setProxyCapabilities(types::bus_kind_t kind, const std::shared_ptr<bus::IBus>& proxy,
                              const bus::BusCapabilities& capabilities) const;
    result_t<std::shared_ptr<bus::IBus>> makeProxyBus(types::bus_kind_t kind, uint8_t bus_id,
                                                      const bus::IBusConfig& cfg,
                                                      const std::shared_ptr<RemoteBusLease>& bus_lease);
    result_t<std::shared_ptr<bus::IBus>> makeProxyBusFromPinConfig(types::bus_kind_t kind, uint8_t bus_id,
                                                                   data::ConstDataSpan pin_config,
                                                                   const std::shared_ptr<RemoteBusLease>& bus_lease);
    static types::gpio_number_t readI16LE(const uint8_t* p);
    static result_t<size_t> extractPinConfig(types::bus_kind_t kind, const bus::IBusConfig& cfg, uint8_t* out,
                                             size_t out_size);
    static result_t<size_t> extractPinConfigFromLogical(types::bus_kind_t kind, const void* config, uint8_t* out,
                                                        size_t out_size);
    uint8_t allocateBusId(types::bus_kind_t kind);
    void freeBusId(types::bus_kind_t kind, uint8_t bus_id);

    std::shared_ptr<RemoteSessionHandle> _session;
    bus::BusRegistry _registry;
    std::shared_ptr<RemoteBusIdState> _bus_ids = std::make_shared<RemoteBusIdState>();
    Capabilities _caps;
};

}  // namespace m5::hal::v2::remote

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_REMOTE_BACKEND_HPP_
