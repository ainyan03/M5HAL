// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_BUS_LOCAL_BACKEND_INL_
#define M5_HAL_HAL_V2_BUS_LOCAL_BACKEND_INL_

#include "local_backend.hpp"

namespace m5::hal::v2::bus {

void LocalBackend::registerKind(ILocalKindAdapter& adapter)
{
    _slots[kindIndex(adapter.kind())].adapter = &adapter;
}

result_t<std::shared_ptr<IBus>> LocalBackend::acquireBusLogical(types::bus_kind_t kind, const IdentityKey& id,
                                                                const AllocationRequest& req)
{
    auto& slot = _slots[kindIndex(kind)];
    if (!slot.adapter) {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
    auto acquired = _registry.acquireOrFind(kind, id, [&]() { return slot.adapter->createLogicalFacade(req); });
    if (!acquired.has_value()) {
        return acquired;
    }
    slot.adapter->retagIntent(*acquired.value(), req);
    return acquired;
}

result_t<void> LocalBackend::completeLogicalRequest(types::bus_kind_t kind, void* logical_config)
{
    auto& slot = _slots[kindIndex(kind)];
    if (!slot.adapter) {
        return {};
    }
    return slot.adapter->completeLogical(logical_config);
}

result_t<void> LocalBackend::commitBuses(types::bus_kind_t kind, uint32_t timeout_ms)
{
    auto& slot = _slots[kindIndex(kind)];
    if (!slot.adapter) {
        return {};
    }
    auto* core = slot.adapter->allocationCore();
    if (!core) {
        return {};
    }
    return core->commitBuses(timeout_ms);
}

uint8_t LocalBackend::hardwareInUse(types::bus_kind_t kind) const
{
    auto& slot = _slots[kindIndex(kind)];
    if (!slot.adapter) {
        return 0;
    }
    const auto* core = slot.adapter->allocationCore();
    return core != nullptr ? core->hardwareInUse() : 0;
}

size_t LocalBackend::kindIndex(types::bus_kind_t k)
{
    switch (k) {
        case types::bus_kind_t::I2C:
            return 0;
        case types::bus_kind_t::SPI:
            return 1;
        case types::bus_kind_t::UART:
            return 2;
        case types::bus_kind_t::I2S:
            return 3;
        default:
            return 4;
    }
}

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_HAL_V2_BUS_LOCAL_BACKEND_INL_
