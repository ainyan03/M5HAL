// SPDX-License-Identifier: MIT
#ifndef M5_HAL_HAL_V2_BUS_LOCAL_BACKEND_INL_
#define M5_HAL_HAL_V2_BUS_LOCAL_BACKEND_INL_

#include "local_backend.hpp"

namespace m5::hal::v2::bus {

void LocalBackend::registerKind(ILocalKindAdapter& adapter)
{
    _slots[kindIndex(adapter.kind())].adapter = &adapter;
}

void LocalBackend::registerPortableProvider(ILocalPortableProvider& provider)
{
    _slots[kindIndex(provider.kind())].portable_provider = &provider;
}

result_t<std::shared_ptr<IBus>> LocalBackend::acquireBusPortable(types::bus_kind_t kind, const ResourceKey& id,
                                                                 const IBusConfig& cfg)
{
    if (!id.isValid() || id.kind != kind || cfg.getBusKind() != kind) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto* provider = _slots[kindIndex(kind)].portable_provider;
    if (provider == nullptr) {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
    if (!(provider->identityFromConfig(cfg) == id)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return busRegistry().acquireOrFind(
        id,
        [provider, &cfg](const std::shared_ptr<IBus>& existing) -> result_t<void> {
            if (!provider->configCompatible(existing->getConfig(), cfg)) {
                return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
            }
            return {};
        },
        [this, provider, &cfg]() { return provider->createFacade(localResources(), cfg); });
}

result_t<std::shared_ptr<IBus>> LocalBackend::acquireBusLogical(types::bus_kind_t kind, const ResourceKey& id,
                                                                const AllocationRequest& req)
{
    if (!id.isValid() || id.kind != kind || req.kind != kind || !(req.identity == id) || req.config == nullptr ||
        !req.intent.valid()) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    auto& slot = _slots[kindIndex(kind)];
    if (!slot.adapter) {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
    auto acquired = busRegistry().acquireOrFind(id, [&]() { return slot.adapter->createLogicalFacade(req); });
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

result_t<int8_t> LocalBackend::claimController(types::bus_kind_t kind, const types::AllocationIntent& intent)
{
    auto& slot = _slots[kindIndex(kind)];
    if (!slot.adapter) {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
    auto* core = slot.adapter->allocationCore();
    if (!core) {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
    return core->claimController(intent);
}

result_t<void> LocalBackend::releaseClaimedController(types::bus_kind_t kind, int8_t controller)
{
    auto& slot = _slots[kindIndex(kind)];
    if (!slot.adapter) {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
    auto* core = slot.adapter->allocationCore();
    if (!core) {
        return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
    }
    return core->releaseClaimedController(controller);
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
        case types::bus_kind_t::PDM:
            return 4;
        default:
            return 5;
    }
}

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_HAL_V2_BUS_LOCAL_BACKEND_INL_
