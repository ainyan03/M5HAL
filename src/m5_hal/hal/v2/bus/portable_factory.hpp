// SPDX-License-Identifier: MIT

#ifndef M5_HAL_BUS_PORTABLE_FACTORY_HPP_
#define M5_HAL_BUS_PORTABLE_FACTORY_HPP_

#include "bus.hpp"

#include <memory>
#include <new>
#include <utility>

namespace m5::hal::v2::bus {

/*! @brief Build one selected provider backend from a portable kind config. */
template <class KindIBus, class Backend, class VariantConfig, class KindConfig, class Prepare>
result_t<std::unique_ptr<KindIBus>> makePortableBackend(const LocalResourceContext& resources, const KindConfig& config,
                                                        Prepare&& prepare)
{
    VariantConfig selected{};
    static_cast<KindConfig&>(selected) = config;
    auto prepared                      = prepare(selected);
    if (!prepared.has_value()) {
        return m5::stl::make_unexpected(prepared.error());
    }

    std::unique_ptr<Backend> concrete{new (std::nothrow) Backend()};
    if (!concrete) {
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }
    concrete->bindLocalResources(resources);
    auto initialized = concrete->init(selected);
    if (!initialized.has_value()) {
        return m5::stl::make_unexpected(initialized.error());
    }
    return std::unique_ptr<KindIBus>{std::move(concrete)};
}

template <class KindIBus, class Backend, class VariantConfig, class KindConfig>
result_t<std::unique_ptr<KindIBus>> makePortableBackend(const LocalResourceContext& resources, const KindConfig& config)
{
    return makePortableBackend<KindIBus, Backend, VariantConfig>(resources, config,
                                                                 [](VariantConfig&) -> result_t<void> { return {}; });
}

}  // namespace m5::hal::v2::bus

#endif  // M5_HAL_BUS_PORTABLE_FACTORY_HPP_
