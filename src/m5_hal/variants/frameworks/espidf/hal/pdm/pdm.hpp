// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_PDM_PDM_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_PDM_PDM_HPP

#include "../../detail/espidf_version.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../../../../hal/v2/pdm/pdm.hpp"

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_PDM_HAS_RX_PCM

#include <atomic>
#include <driver/i2s_pdm.h>

namespace m5::hal::v2::pdm {

class Bus_espidf : public pdm::IBus {
public:
    ~Bus_espidf() override;

    result_t<void> init(const IBusConfig& config);
    result_t<void> close(void)
    {
        return bus::IBus::close();
    }

    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }
    int8_t controllerId(void) const override
    {
        return _controller;
    }
    bus::BusCapabilities capabilities(void) const override
    {
        return bus::detail::BusCapabilitiesBuilder{bus::IBus::capabilities()}.enable(bus::BusFeature::Receive).build();
    }

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<AccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<AccessConfig>& context) override;
    result_t<size_t> readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len) override;
    result_t<size_t> readableBytesBackend(bus::OperationContext<AccessConfig>& context) override;

    bus::CloseOutcome closeBackend(void) override;

private:
    result_t<void> ensureChannel(const AccessConfig& cfg);
    bus::CloseOutcome teardownBackend(void);
    result_t<void> resetForInitialization(void);
    result_t<void> failAfterSetup(error::error_t cause);
    static bool onRecvCallback(i2s_chan_handle_t, i2s_event_data_t* event, void* user_ctx);

    i2s_chan_handle_t _rx_handle = nullptr;
    int8_t _controller           = -1;
    bool _configured             = false;
    AccessConfig _applied_cfg;
    size_t _dma_capacity = 0;
    std::atomic<size_t> _dma_available{0};
};

inline result_t<std::unique_ptr<IBus>> makePortableBackend_espidf(const bus::LocalResourceContext& resources,
                                                                  const IBusConfig& config)
{
    return bus::makePortableBackend<IBus, Bus_espidf, IBusConfig>(resources, config);
}

template <class Policy>
struct NativeProvider_espidf {
    static result_t<std::shared_ptr<IBus>> acquire(bus::IHalBackend&, const IBusConfig&, Policy)
    {
        return m5::stl::make_unexpected(error::error_t::UNSUPPORTED);
    }
};

}  // namespace m5::hal::v2::pdm

#endif
#endif
