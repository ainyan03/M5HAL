// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/bus/hal_backend.hpp"
#include "../../../../../hal/v2/bus/portable_factory.hpp"
#include "../../../../../hal/v2/uart/bus_streaming.hpp"
#include "../../../../../hal/v2/uart/uart.hpp"

#if defined(ESP_PLATFORM)
#include <driver/uart.h>
#include <esp_err.h>
#endif

#if defined(ESP_PLATFORM)

namespace m5::hal::v2::uart {

/*! @brief Direct-owner ESP-IDF UART controller selection. */
struct NativePort {
    ::uart_port_t value = UART_NUM_0;
};

class Bus_espidf : public uart::Bus_streaming {
public:
    ~Bus_espidf() override
    {
        (void)teardownBackend();
    }

    result_t<void> init(const IBusConfig& config);
    result_t<void> init(const IBusConfig& config, native::Managed<NativePort> policy);
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
        return static_cast<int8_t>(_port);
    }

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<uart::AccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<uart::AccessConfig>& context) override;

    result_t<size_t> writeBackend(bus::OperationContext<uart::AccessConfig>& context, data::Source* src,
                                  size_t len) override;
    result_t<size_t> readBackend(bus::OperationContext<uart::AccessConfig>& context, data::Sink* dst,
                                 size_t len) override;
    result_t<size_t> readableBytesBackend(bus::OperationContext<uart::AccessConfig>& context) override;

public:
    ::uart_port_t nativePort() const
    {
        return _port;
    }

    /*! @brief Reconfiguration-skip count (diagnostic only); see spec/design/uart.md §state mutex. */
    uint32_t reconfigSkips();

protected:
    bus::CloseOutcome closeBackend(void) override
    {
        return teardownBackend();
    }

    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    result_t<void> initWithPort(const IBusConfig& config, ::uart_port_t port);
    bus::CloseOutcome teardownBackend(void);

    // Reconfiguration quiescence gate (spec/design/uart.md): `owner`/`entered`
    // identify the calling accessor and the channel it already holds so a
    // config change different from `_applied_cfg` can be gated through
    // `uart::IBus::tryAcquireOppositeChannel`. The first apply on a fresh
    // bus (`!_configured`) skips the gate.
    result_t<void> applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg,
                               uint32_t timeout_ms);
    // Actual ESP-IDF apply; assumes `_state_mutex` is already held.
    result_t<void> applyConfigLocked(const uart::AccessConfig& cfg);

    ::uart_port_t _port = UART_NUM_0;
    bool _installed     = false;
    bool _configured    = false;
    uart::AccessConfig _applied_cfg;
    // Leaf mutex (see uart::IBus class comment) guarding
    // _installed/_configured/_applied_cfg against concurrent TX/RX access.
    runtime::Mutex _state_mutex;
    uint32_t _reconfig_skips = 0;  // rejected reconfigures (opposite channel busy); read via reconfigSkips()
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

}  // namespace m5::hal::v2::uart

#endif

#endif
