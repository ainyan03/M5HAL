// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_UART_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/uart/bus_streaming.hpp"
#include "../../../../../hal/v2/uart/uart.hpp"

#if defined(ESP_PLATFORM)
#include <driver/uart.h>
#include <esp_err.h>
#endif

#if defined(ESP_PLATFORM)

namespace m5::hal::v2::uart {

struct BusConfig_espidf : public uart::IBusConfig {
    // Inherit the tag-pin constructors (Tx / Rx, either order);
    // `port_num` keeps its member initializer and is set by assignment.
    using uart::IBusConfig::IBusConfig;

    int8_t port_num = -1;  ///< UART_NUM_0 when negative.

    constexpr BusConfig_espidf(void) : uart::IBusConfig{}
    {
    }
};

class Bus_espidf : public uart::Bus_streaming {
public:
    ~Bus_espidf() override
    {
        (void)release();
    }

    result_t<void> init(const BusConfig_espidf& config);
    result_t<void> release(void) override;

    result_t<size_t> write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src,
                           size_t len) override;
    result_t<size_t> read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len) override;
    result_t<size_t> readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg) override;

    ::uart_port_t nativePort() const
    {
        return _port;
    }

    /*! @brief Reconfiguration-skip count (diagnostic only); see spec/design/uart.md §state mutex. */
    uint32_t reconfigSkips();

protected:
    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    // Reconfiguration quiescence gate (spec/design/uart.md): `owner`/`entered`
    // identify the calling accessor and the channel it already holds so a
    // config change different from `_applied_cfg` can be gated through
    // `uart::IBus::tryAcquireOppositeChannel`. The first apply on a fresh
    // bus (`!_configured`) skips the gate.
    result_t<void> applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg);
    // Actual ESP-IDF apply; assumes `_state_mutex` is already held.
    result_t<void> applyConfigLocked(const uart::AccessConfig& cfg);

    ::uart_port_t _port = UART_NUM_0;
    bool _installed     = false;
    bool _configured    = false;
    uart::AccessConfig _applied_cfg;
    // Leaf mutex (see uart::IBus class comment) guarding
    // _installed/_configured/_applied_cfg against concurrent TX/RX access.
    runtime::Mutex _state_mutex;
    uint32_t _reconfig_skips = 0;  // skipped reconfigures (opposite channel busy); read via reconfigSkips()
};

// Facade backend selection: uart::Bus::init(BusConfig_espidf) -> Bus_espidf.
template <>
struct BackendFor<BusConfig_espidf> {
    using type = Bus_espidf;
};

}  // namespace m5::hal::v2::uart

#endif

#endif
