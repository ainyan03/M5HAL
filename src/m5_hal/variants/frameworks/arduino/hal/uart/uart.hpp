// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ARDUINO_HAL_UART_UART_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/uart/bus_streaming.hpp"
#include "../../../../../hal/v2/uart/uart.hpp"

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#endif

#if defined(ARDUINO)

namespace m5::hal::v2::uart {

struct BusConfig_arduino : public uart::IBusConfig {
    using uart::IBusConfig::IBusConfig;

    constexpr BusConfig_arduino(void) : uart::IBusConfig{}
    {
    }

    void setSerial(::HardwareSerial& s)
    {
        serial     = &s;
        _hw_serial = true;
    }
    void setSerial(::Stream& s)
    {
        serial     = &s;
        _hw_serial = false;
    }

    ::Stream* serial = nullptr;
    bool _hw_serial  = false;
};

class Bus_arduino : public uart::Bus_streaming {
public:
    ~Bus_arduino() override
    {
        (void)release();
    }

    result_t<void> init(const BusConfig_arduino& config);
    result_t<void> release(void) override;

    result_t<size_t> write(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Source* src,
                           size_t len) override;
    result_t<size_t> read(bus::IAccessor* owner, const uart::AccessConfig& cfg, data::Sink* dst, size_t len) override;
    result_t<size_t> readableBytes(bus::IAccessor* owner, const uart::AccessConfig& cfg) override;

    /*!
      @brief Adopt a caller-owned HardwareSerial (or plain Stream, below).

      Same contract as managed_facade's `init`/`release` (see
      managed_facade.hpp): call only OUTSIDE any access window on this bus
      (no TX/RX accessor mid-transfer). Attaching while an accessor holds a
      channel would race the serial object's lifetime the way a direct
      re-init/release would.
     */
    error::error_t attach(::HardwareSerial& serial);
    /*! @brief Adopt a caller-owned plain Stream. Same contract as the overload above. */
    error::error_t attach(::Stream& stream);
    ::Stream* nativeStream() const
    {
        return _serial;
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
    // `uart::IBus::tryAcquireOppositeChannel`. The first apply (`!_begun`)
    // skips the gate. The gate applies uniformly regardless of `_hw_serial`
    // (a plain Stream still gates its `setTimeout` + bookkeeping update).
    result_t<void> applyConfig(bus::IAccessor* owner, Channel entered, const uart::AccessConfig& cfg);
    // Actual apply (HardwareSerial::begin / Stream::setTimeout); assumes
    // `_state_mutex` is already held.
    result_t<void> applyConfigLocked(const uart::AccessConfig& cfg);

    ::Stream* _serial = nullptr;
    bool _hw_serial   = false;
    bool _begun       = false;
    bool _attached    = false;
    uart::AccessConfig _applied_cfg;
    // Leaf mutex (see uart::IBus class comment) guarding
    // _serial/_hw_serial/_begun/_attached/_applied_cfg against concurrent
    // TX/RX access.
    runtime::Mutex _state_mutex;
    uint32_t _reconfig_skips = 0;  // skipped reconfigures (opposite channel busy); read via reconfigSkips()
};

// Facade backend selection: uart::Bus::init(BusConfig_arduino) -> Bus_arduino.
template <>
struct BackendFor<BusConfig_arduino> {
    using type = Bus_arduino;
};

}  // namespace m5::hal::v2::uart

#endif

#endif
