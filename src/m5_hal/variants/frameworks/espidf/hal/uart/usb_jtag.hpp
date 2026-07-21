// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_USB_JTAG_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_USB_JTAG_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/uart/bus_streaming.hpp"
#include "../../../../../hal/v2/uart/uart.hpp"

#if defined(ESP_PLATFORM)
#include <sdkconfig.h>
#if defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED)

#include "../../../freertos/hal/runtime/time.hpp"

#include <driver/usb_serial_jtag.h>
#include <esp_err.h>
#include <string.h>

namespace m5::hal::v2::uart {

/*!
  @brief uart::IBus backed by the ESP-IDF USB-Serial-JTAG driver.

  ESP32-S3, C3, C6, and other chips with an on-chip USB-Serial-JTAG
  peripheral can use this as a UART-like transport without a USB-UART
  bridge IC. The CDC virtual serial port appears as a regular COM/tty
  on the host.

  Unlike uart::Bus_espidf, this bus has no pin configuration (the USB
  D+/D- are fixed-function) and no baud rate (USB Full-Speed 12 Mbps;
  baud in AccessConfig is host metadata only). Instantiate directly:

    m5hal::uart::Bus_espidf_usb_jtag cdc;
    cdc.init();
 */
class Bus_espidf_usb_jtag : public uart::Bus_streaming {
    friend class Bus_console;

public:
    ~Bus_espidf_usb_jtag() override
    {
        (void)teardownBackend();
    }

    result_t<void> init();
    result_t<void> close(void)
    {
        return bus::IBus::close();
    }
    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }

protected:
    bus::CloseOutcome closeBackend(void) override
    {
        return teardownBackend();
    }

    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    bus::CloseOutcome teardownBackend(void);
    result_t<void> initWithBufferSizes(uint32_t rx_buffer_size, uint32_t tx_buffer_size, bool attach_existing);

    static constexpr uint32_t kRxDriverBufferSize = 8192;
    static constexpr uint32_t kTxDriverBufferSize = 2048;
    static constexpr size_t kRxCacheSize          = 512;

    static TickType_t ticks(uint32_t timeout_ms)
    {
        return ::m5::hal::v2::detail::timeoutMsToTicks(timeout_ms);
    }

    void clearRxCache();
    size_t rxTail() const;
    void pumpRxCache();
    size_t popRxCache(uint8_t* dst, size_t max_len);

    bool _installed    = false;
    bool _driver_owned = false;
    uint8_t _rx_cache[kRxCacheSize];
    size_t _rx_head  = 0;
    size_t _rx_count = 0;
};

}  // namespace m5::hal::v2::uart

#endif  // CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
#endif  // ESP_PLATFORM

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_USB_JTAG_HPP
