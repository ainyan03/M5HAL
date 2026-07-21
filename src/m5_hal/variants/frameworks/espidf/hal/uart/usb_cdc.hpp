// SPDX-License-Identifier: MIT
#ifndef M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_USB_CDC_HPP
#define M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_USB_CDC_HPP

#include "../../../../../hal/v2/bus/bus.hpp"
#include "../../../../../hal/v2/uart/bus_streaming.hpp"
#include "../../../../../hal/v2/uart/uart.hpp"

#if defined(ESP_PLATFORM)
#include <sdkconfig.h>
#if defined(SOC_USB_OTG_SUPPORTED) && SOC_USB_OTG_SUPPORTED && defined(CONFIG_TINYUSB_CDC_ENABLED) && \
    CONFIG_TINYUSB_CDC_ENABLED && __has_include(<tinyusb.h>)

#include "../../../freertos/hal/runtime/time.hpp"

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <tinyusb.h>
#if __has_include(<tinyusb_cdc_acm.h>)
#include <tinyusb_cdc_acm.h>
#else
#include <tusb_cdc_acm.h>
#endif

namespace m5::hal::v2::uart {

/*!
  @brief uart::IBus backed by USB OTG CDC ACM (TinyUSB).

  ESP32-S2, S3, and other chips with a USB OTG controller can use this
  as a UART-like transport via TinyUSB's CDC ACM class. The USB D+/D-
  pins are shared with the USB-Serial-JTAG peripheral; only one can be
  active per boot (selected by eFuse / menuconfig).

  Requires CONFIG_TINYUSB_CDC_ENABLED=y in sdkconfig. If the TinyUSB
  driver is not yet installed when init() is called, it will be installed
  with default configuration.

    m5hal::uart::Bus_espidf_usb_cdc cdc;
    cdc.init();
 */
namespace detail {
struct BusUsbCdcRxBridge;
}

class Bus_espidf_usb_cdc : public uart::Bus_streaming {
    friend struct detail::BusUsbCdcRxBridge;

public:
    ~Bus_espidf_usb_cdc() override
    {
        // teardownBackend() is bounded and may leave callback-
        // owned state alive on TIMEOUT_ERROR. Object destruction cannot return
        // while such a callback still holds `this`, so retry until teardown
        // completes instead of freeing the object behind it.
        while (_installed) {
            (void)teardownBackend();
        }
    }

    result_t<void> init(tinyusb_cdcacm_itf_t itf = TINYUSB_CDC_ACM_0);
    result_t<void> close(void)
    {
        return bus::IBus::close();
    }
    types::backend_kind_t backendKind(void) const override
    {
        return types::backend_kind_t::Hardware;
    }

protected:
    result_t<size_t> writeBackend(bus::OperationContext<uart::AccessConfig>& context, data::Source* src,
                                  size_t len) override;

    bus::CloseOutcome closeBackend(void) override
    {
        return teardownBackend();
    }

    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    bus::CloseOutcome teardownBackend(void);
    void onRxReady();

    static TickType_t ticks(uint32_t timeout_ms)
    {
        return ::m5::hal::v2::detail::timeoutMsToTicks(timeout_ms);
    }

    tinyusb_cdcacm_itf_t _itf = TINYUSB_CDC_ACM_0;
    SemaphoreHandle_t _rx_sem = nullptr;
    bool _installed           = false;
    bool _driver_owned        = false;
};

}  // namespace m5::hal::v2::uart

#endif  // SOC_USB_OTG_SUPPORTED && CONFIG_TINYUSB_CDC_ENABLED
#endif  // ESP_PLATFORM

#endif  // M5_HAL_VARIANTS_FRAMEWORKS_ESPIDF_HAL_UART_USB_CDC_HPP
