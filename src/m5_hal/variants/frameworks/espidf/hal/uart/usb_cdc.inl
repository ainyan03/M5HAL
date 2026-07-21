// SPDX-License-Identifier: MIT
#include "usb_cdc.hpp"
#include "error.hpp"

#if defined(ESP_PLATFORM) && defined(SOC_USB_OTG_SUPPORTED) && SOC_USB_OTG_SUPPORTED && \
    defined(CONFIG_TINYUSB_CDC_ENABLED) && CONFIG_TINYUSB_CDC_ENABLED && __has_include(<tinyusb.h>)

#include <freertos/task.h>

// esp_tinyusb v2 uses tusb_cdc_acm_init/deinit; v2 renamed to tinyusb_cdcacm_init/deinit.
#ifndef tinyusb_cdcacm_init
#define tinyusb_cdcacm_init(cfg) tusb_cdc_acm_init(cfg)
#endif
#ifndef tinyusb_cdcacm_deinit
#define tinyusb_cdcacm_deinit(itf) tusb_cdc_acm_deinit(itf)
#endif

namespace m5::hal::v2::uart {

namespace detail {
struct BusUsbCdcRxBridge {
    static void onRx(Bus_espidf_usb_cdc* c)
    {
        if (c) {
            c->onRxReady();
        }
    }
};
}  // namespace detail

namespace {
Bus_espidf_usb_cdc* s_usb_cdc_instances[TINYUSB_CDC_ACM_MAX]  = {};
portMUX_TYPE s_usb_cdc_mux                                    = portMUX_INITIALIZER_UNLOCKED;
uint32_t s_usb_cdc_rx_callback_in_flight[TINYUSB_CDC_ACM_MAX] = {};
constexpr uint32_t kReleaseCallbackDrainTimeoutMs             = 100;

bool waitUsbCdcRxCallbacksDrained(int itf)
{
    // pdMS_TO_TICKS(1) truncates to 0 at the default 100 Hz tick rate, where
    // vTaskDelay(0) only yields. Delay a whole tick and bound the wait by
    // elapsed ticks. The budget itself truncates to 0 below 10 Hz, so floor it
    // at one tick to keep the wait non-empty at any configTICK_RATE_HZ.
    const TickType_t start     = xTaskGetTickCount();
    const TickType_t raw_ticks = pdMS_TO_TICKS(kReleaseCallbackDrainTimeoutMs);
    const TickType_t budget    = (raw_ticks != 0) ? raw_ticks : 1;
    for (;;) {
        uint32_t in_flight = 0;
        portENTER_CRITICAL_SAFE(&s_usb_cdc_mux);
        in_flight = s_usb_cdc_rx_callback_in_flight[itf];
        portEXIT_CRITICAL_SAFE(&s_usb_cdc_mux);
        if (in_flight == 0) {
            return true;
        }
        if ((xTaskGetTickCount() - start) >= budget) {
            return false;
        }
        vTaskDelay(1);
    }
}

void usb_cdc_rx_callback(int itf, cdcacm_event_t* event)
{
    (void)event;
    if (itf < 0 || itf >= static_cast<int>(TINYUSB_CDC_ACM_MAX)) {
        return;
    }

    Bus_espidf_usb_cdc* instance = nullptr;
    portENTER_CRITICAL_SAFE(&s_usb_cdc_mux);
    instance = s_usb_cdc_instances[itf];
    if (instance != nullptr) {
        ++s_usb_cdc_rx_callback_in_flight[itf];
    }
    portEXIT_CRITICAL_SAFE(&s_usb_cdc_mux);

    if (instance == nullptr) {
        return;
    }

    detail::BusUsbCdcRxBridge::onRx(instance);

    portENTER_CRITICAL_SAFE(&s_usb_cdc_mux);
    --s_usb_cdc_rx_callback_in_flight[itf];
    portEXIT_CRITICAL_SAFE(&s_usb_cdc_mux);
}
}  // namespace

void Bus_espidf_usb_cdc::onRxReady()
{
    if (_rx_sem != nullptr) {
        xSemaphoreGiveFromISR(_rx_sem, nullptr);
    }
}

result_t<void> Bus_espidf_usb_cdc::init(tinyusb_cdcacm_itf_t itf)
{
    if (!initializationAllowed(false)) {
        return m5::stl::make_unexpected(error::error_t::INVALID_STATE);
    }

    if (_installed) {
        auto closed = teardownBackend();
        if (closed.disposition != bus::CloseDisposition::Success) {
            return m5::stl::make_unexpected(closed.error_code);
        }
    }

    _itf = itf;

    const tinyusb_config_t tusb_cfg = {};
    esp_err_t err                   = tinyusb_driver_install(&tusb_cfg);
    if (err == ESP_OK) {
        _driver_owned = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        return m5::stl::make_unexpected(impl_espidf::mapEspErr(err));
    }

    tinyusb_config_cdcacm_t cdc_cfg = {};
    cdc_cfg.cdc_port                = _itf;
    cdc_cfg.callback_rx             = usb_cdc_rx_callback;
    err                             = tinyusb_cdcacm_init(&cdc_cfg);
    if (err != ESP_OK) {
        if (_driver_owned) {
            tinyusb_driver_uninstall();
            _driver_owned = false;
        }
        return m5::stl::make_unexpected(impl_espidf::mapEspErr(err));
    }

    _rx_sem = xSemaphoreCreateBinary();
    if (_rx_sem == nullptr) {
        tinyusb_cdcacm_deinit(static_cast<int>(_itf));
        if (_driver_owned) {
            tinyusb_driver_uninstall();
            _driver_owned = false;
        }
        return m5::stl::make_unexpected(error::error_t::OUT_OF_RESOURCE);
    }

    portENTER_CRITICAL_SAFE(&s_usb_cdc_mux);
    s_usb_cdc_instances[static_cast<int>(_itf)] = this;
    portEXIT_CRITICAL_SAFE(&s_usb_cdc_mux);

    _installed       = true;
    auto initialized = markInitializationSucceeded(false);
    if (!initialized) {
        (void)teardownBackend();
        return initialized;
    }
    return {};
}

// If teardownBackend() returns TIMEOUT_ERROR, RX notifications are already
// stopped and teardownBackend() must be called again to finish. Resources are
// deliberately left allocated in that case: leaking a semaphore beats freeing
// one a callback still holds.
//
// The destructor retries this bounded operation until callback drain completes;
// it therefore never frees the object while a callback still owns `this`.
bus::CloseOutcome Bus_espidf_usb_cdc::teardownBackend(void)
{
    if (!_installed) {
        return bus::CloseOutcome::success();
    }

    const int itf = static_cast<int>(_itf);
    (void)tinyusb_cdcacm_unregister_callback(_itf, CDC_EVENT_RX);

    portENTER_CRITICAL_SAFE(&s_usb_cdc_mux);
    s_usb_cdc_instances[itf] = nullptr;
    portEXIT_CRITICAL_SAFE(&s_usb_cdc_mux);

    if (!waitUsbCdcRxCallbacksDrained(itf)) {
        return bus::CloseOutcome::partialOrUnknown(error::error_t::TIMEOUT_ERROR);
    }

    tinyusb_cdcacm_deinit(itf);

    if (_rx_sem != nullptr) {
        vSemaphoreDelete(_rx_sem);
        _rx_sem = nullptr;
    }

    if (_driver_owned) {
        tinyusb_driver_uninstall();
        _driver_owned = false;
    }

    _installed = false;
    return bus::CloseOutcome::success();
}

result_t<size_t> Bus_espidf_usb_cdc::writeBackend(bus::OperationContext<AccessConfig>& context, data::Source* src,
                                                  size_t len)
{
    auto result = Bus_streaming::writeBackend(context, src, len);
    if (_installed) {
        esp_err_t err = tinyusb_cdcacm_write_flush(_itf, ticks(context.config.write_timeout_ms));
        return completeWrite(std::move(result), impl_espidf::mapEspErr(err));
    }
    return result;
}

result_t<size_t> Bus_espidf_usb_cdc::rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (!_installed) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    const size_t queued = tinyusb_cdcacm_write_queue(_itf, data, len);
    if (queued == 0) {
        return static_cast<size_t>(0);
    }
    esp_err_t flush_err = tinyusb_cdcacm_write_flush(_itf, ticks(timeout_ms));
    // The queue already owns this prefix. Completion failure is reconciled by
    // write() after the base loop advances Source by the accepted byte count.
    (void)flush_err;
    return queued;
}

result_t<size_t> Bus_espidf_usb_cdc::rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    if (!_installed) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    size_t rx_size = 0;
    esp_err_t err  = tinyusb_cdcacm_read(_itf, buf, len, &rx_size);
    if (err != ESP_OK && err != ESP_FAIL) {
        return m5::stl::make_unexpected(impl_espidf::mapEspErr(err));
    }
    if (rx_size > 0) {
        return rx_size;
    }
    if (_rx_sem != nullptr && xSemaphoreTake(_rx_sem, ticks(timeout_ms)) == pdTRUE) {
        err = tinyusb_cdcacm_read(_itf, buf, len, &rx_size);
        if (err != ESP_OK && err != ESP_FAIL) {
            return m5::stl::make_unexpected(impl_espidf::mapEspErr(err));
        }
        return rx_size;
    }
    return static_cast<size_t>(0);
}

result_t<size_t> Bus_espidf_usb_cdc::rawReadableBytes()
{
    if (!_installed) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    return static_cast<size_t>(tud_cdc_n_available(static_cast<uint8_t>(_itf)));
}

}  // namespace m5::hal::v2::uart

#endif  // SOC_USB_OTG_SUPPORTED && CONFIG_TINYUSB_CDC_ENABLED
