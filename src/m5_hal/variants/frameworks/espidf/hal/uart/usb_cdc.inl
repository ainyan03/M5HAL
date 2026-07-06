// SPDX-License-Identifier: MIT
#include "usb_cdc.hpp"

#if defined(ESP_PLATFORM) && defined(SOC_USB_OTG_SUPPORTED) && SOC_USB_OTG_SUPPORTED && \
    defined(CONFIG_TINYUSB_CDC_ENABLED) && CONFIG_TINYUSB_CDC_ENABLED && __has_include(<tinyusb.h>)

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
Bus_espidf_usb_cdc* s_usb_cdc_instances[TINYUSB_CDC_ACM_MAX] = {};

void usb_cdc_rx_callback(int itf, cdcacm_event_t* event)
{
    (void)event;
    if (itf >= 0 && itf < static_cast<int>(TINYUSB_CDC_ACM_MAX) && s_usb_cdc_instances[itf] != nullptr) {
        detail::BusUsbCdcRxBridge::onRx(s_usb_cdc_instances[itf]);
    }
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
    if (_installed) {
        (void)release();
    }

    _itf = itf;

    const tinyusb_config_t tusb_cfg = {};
    esp_err_t err                   = tinyusb_driver_install(&tusb_cfg);
    if (err == ESP_OK) {
        _driver_owned = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        return m5::stl::make_unexpected(mapEspErr(err));
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
        return m5::stl::make_unexpected(mapEspErr(err));
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

    s_usb_cdc_instances[static_cast<int>(_itf)] = this;
    _installed                                  = true;
    return {};
}

result_t<void> Bus_espidf_usb_cdc::release()
{
    if (!_installed) {
        return {};
    }

    s_usb_cdc_instances[static_cast<int>(_itf)] = nullptr;

    tinyusb_cdcacm_deinit(static_cast<int>(_itf));

    if (_rx_sem != nullptr) {
        vSemaphoreDelete(_rx_sem);
        _rx_sem = nullptr;
    }

    if (_driver_owned) {
        tinyusb_driver_uninstall();
        _driver_owned = false;
    }

    _installed = false;
    return {};
}

result_t<size_t> Bus_espidf_usb_cdc::write(bus::IAccessor* owner, const AccessConfig& cfg, data::Source* src,
                                           size_t len)
{
    auto result = Bus_streaming::write(owner, cfg, src, len);
    if (_installed) {
        esp_err_t err = tinyusb_cdcacm_write_flush(_itf, ticks(cfg.write_timeout_ms));
        if (err != ESP_OK && result.has_value()) {
            return m5::stl::make_unexpected(err == ESP_ERR_TIMEOUT ? error::error_t::TIMEOUT_ERROR
                                                                   : error::error_t::IO_ERROR);
        }
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
    if (flush_err != ESP_OK && flush_err != ESP_ERR_TIMEOUT) {
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
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
        return m5::stl::make_unexpected(mapEspErr(err));
    }
    if (rx_size > 0) {
        return rx_size;
    }
    if (_rx_sem != nullptr && xSemaphoreTake(_rx_sem, ticks(timeout_ms)) == pdTRUE) {
        err = tinyusb_cdcacm_read(_itf, buf, len, &rx_size);
        if (err != ESP_OK && err != ESP_FAIL) {
            return m5::stl::make_unexpected(mapEspErr(err));
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
    uint8_t tmp[1];
    size_t rx_size = 0;
    (void)tinyusb_cdcacm_read(_itf, tmp, 0, &rx_size);
    return rx_size > 0 ? static_cast<size_t>(1) : static_cast<size_t>(0);
}

error::error_t Bus_espidf_usb_cdc::mapEspErr(esp_err_t err)
{
    switch (err) {
        case ESP_OK:
            return error::error_t::OK;
        case ESP_ERR_INVALID_ARG:
        case ESP_ERR_INVALID_STATE:
            return error::error_t::INVALID_ARGUMENT;
        case ESP_ERR_TIMEOUT:
            return error::error_t::TIMEOUT_ERROR;
        case ESP_ERR_NO_MEM:
            return error::error_t::OUT_OF_RESOURCE;
        default:
            return error::error_t::IO_ERROR;
    }
}

}  // namespace m5::hal::v2::uart

#endif  // SOC_USB_OTG_SUPPORTED && CONFIG_TINYUSB_CDC_ENABLED
