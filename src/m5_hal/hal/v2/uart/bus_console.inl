// SPDX-License-Identifier: MIT
#ifndef M5_HAL_UART_BUS_CONSOLE_INL_
#define M5_HAL_UART_BUS_CONSOLE_INL_

#include "bus_console.hpp"

#if defined(ESP_PLATFORM)
#include <driver/uart.h>
#include <freertos/task.h>
#if defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED) && CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
#include <driver/usb_serial_jtag.h>
#endif
#if defined(SOC_USB_OTG_SUPPORTED) && SOC_USB_OTG_SUPPORTED && defined(CONFIG_TINYUSB_CDC_ENABLED) && \
    CONFIG_TINYUSB_CDC_ENABLED && __has_include(<tinyusb.h>)
#include <tinyusb.h>
#if __has_include(<tinyusb_cdc_acm.h>)
#include <tinyusb_cdc_acm.h>
#else
#include <tusb_cdc_acm.h>
#endif
#ifndef tinyusb_cdcacm_init
#define tinyusb_cdcacm_init(cfg) tusb_cdc_acm_init(cfg)
#endif
#define M5HAL_DETAIL_BUS_CONSOLE_HAS_USB_CDC_ 1
#endif
#elif !defined(_WIN32) && !defined(ARDUINO)
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <unistd.h>
#endif

namespace m5::hal::v2::uart {

namespace detail {
struct BusConsoleCdcBridge {
    static void onRx(Bus_console* c)
    {
        if (c) {
            c->onCdcRx();
        }
    }
};
}  // namespace detail

#if defined(M5HAL_DETAIL_BUS_CONSOLE_HAS_USB_CDC_)
namespace {
Bus_console* s_cdc_console                        = nullptr;
portMUX_TYPE s_cdc_console_mux                    = portMUX_INITIALIZER_UNLOCKED;
uint32_t s_cdc_console_rx_callback_in_flight      = 0;
constexpr uint32_t kReleaseCallbackDrainTimeoutMs = 100;

bool waitCdcConsoleRxCallbacksDrained()
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
        portENTER_CRITICAL_SAFE(&s_cdc_console_mux);
        in_flight = s_cdc_console_rx_callback_in_flight;
        portEXIT_CRITICAL_SAFE(&s_cdc_console_mux);
        if (in_flight == 0) {
            return true;
        }
        if ((xTaskGetTickCount() - start) >= budget) {
            return false;
        }
        vTaskDelay(1);
    }
}

void cdc_rx_callback(int itf, cdcacm_event_t* event)
{
    (void)itf;
    (void)event;

    Bus_console* console = nullptr;
    portENTER_CRITICAL_SAFE(&s_cdc_console_mux);
    console = s_cdc_console;
    if (console != nullptr) {
        ++s_cdc_console_rx_callback_in_flight;
    }
    portEXIT_CRITICAL_SAFE(&s_cdc_console_mux);

    if (console == nullptr) {
        return;
    }

    detail::BusConsoleCdcBridge::onRx(console);

    portENTER_CRITICAL_SAFE(&s_cdc_console_mux);
    --s_cdc_console_rx_callback_in_flight;
    portEXIT_CRITICAL_SAFE(&s_cdc_console_mux);
}
}  // namespace
#endif

void Bus_console::onCdcRx()
{
#if defined(ESP_PLATFORM)
    if (_rx_sem != nullptr) {
        xSemaphoreGiveFromISR(_rx_sem, nullptr);
    }
#endif
}

result_t<void> Bus_console::init(FILE* in, FILE* out)
{
    auto released = release();
    if (!released.has_value()) {
        return m5::stl::make_unexpected(released.error());
    }

    if (in == nullptr || out == nullptr) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
    _file_in  = in;
    _file_out = out;
    _fd_in    = fileno(in);
    _fd_out   = fileno(out);
    if (_fd_in < 0 || _fd_out < 0) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }

#if defined(ESP_PLATFORM)
    // Runtime auto-detection. When TinyUSB CDC is enabled, try OTG CDC
    // first (USB-JTAG driver install can succeed even when the phy is in
    // OTG mode, producing a silent dead path). Otherwise try USB-JTAG first.
#if defined(M5HAL_DETAIL_BUS_CONSOLE_HAS_USB_CDC_)
    {
        const tinyusb_config_t tusb_cfg = {};
        esp_err_t err                   = tinyusb_driver_install(&tusb_cfg);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            tinyusb_config_cdcacm_t cdc_cfg = {};
            cdc_cfg.cdc_port                = TINYUSB_CDC_ACM_0;
            cdc_cfg.callback_rx             = cdc_rx_callback;
            if (tinyusb_cdcacm_init(&cdc_cfg) == ESP_OK) {
                _rx_sem = xSemaphoreCreateBinary();
                portENTER_CRITICAL_SAFE(&s_cdc_console_mux);
                s_cdc_console = this;
                portEXIT_CRITICAL_SAFE(&s_cdc_console_mux);
                _transport = Transport::UsbCdc;
            }
        }
    }
#endif
#if defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED) && CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
    if (_transport == Transport::None) {
        usb_serial_jtag_driver_config_t jtag_cfg = {.tx_buffer_size = 1024, .rx_buffer_size = 1024};
        esp_err_t jtag_err                       = usb_serial_jtag_driver_install(&jtag_cfg);
        if (jtag_err == ESP_OK || jtag_err == ESP_ERR_INVALID_STATE) {
            _transport = Transport::UsbJtag;
        }
    }
#endif
    if (_transport == Transport::None) {
        const uart_port_t port = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
        if (!uart_is_driver_installed(port)) {
            uart_driver_install(port, 256, 0, 0, nullptr, 0);
        }
        _transport = Transport::Uart;
    }
#else
    _transport = Transport::Posix;
#endif

    return {};
}

// If release() returns TIMEOUT_ERROR, RX notifications are already stopped and
// release() must be called again to finish teardown. Resources are deliberately
// left allocated in that case: leaking a semaphore beats freeing one a callback
// still holds.
//
// The destructor cannot honor that contract — it discards the result, so an
// in-flight callback that outlives the drain budget still reaches a destroyed
// object. Clearing the global pointer bounds the exposure to callbacks that had
// already loaded it; no new callback can enter. Closing the remaining window
// would require blocking a destructor indefinitely.
result_t<void> Bus_console::release()
{
    if (_transport == Transport::None) {
        return {};
    }

#if defined(ESP_PLATFORM)
#if defined(M5HAL_DETAIL_BUS_CONSOLE_HAS_USB_CDC_)
    if (_transport == Transport::UsbCdc) {
        (void)tinyusb_cdcacm_unregister_callback(TINYUSB_CDC_ACM_0, CDC_EVENT_RX);

        portENTER_CRITICAL_SAFE(&s_cdc_console_mux);
        if (s_cdc_console == this) {
            s_cdc_console = nullptr;
        }
        portEXIT_CRITICAL_SAFE(&s_cdc_console_mux);

        if (!waitCdcConsoleRxCallbacksDrained()) {
            return m5::stl::make_unexpected(error::error_t::TIMEOUT_ERROR);
        }
    }
#endif
    if (_rx_sem != nullptr) {
        vSemaphoreDelete(_rx_sem);
        _rx_sem = nullptr;
    }
#endif
    _file_in   = nullptr;
    _file_out  = nullptr;
    _fd_in     = -1;
    _fd_out    = -1;
    _transport = Transport::None;
    return {};
}

result_t<size_t> Bus_console::rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    if (_transport == Transport::None) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if defined(ESP_PLATFORM)
    switch (_transport) {
#if defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED) && CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
        case Transport::UsbJtag: {
            const int n = usb_serial_jtag_write_bytes(data, len, pdMS_TO_TICKS(timeout_ms));
            return (n > 0) ? static_cast<size_t>(n) : static_cast<size_t>(0);
        }
#endif
#if defined(M5HAL_DETAIL_BUS_CONSOLE_HAS_USB_CDC_)
        case Transport::UsbCdc: {
            const size_t queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, len);
            if (queued > 0) {
                (void)tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(timeout_ms));
            }
            return queued;
        }
#endif
        case Transport::Uart: {
            const uart_port_t port = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
            const int n            = uart_write_bytes(port, data, len);
            (void)uart_wait_tx_done(port, pdMS_TO_TICKS(timeout_ms));
            return (n > 0) ? static_cast<size_t>(n) : static_cast<size_t>(0);
        }
        default:
            return static_cast<size_t>(0);
    }
#elif !defined(_WIN32) && !defined(ARDUINO)
    (void)timeout_ms;
    size_t done = ::fwrite(data, 1, len, _file_out);
    ::fflush(_file_out);
    return done;
#else
    (void)data;
    (void)len;
    (void)timeout_ms;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
#endif
}

result_t<size_t> Bus_console::rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    if (_transport == Transport::None) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if defined(ESP_PLATFORM)
    switch (_transport) {
#if defined(CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED) && CONFIG_SOC_USB_SERIAL_JTAG_SUPPORTED
        case Transport::UsbJtag: {
            const int n = usb_serial_jtag_read_bytes(buf, len, pdMS_TO_TICKS(timeout_ms));
            return (n > 0) ? static_cast<size_t>(n) : static_cast<size_t>(0);
        }
#endif
#if defined(M5HAL_DETAIL_BUS_CONSOLE_HAS_USB_CDC_)
        case Transport::UsbCdc: {
            size_t rx_size = 0;
            (void)tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf, len, &rx_size);
            if (rx_size > 0) {
                return rx_size;
            }
            if (_rx_sem != nullptr && xSemaphoreTake(_rx_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
                (void)tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, buf, len, &rx_size);
                return rx_size;
            }
            return static_cast<size_t>(0);
        }
#endif
        case Transport::Uart: {
            const uart_port_t port = static_cast<uart_port_t>(CONFIG_ESP_CONSOLE_UART_NUM);
            const int n            = uart_read_bytes(port, buf, len, pdMS_TO_TICKS(timeout_ms));
            return (n > 0) ? static_cast<size_t>(n) : static_cast<size_t>(0);
        }
        default:
            return static_cast<size_t>(0);
    }
#elif !defined(_WIN32) && !defined(ARDUINO)
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(_fd_in, &fds);
    struct timeval tv;
    tv.tv_sec  = static_cast<time_t>(timeout_ms / 1000);
    tv.tv_usec = static_cast<suseconds_t>((timeout_ms % 1000) * 1000);
    int ready  = ::select(_fd_in + 1, &fds, nullptr, nullptr, &tv);
    if (ready < 0) {
        if (errno == EINTR) {
            return static_cast<size_t>(0);
        }
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    if (ready == 0) {
        return static_cast<size_t>(0);
    }
    ssize_t n = ::read(_fd_in, buf, len);
    if (n < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            return static_cast<size_t>(0);
        }
        return m5::stl::make_unexpected(error::error_t::IO_ERROR);
    }
    return static_cast<size_t>(n);
#else
    (void)buf;
    (void)len;
    (void)timeout_ms;
    return m5::stl::make_unexpected(error::error_t::NOT_IMPLEMENTED);
#endif
}

result_t<size_t> Bus_console::rawReadableBytes()
{
    if (_transport == Transport::None) {
        return m5::stl::make_unexpected(error::error_t::INVALID_ARGUMENT);
    }
#if !defined(_WIN32) && !defined(ESP_PLATFORM) && !defined(ARDUINO)
    int avail = 0;
    if (::ioctl(_fd_in, FIONREAD, &avail) != 0 || avail < 0) {
        return static_cast<size_t>(0);
    }
    return static_cast<size_t>(avail);
#else
    return static_cast<size_t>(0);
#endif
}

}  // namespace m5::hal::v2::uart

#endif  // M5_HAL_UART_BUS_CONSOLE_INL_
