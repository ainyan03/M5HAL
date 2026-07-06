// SPDX-License-Identifier: MIT
#ifndef M5_HAL_UART_BUS_CONSOLE_HPP_
#define M5_HAL_UART_BUS_CONSOLE_HPP_

#include "bus_streaming.hpp"

#include <cstdio>

#if defined(ESP_PLATFORM)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

namespace m5::hal::v2::uart {

namespace detail {
struct BusConsoleCdcBridge;
}

/*!
  @brief uart::IBus backed by the platform console transport.

  On ESP32, runtime auto-detects the transport: tries USB-Serial-JTAG
  driver, then USB OTG CDC (TinyUSB), then falls back to UART. On a
  POSIX host, uses select()+read()/write() on stdin/stdout.

    m5hal::uart::Bus_console con;
    con.init();   // auto-detect
 */
class Bus_console : public Bus_streaming {
    friend struct detail::BusConsoleCdcBridge;

public:
    ~Bus_console() override
    {
        (void)release();
    }

    result_t<void> init(FILE* in = stdin, FILE* out = stdout);
    result_t<void> release() override;

protected:
    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    void onCdcRx();

    enum class Transport { None, Uart, UsbJtag, UsbCdc, Posix };

    FILE* _file_in       = nullptr;
    FILE* _file_out      = nullptr;
    int _fd_in           = -1;
    int _fd_out          = -1;
    Transport _transport = Transport::None;
#if defined(ESP_PLATFORM)
    SemaphoreHandle_t _rx_sem = nullptr;
#endif
};

}  // namespace m5::hal::v2::uart

#endif  // M5_HAL_UART_BUS_CONSOLE_HPP_
