// SPDX-License-Identifier: MIT
#ifndef M5_HAL_UART_BUS_CONSOLE_HPP_
#define M5_HAL_UART_BUS_CONSOLE_HPP_

#include "bus_streaming.hpp"

#include <cstdio>
#include <memory>

namespace m5::hal::v2::uart {

/*!
  @brief uart::IBus backed by the platform console transport.

  On ESP32, runtime auto-detects the transport: tries USB OTG CDC
  (TinyUSB), then USB-Serial-JTAG, then falls back to UART. On a
  POSIX host, uses select()+read()/write() on stdin/stdout.

    m5hal::uart::Bus_console con;
    con.init();   // auto-detect
 */
class Bus_console : public Bus_streaming {
    enum class Transport { None, Delegate, Uart, Posix };

public:
    ~Bus_console() override
    {
        (void)teardownBackend();
    }

    [[nodiscard]] result_t<void> init(FILE* in = stdin, FILE* out = stdout);
    [[nodiscard]] result_t<void> close(void)
    {
        return bus::IBus::close();
    }
    types::backend_kind_t backendKind(void) const override
    {
#if defined(_WIN32) || (defined(ARDUINO) && !defined(ESP_PLATFORM))
        return types::backend_kind_t::Software;
#else
        if (_delegate != nullptr) {
            return _delegate->backendKind();
        }
        return (_transport == Transport::Uart || _transport == Transport::Posix) ? types::backend_kind_t::Hardware
                                                                                 : types::backend_kind_t::Software;
#endif
    }
    bus::BusCapabilities capabilities(void) const override
    {
#if defined(_WIN32) || (defined(ARDUINO) && !defined(ESP_PLATFORM))
        return bus::IBus::capabilities();
#else
        return _delegate != nullptr ? _delegate->capabilities() : Bus_streaming::capabilities();
#endif
    }

protected:
    result_t<void> beginOperationBackend(bus::OperationContext<AccessConfig>& context) override;
    result_t<void> endOperationBackend(bus::OperationContext<AccessConfig>& context) override;
    result_t<size_t> writeBackend(bus::OperationContext<AccessConfig>& context, data::Source* src, size_t len) override;
    result_t<size_t> readBackend(bus::OperationContext<AccessConfig>& context, data::Sink* dst, size_t len) override;
    result_t<size_t> readableBytesBackend(bus::OperationContext<AccessConfig>& context) override;

    bus::CloseOutcome closeBackend(void) override;
    result_t<size_t> rawWrite(const uint8_t* data, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawRead(uint8_t* buf, size_t len, uint32_t timeout_ms) override;
    result_t<size_t> rawReadableBytes() override;

private:
    bus::CloseOutcome teardownBackend(void);

    FILE* _file_in       = nullptr;
    FILE* _file_out      = nullptr;
    int _fd_in           = -1;
    int _fd_out          = -1;
    Transport _transport = Transport::None;
    std::unique_ptr<IBus> _delegate;
};

}  // namespace m5::hal::v2::uart

#endif  // M5_HAL_UART_BUS_CONSOLE_HPP_
