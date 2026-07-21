// SPDX-License-Identifier: MIT
//
// HIL device for Bus_console::readableBytes(). It echoes only bytes first
// reported as immediately readable, so the regular UART echo host also proves
// that the console backend does not return a permanent zero.

#include <M5HAL_v2.hpp>
#include <m5_hal/hal/v2/uart/bus_console.hpp>

namespace m5hal = m5::hal::v2;

namespace {

m5hal::uart::Bus_console console_bus;
m5hal::uart::AccessConfig console_config;

}  // namespace

extern "C" void app_main()
{
    if (!console_bus.init().has_value()) {
        return;
    }

    console_config.baud_rate             = 115200;
    console_config.first_byte_timeout_ms = 2;
    console_config.inter_byte_timeout_ms = 1;
    console_config.write_timeout_ms      = 100;

    m5hal::uart::RxAccessor rx{console_bus, console_config};
    m5hal::uart::TxAccessor tx{console_bus, console_config};
    uint8_t buffer[1024];

    for (;;) {
        auto readable = rx.readableBytes();
        if (!readable.has_value() || readable.value() == 0) {
            m5hal::runtime::delayMs(1);
            continue;
        }

        const size_t requested = (readable.value() < sizeof(buffer)) ? readable.value() : sizeof(buffer);
        auto received          = rx.read(buffer, requested);
        if (received.has_value() && received.value() > 0) {
            (void)tx.write(buffer, received.value());
        }
    }
}
