// =============================================================================
// M5HAL — HowToUseRemoteI2S (device side)
//
// Turns an M5Stack Core2 V1.1 into a remote I2S playback server: a PC
// connected over the USB serial cable streams raw PCM audio to this board's
// speaker as if the I2S bus were local. The counterpart host program lives
// in host/remote_i2s_host.cpp (built on the PC with the posix UART variant).
//
// How it works (spec/design/remote.md §stream credit):
//   - UART0 (the USB bridge) carries framed remote messages at 3,000,000 baud.
//     There is deliberately NO console logging on UART0: the channel belongs
//     to the protocol.
//   - A remote::Server registers ONE I2S TX accessor (bus_id 0). Registration
//     = allowlist: that is all a remote peer can reach. The host's
//     RemoteI2SBus sends NORESP bus_write_stream bursts paced by stream credit;
//     the server's pollStreamCredit reports the DMA drain back as credit events.
//   - remote::RemoteServerService polls the link cooperatively from a
//     service::ServiceRunner in the main loop.
//
// Speaker / I2S setup is shared with the I2SAudio example (Core2 V1.1):
//   - Amplifier: AXP2101 ALDO3 = 3300 mV (reg 0x94 = 0x1C, reg 0x90 bit 2)
//   - I2S pins:  BCK=12, WS=0, DOUT=2 (16-bit, Philips standard)
//
// Framework: ESP-IDF only. The Core2 V1.1 I2S backend needs the IDF gen5
// driver (driver/i2s_std.h), and UART0 is driven through the espidf UART
// variant. The PIO build-check env (HowToUse_RemoteI2S_esp32) builds this with
// the espidf framework; there is no Arduino variant for this sketch (see the
// pio_envs note and the build-check workflow).
// =============================================================================

#include <M5HAL_v1.hpp>

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace m5hal = m5::hal::v1;

namespace {

// ---- pins / addresses (Core2 V1.1) ----
constexpr int PIN_UART_TX      = 1;   // UART0 TX (USB bridge) on classic ESP32
constexpr int PIN_UART_RX      = 3;   // UART0 RX (USB bridge)
constexpr int PIN_I2C_SCL      = 22;  // external I2C shared with the PMIC
constexpr int PIN_I2C_SDA      = 21;
constexpr int PIN_I2S_BCK      = 12;
constexpr int PIN_I2S_WS       = 0;
constexpr int PIN_I2S_DOUT     = 2;
constexpr uint8_t AXP2101_ADDR = 0x34;

// UART0 carries the protocol at a high baud so audio keeps up. first_byte
// timeout is tiny: the server polls cooperatively and a StreamSource peek
// blocks up to first_byte_timeout when the line is idle.
constexpr uint32_t PROTOCOL_BAUD = 3000000;

// ---- the protocol link (UART0 over USB) ----
m5hal::uart::variant::espidf::Bus uart_bus;
m5hal::uart::UARTAccessConfig uart_cfg;

// ---- the published bus (internal I2S to the speaker) ----
m5hal::i2s::Bus i2s_bus;
m5hal::i2s::I2SBusConfig i2s_bus_cfg;
m5hal::i2s::I2SAccessConfig i2s_acc_cfg;

// ---- the external I2C (only used to enable the amplifier) ----
m5hal::i2c::Bus i2c_bus;

// ---- the server ----
uint8_t server_scratch[m5hal::remote::kMaxMessageSize];
uint8_t rx_scratch[m5hal::frame::kMaxWireSize];
uint8_t tx_scratch[m5hal::frame::kMaxWireSize];
m5hal::service::ServiceRunner runner;

// Enable the Core2 V1.1 speaker amplifier: AXP2101 ALDO3 -> 3300 mV.
// Source: I2SAudio example (M5Unified AXP2101 branch).
void enableAmplifier()
{
    m5hal::i2c::I2CMasterAccessConfig cfg;
    cfg.i2c_addr = AXP2101_ADDR;
    cfg.freq     = 400000;
    m5hal::i2c::I2CMasterAccessor acc{i2c_bus, cfg};

    // reg 0x94 = (3300 - 500) / 100 = 0x1C  (ALDO3 = 3300 mV)
    uint8_t set_voltage[2] = {0x94, 0x1C};
    (void)acc.write(set_voltage, sizeof(set_voltage));

    // reg 0x90 bit 2 = ALDO3 enable (read-modify-write)
    uint8_t reg90 = 0;
    if (acc.readRegister(static_cast<int>(0x90), &reg90, 1).has_value()) {
        uint8_t enable[2] = {0x90, static_cast<uint8_t>(reg90 | 0x04)};
        (void)acc.write(enable, sizeof(enable));
    }
}

void deviceInit()
{
    // UART0 doubles as the ESP-IDF log console; any runtime ESP_LOG output
    // (e.g. a driver warning) would corrupt the protocol stream. Silence it.
    esp_log_level_set("*", ESP_LOG_NONE);

    // ---- external I2C (PMIC) ----
    m5hal::i2c::BusConfig i2c_cfg{PIN_I2C_SCL, PIN_I2C_SDA};
    (void)i2c_bus.init(i2c_cfg);
    enableAmplifier();

    // ---- I2S TX to the speaker ----
    i2s_bus_cfg.pin_bclk       = PIN_I2S_BCK;
    i2s_bus_cfg.pin_ws         = PIN_I2S_WS;
    i2s_bus_cfg.pin_dout       = PIN_I2S_DOUT;
    // Generous DMA so every supported config rides out the link's occasional
    // rough second (USB scheduling; ~30 KB momentary supply deficit observed
    // at 96 KB/s). The floor is much lower: 24 kHz mono runs clean on a
    // 16 KiB DMA + 6 KiB in-flight window (see spec/design/remote.md
    // §stream credit for the sizing rule).
    i2s_bus_cfg.tx_buffer_size = 49152;
    (void)i2s_bus.init(i2s_bus_cfg);

    i2s_acc_cfg.sample_rate_hz   = 44100;
    i2s_acc_cfg.bits_per_sample  = 16;
    i2s_acc_cfg.channels         = 2;
    i2s_acc_cfg.write_timeout_ms = 0;  // the remote side drives flow via credit

    static m5hal::i2s::I2STxAccessor i2s_acc{i2s_bus, i2s_acc_cfg};

    // ---- UART0 protocol link (espidf variant) ----
    m5hal::uart::variant::espidf::BusConfig bus_cfg;
    bus_cfg.port_num = 0;  // UART0 = USB bridge
    bus_cfg.pin_tx   = PIN_UART_TX;
    bus_cfg.pin_rx   = PIN_UART_RX;
    // The host bursts up to its in-flight window at 3 Mbaud; the rx ring must
    // absorb such a burst (plus framing) while the service loop is busy, or
    // NORESP writes are silently lost to overflow (audio gaps).
    bus_cfg.rx_buffer_size = 49152;
    bus_cfg.tx_buffer_size = 2048;
    (void)uart_bus.init(bus_cfg);

    uart_cfg.baud_rate             = PROTOCOL_BAUD;
    uart_cfg.first_byte_timeout_ms = 2;
    uart_cfg.inter_byte_timeout_ms = 1;
    uart_cfg.write_timeout_ms      = 100;

    static m5hal::uart::UARTTxAccessor uart_tx{uart_bus, uart_cfg};
    static m5hal::uart::UARTRxAccessor uart_rx{uart_bus, uart_cfg};

    // ---- server: publish I2S bus_id 0 (and nothing else) ----
    static m5hal::remote::Server srv{m5hal::data::DataSpan{server_scratch, sizeof(server_scratch)}};
    (void)srv.registerI2S(0, i2s_acc);

    static m5hal::data::StreamSource link_src{uart_rx, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    static m5hal::data::StreamSink link_snk{uart_tx, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
    static m5hal::remote::RemoteServerService sv{srv, link_src, link_snk};
    runner.add(sv);
}

}  // namespace

// ---------------------------------------------------------------------------
// Arduino entry points (build via espidf framework keeps these in app_main).
// ---------------------------------------------------------------------------

#ifdef ARDUINO

void setup()
{
    deviceInit();
}

void loop()
{
    runner.runOnce();
}

#else  // ESP-IDF

extern "C" void app_main(void)
{
    deviceInit();
    while (true) {
        // Sustained 16-bit/44.1kHz stereo needs ~750 frames/s through the
        // server. A vTaskDelay(1) per poll (10 ms at the default 100 Hz tick)
        // would cap throughput far below that and overflow the rx ring, so
        // batch many polls per yield. When idle each poll blocks for the UART
        // first_byte timeout (2 ms), which also lets the idle task run.
        for (int i = 0; i < 50; ++i) {
            runner.runOnce();
        }
        vTaskDelay(1);  // let the idle task / task watchdog breathe
    }
}

#endif
