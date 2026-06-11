// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL — RemoteMenu device side
//
// Permanent acceptance harness for decisions/022-024.  Combines the four
// former Remote examples (RemoteBus / RemoteI2S / RemoteBusTcp /
// RemoteI2STcp, removed from examples/) into a single translation unit.
//
// --- Three-axis branching ---
//   transport : M5HAL_EXPERIMENT_REMOTE_TCP defined = WiFi/TCP, undefined = UART0
//   framework : M5HAL_FRAMEWORK_HAS_ARDUINO (1 = arduino-esp32, 0 = pure ESP-IDF)
//   I2S       : compile-gated by ESP_PLATFORM && M5HAL_ESPIDF_I2S_HAS_STD, plus
//               run-time AXP2101 (0x34) probe success = Core2 V1.1 only
//
// --- PIO envs covered by this file ---
//   v1_exp_remote_arduino      UART / arduino
//   v1_exp_remote_idf5         UART / espidf
//   v1_exp_remote_tcp_arduino  TCP  / arduino
//   v1_exp_remote_tcp_idf5     TCP  / espidf
//   (PC side: v1_exp_remote_host)
//
// --- WiFi credentials ---
//   Injected from env vars M5HAL_WIFI_SSID / M5HAL_WIFI_PASS as
//   M5HAL_EXPERIMENT_WIFI_SSID / M5HAL_EXPERIMENT_WIFI_PASS.
//   The pio env sets '-DM5HAL_EXPERIMENT_WIFI_SSID="${sysenv.M5HAL_WIFI_SSID}"'.
// =============================================================================

#include <M5HAL_v1.hpp>

// ---------------------------------------------------------------------------
// Additional includes for the active transport / framework combination
// ---------------------------------------------------------------------------
#if defined(M5HAL_EXPERIMENT_REMOTE_TCP)

#if M5HAL_FRAMEWORK_HAS_ARDUINO
// WiFi is the SKETCH's opt-in, never M5HAL's: the library includes no
// <WiFi.h> and references no esp_wifi symbol, so a sketch that does not
// bring the network up itself links no WiFi stack at all.
#include <WiFi.h>
#endif  // M5HAL_FRAMEWORK_HAS_ARDUINO

#if !M5HAL_FRAMEWORK_HAS_ARDUINO
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

#include <atomic>
#include <cstring>
#endif  // !M5HAL_FRAMEWORK_HAS_ARDUINO

#else  // UART transport

#if !M5HAL_FRAMEWORK_HAS_ARDUINO
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif  // !M5HAL_FRAMEWORK_HAS_ARDUINO

#endif  // M5HAL_EXPERIMENT_REMOTE_TCP

#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include <Arduino.h>
#include <Wire.h>
#endif

// ---------------------------------------------------------------------------
// Default values for build-configuration defines
// ---------------------------------------------------------------------------
#ifndef M5HAL_EXPERIMENT_WIFI_SSID
#define M5HAL_EXPERIMENT_WIFI_SSID ""
#endif
#ifndef M5HAL_EXPERIMENT_WIFI_PASS
#define M5HAL_EXPERIMENT_WIFI_PASS ""
#endif
#ifndef M5HAL_EXPERIMENT_TCP_PORT
#define M5HAL_EXPERIMENT_TCP_PORT 5555
#endif

// Default baud rate: 115200 for arduino, 3000000 for espidf (framework branch)
#ifndef M5HAL_EXPERIMENT_REMOTE_BAUD
#if M5HAL_FRAMEWORK_HAS_ARDUINO
#define M5HAL_EXPERIMENT_REMOTE_BAUD 115200
#else
#define M5HAL_EXPERIMENT_REMOTE_BAUD 3000000
#endif
#endif

// ---------------------------------------------------------------------------
// Namespace aliases
// ---------------------------------------------------------------------------
namespace m5hal = m5::hal::v1;
// The TCP transport is not a bus kind, so it has no `variant::` alias —
// name the espidf variant namespace directly (it is written against the
// BSD socket API lwIP serves to both frameworks, so arduino uses it too).
namespace dev_tcp = m5::variants::frameworks::espidf::hal::v1::tcp;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr int PIN_I2C_SCL      = 22;  // Core common internal I2C
static constexpr int PIN_I2C_SDA      = 21;
static constexpr int PIN_I2S_BCK      = 12;  // Core2 V1.1 speaker (exposed on M-BUS)
static constexpr int PIN_I2S_WS       = 0;
static constexpr int PIN_I2S_DOUT     = 2;
static constexpr uint8_t AXP2101_ADDR = 0x34;
static constexpr int PIN_UART_TX      = 1;  // UART0 TX (USB bridge)
static constexpr int PIN_UART_RX      = 3;  // UART0 RX (USB bridge)

// GPIO allowlist: expose only Core BASIC buttons A/B/C
static constexpr uint8_t PUBLISHED_PINS[] = {39, 38, 37};

// ---------------------------------------------------------------------------
// AllowlistGPIO — GPIO safety-boundary filter
//
// Forwards an IGPIO but resolves only the allowlisted pins: everything
// else looks unconnected to the remote peer. This is the safety-boundary
// pattern from spec/design/remote.md — publish a dedicated, filtered
// view instead of the whole pin bank.
// ---------------------------------------------------------------------------
class AllowlistGPIO : public m5hal::gpio::IGPIO {
public:
    AllowlistGPIO(const m5hal::gpio::IGPIO* inner, const uint8_t* pins, size_t count)
        : _inner{inner}, _pins{pins}, _count{count}
    {
    }

    m5hal::gpio::IPort* portForPin(m5hal::types::gpio_local_pin_t pin_index) const override
    {
        return allowed(pin_index) ? _inner->portForPin(pin_index) : nullptr;
    }
    m5hal::gpio::IPort* getPort(uint8_t) const override
    {
        return nullptr;  // no whole-port access through the allowlist
    }
    uint16_t getPinCount() const override
    {
        return _inner->getPinCount();  // numbering stays the physical one
    }
    uint8_t getPortCount() const override
    {
        return 0;
    }
    bool isValid(m5hal::types::gpio_local_pin_t pin_index) const override
    {
        return allowed(pin_index) && _inner->isValid(pin_index);
    }

private:
    bool allowed(m5hal::types::gpio_local_pin_t pin_index) const
    {
        for (size_t i = 0; i < _count; ++i) {
            if (_pins[i] == pin_index) {
                return true;
            }
        }
        return false;
    }

    const m5hal::gpio::IGPIO* _inner = nullptr;
    const uint8_t* _pins             = nullptr;
    size_t _count                    = 0;
};

// ---------------------------------------------------------------------------
// Shared resources
// ---------------------------------------------------------------------------
static m5hal::i2c::Bus i2c_bus;
static m5hal::i2c::I2CMasterAccessConfig i2c_acc_cfg;

// server_scratch: working buffer where the server unpacks received messages
static uint8_t server_scratch[m5hal::remote::kMaxMessageSize];

// RX scratch sized for a BATCH of frames, not one: with a one-frame scratch
// the serve loop pays one driver read per frame, and that per-read quantum
// caps the link well below the line rate. A 4 KiB scratch lets one read
// drain every frame the UART driver (or socket) has buffered.
static uint8_t rx_scratch[4096];
static uint8_t tx_scratch[m5hal::frame::kMaxWireSize];

static m5hal::remote::Server* server_ptr = nullptr;
static m5hal::service::ServiceRunner runner;

// ---------------------------------------------------------------------------
// I2S amplifier + bus (compiled only when ESP_PLATFORM && IDF gen5 I2S present)
// ---------------------------------------------------------------------------
#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD

static m5hal::i2s::Bus i2s_bus;
static m5hal::i2s::I2SBusConfig i2s_bus_cfg;
static m5hal::i2s::I2SAccessConfig i2s_acc_cfg;
static m5hal::i2s::I2STxAccessor* i2s_acc_ptr = nullptr;

// Enable the Core2 V1.1 speaker amplifier: AXP2101 ALDO3 -> 3300 mV.
// Source: I2SAudio example (M5Unified AXP2101 branch).
static void enableAmplifier(m5hal::i2c::I2CMasterAccessor& acc)
{
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

// AXP2101 probe + amplifier enable + I2S bus/accessor init.
// Sets i2s_acc_ptr only on success; the caller then calls registerI2S.
//
// Why probe first: GPIO12/0/2 are exposed on the M-BUS and GPIO12 is
// also a bootstrap-strapping pin on classic ESP32. Boards without an
// AXP2101 (e.g. Core BASIC) skip amp and I2S entirely, eliminating any
// risk of accidentally driving those pins (safety boundary).
static bool tryInitI2S(m5hal::i2c::I2CMasterAccessor& main_i2c_acc)
{
    // Probe AXP2101 (0x34) via the I2C accessor
    m5hal::i2c::I2CMasterAccessConfig axp_cfg;
    axp_cfg.i2c_addr = AXP2101_ADDR;
    axp_cfg.freq     = 400000;
    m5hal::i2c::I2CMasterAccessor axp_acc{main_i2c_acc.getI2CBus(), axp_cfg};
    if (!axp_acc.probe().has_value()) {
        // AXP2101 absent (e.g. Core BASIC) -> skip I2S
        return false;
    }
    enableAmplifier(axp_acc);

    i2s_bus_cfg.pin_bclk = PIN_I2S_BCK;
    i2s_bus_cfg.pin_ws   = PIN_I2S_WS;
    i2s_bus_cfg.pin_dout = PIN_I2S_DOUT;

#if defined(M5HAL_EXPERIMENT_REMOTE_TCP)
    // Generous DMA so every supported config rides out the link's rough
    // moments (WiFi shares airtime; expect coarser arrival clumps than the
    // USB bridge). Sizing rule: spec/design/remote.md §stream credit;
    // T-2 of decisions/024 calibrates the TCP numbers.
    i2s_bus_cfg.tx_buffer_size = 49152;
#else
    // UART path: 2x the measured floor of 6144 at 48 kHz stereo over the
    // USB bridge (decisions/024).
    i2s_bus_cfg.tx_buffer_size = 12288;
#endif

    if (!i2s_bus.init(i2s_bus_cfg).has_value()) {
        return false;
    }

    i2s_acc_cfg.sample_rate_hz   = 44100;
    i2s_acc_cfg.bits_per_sample  = 16;
    i2s_acc_cfg.channels         = 2;
    i2s_acc_cfg.write_timeout_ms = 0;  // the remote side drives flow via credit

    static m5hal::i2s::I2STxAccessor i2s_acc{i2s_bus, i2s_acc_cfg};
    i2s_acc_ptr = &i2s_acc;
    return true;
}

#endif  // ESP_PLATFORM && M5HAL_ESPIDF_I2S_HAS_STD

// ---------------------------------------------------------------------------
// Device-common initialisation (transport-independent)
// ---------------------------------------------------------------------------
static void commonDeviceInit()
{
    // Initialise the internal I2C bus
#if M5HAL_FRAMEWORK_HAS_ARDUINO
    m5hal::i2c::BusConfig i2c_cfg{&Wire, PIN_I2C_SCL, PIN_I2C_SDA};
#else
    m5hal::i2c::BusConfig i2c_cfg{PIN_I2C_SCL, PIN_I2C_SDA};
#endif
    (void)i2c_bus.init(i2c_cfg);

    static m5hal::i2c::I2CMasterAccessor i2c_acc{i2c_bus, i2c_acc_cfg};

    // Server + cooperative poll service. Registering the accessor as
    // bus_id 0 is what makes it reachable (and nothing else is).
    static m5hal::remote::Server srv{m5hal::data::DataSpan{server_scratch, sizeof(server_scratch)}};
    (void)srv.registerI2C(0, i2c_acc);

    // Publish the buttons (and nothing else) through the allowlist view.
    static AllowlistGPIO published_gpio{m5hal::gpio::getGPIO(), PUBLISHED_PINS,
                                        sizeof(PUBLISHED_PINS) / sizeof(PUBLISHED_PINS[0])};
    static m5hal::gpio::GPIOGroup published_group;
    if (published_group.addGPIO(&published_gpio, 0).has_value()) {
        srv.setGPIOGroup(published_group);
    }

#if defined(ESP_PLATFORM) && M5HAL_ESPIDF_I2S_HAS_STD
    // I2S: only configure and publish on bus_id 1 if AXP2101 probe succeeds (Core2 V1.1)
    if (tryInitI2S(i2c_acc) && i2s_acc_ptr != nullptr) {
        (void)srv.registerI2S(1, *i2s_acc_ptr);
    }
#endif

    server_ptr = &srv;
}

// ===========================================================================
// UART transport
// ===========================================================================
#if !defined(M5HAL_EXPERIMENT_REMOTE_TCP)

#if M5HAL_FRAMEWORK_HAS_ARDUINO

// ---- arduino UART0 ----
static m5hal::uart::Bus uart_bus;
static m5hal::uart::UARTAccessConfig uart_cfg;

static void uartTransportInit()
{
    // UART0 through M5HAL (no Serial.begin(); the variant begins lazily).
    // UART0 belongs exclusively to the protocol — do not write to Serial.
    m5hal::uart::BusConfig bus_cfg;
    bus_cfg.serial         = &Serial;
    bus_cfg.pin_tx         = PIN_UART_TX;
    bus_cfg.pin_rx         = PIN_UART_RX;
    bus_cfg.rx_buffer_size = 8192;
    bus_cfg.tx_buffer_size = 2048;
    (void)uart_bus.init(bus_cfg);

    uart_cfg.baud_rate             = M5HAL_EXPERIMENT_REMOTE_BAUD;
    uart_cfg.first_byte_timeout_ms = 2;  // cooperative poll when idle
    uart_cfg.inter_byte_timeout_ms = 1;
    uart_cfg.write_timeout_ms      = 100;

    static m5hal::uart::UARTTxAccessor uart_tx{uart_bus, uart_cfg};
    static m5hal::uart::UARTRxAccessor uart_rx{uart_bus, uart_cfg};

    static m5hal::data::StreamSource link_src{uart_rx, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    static m5hal::data::StreamSink link_snk{uart_tx, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
    static m5hal::remote::RemoteServerService sv{*server_ptr, link_src, link_snk};
    runner.add(sv);
}

void setup()
{
    commonDeviceInit();
    uartTransportInit();
}

void loop()
{
    // When idle each poll blocks for the UART first_byte timeout (2 ms),
    // which also lets lower-priority tasks run. Batch 50 polls per yield.
    for (int i = 0; i < 50; ++i) {
        runner.runOnce();
    }
    delay(1);
}

#else  // espidf UART

// ---- espidf UART0 ----
static m5hal::uart::variant::espidf::Bus uart_bus;
static m5hal::uart::UARTAccessConfig uart_cfg;

static void uartTransportInit()
{
    // UART0 doubles as the ESP-IDF log console; any runtime ESP_LOG output
    // would corrupt the protocol stream. Silence it.
    esp_log_level_set("*", ESP_LOG_NONE);

    m5hal::uart::variant::espidf::BusConfig bus_cfg;
    bus_cfg.port_num       = 0;  // UART0 = USB bridge
    bus_cfg.pin_tx         = PIN_UART_TX;
    bus_cfg.pin_rx         = PIN_UART_RX;
    bus_cfg.rx_buffer_size = 8192;
    bus_cfg.tx_buffer_size = 2048;
    (void)uart_bus.init(bus_cfg);

    uart_cfg.baud_rate             = M5HAL_EXPERIMENT_REMOTE_BAUD;
    uart_cfg.first_byte_timeout_ms = 2;
    uart_cfg.inter_byte_timeout_ms = 1;
    uart_cfg.write_timeout_ms      = 100;

    static m5hal::uart::UARTTxAccessor uart_tx{uart_bus, uart_cfg};
    static m5hal::uart::UARTRxAccessor uart_rx{uart_bus, uart_cfg};

    static m5hal::data::StreamSource link_src{uart_rx, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    static m5hal::data::StreamSink link_snk{uart_tx, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
    static m5hal::remote::RemoteServerService sv{*server_ptr, link_src, link_snk};
    runner.add(sv);
}

extern "C" void app_main(void)
{
    commonDeviceInit();
    uartTransportInit();

    while (true) {
        // Sustained 16-bit/44.1kHz stereo needs ~750 frames/s through the
        // server. A vTaskDelay(1) per poll (10 ms at the default 100 Hz tick)
        // would cap throughput far below that, so batch 50 polls per yield.
        // When idle each poll blocks for the UART first_byte timeout (2 ms).
        for (int i = 0; i < 50; ++i) {
            runner.runOnce();
        }
        vTaskDelay(1);
    }
}

#endif  // M5HAL_FRAMEWORK_HAS_ARDUINO (UART)

// ===========================================================================
// TCP transport
// ===========================================================================
#else  // M5HAL_EXPERIMENT_REMOTE_TCP defined

#if !M5HAL_FRAMEWORK_HAS_ARDUINO

// ---------------------------------------------------------------------------
// espidf WiFi bring-up
// ---------------------------------------------------------------------------
static constexpr char TAG[] = "RemoteMenu";
static std::atomic<bool> got_ip{false};
static esp_netif_ip_info_t ip_info    = {};
static constexpr uint16_t LISTEN_PORT = M5HAL_EXPERIMENT_TCP_PORT;

static void onWiFiEvent(void*, esp_event_base_t base, int32_t id, void* data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        (void)esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        got_ip.store(false);
        ESP_LOGW(TAG, "WiFi disconnected, retrying");
        (void)esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_info = static_cast<ip_event_got_ip_t*>(data)->ip_info;
        got_ip.store(true);
    }
}

static void wifiInit()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        (void)nvs_flash_erase();
        (void)nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    (void)esp_netif_create_default_wifi_sta();
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &onWiFiEvent, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &onWiFiEvent, nullptr));

    wifi_config_t wifi_cfg = {};
    ::snprintf(reinterpret_cast<char*>(wifi_cfg.sta.ssid), sizeof(wifi_cfg.sta.ssid), "%s", M5HAL_EXPERIMENT_WIFI_SSID);
    ::snprintf(reinterpret_cast<char*>(wifi_cfg.sta.password), sizeof(wifi_cfg.sta.password), "%s",
               M5HAL_EXPERIMENT_WIFI_PASS);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    // Modem power save (the default, MIN_MODEM) wakes the radio only per
    // DTIM beacon: measured 200-350 ms RTT spikes, which starve a
    // latency-paced stream (the credit window is sized for ~100 ms event
    // delay, spec/design/remote.md §stream credit). A streaming server
    // wants the radio awake.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

// listen -> accept one peer -> serve until it disconnects -> repeat.
// Never returns.
static void serveForever()
{
    dev_tcp::TcpListener listener;
    if (listener.listen(LISTEN_PORT) != m5hal::error::error_t::OK) {
        ESP_LOGE(TAG, "listen(%u) failed", static_cast<unsigned>(LISTEN_PORT));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    ESP_LOGI(TAG, "listening on " IPSTR ":%u", IP2STR(&ip_info.ip), static_cast<unsigned>(LISTEN_PORT));

    for (;;) {
        const int fd = listener.accept(1000);
        if (fd < 0) {
            continue;
        }
        dev_tcp::TcpStream stream;
        if (stream.attach(fd) != m5hal::error::error_t::OK) {
            continue;
        }
        // Tiny first-byte timeout: the server poll must not stall when the
        // link is idle (spec/design/remote.md, server execution model).
        stream.read_timeout_ms = 2;
        ESP_LOGI(TAG, "client connected");

        // Fresh per-connection plumbing: the frame reader's resync state and
        // the stream adapters' buffers must not leak across connections. The
        // Server itself persists (registrations are the allowlist; hello
        // resets its per-connection management state).
        m5hal::data::StreamSource link_src{stream, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
        m5hal::data::StreamSink link_snk{stream, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
        m5hal::remote::RemoteServerService sv{*server_ptr, link_src, link_snk};

        while (stream.isOpen()) {
            for (int i = 0; i < 50; ++i) {
                sv.service(m5hal::service::ServiceContext{});
            }
            vTaskDelay(1);
        }
        ESP_LOGI(TAG, "client disconnected");
    }
}

extern "C" void app_main(void)
{
    if (M5HAL_EXPERIMENT_WIFI_SSID[0] == '\0') {
        ESP_LOGE(TAG, "no WiFi credentials: rebuild with M5HAL_WIFI_SSID / M5HAL_WIFI_PASS set");
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    commonDeviceInit();
    wifiInit();
    while (!got_ip.load()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    serveForever();
}

#else  // arduino TCP

// ---------------------------------------------------------------------------
// arduino WiFi.begin() bring-up
// ---------------------------------------------------------------------------
static constexpr uint16_t LISTEN_PORT = M5HAL_EXPERIMENT_TCP_PORT;
static dev_tcp::TcpListener g_listener;

void setup()
{
    Serial.begin(115200);
    commonDeviceInit();

    if (M5HAL_EXPERIMENT_WIFI_SSID[0] == '\0') {
        Serial.println("no WiFi credentials: rebuild with M5HAL_WIFI_SSID / M5HAL_WIFI_PASS set");
        for (;;) {
            delay(1000);
        }
    }

    // The idiomatic Arduino bring-up — this block is the WHOLE framework
    // difference from the esp_wifi version in the espidf branch above.
    WiFi.mode(WIFI_STA);
    // Modem power save (the default) wakes the radio only per DTIM beacon:
    // measured 200-350 ms RTT spikes. A server wants the radio awake.
    WiFi.setSleep(false);
    WiFi.begin(M5HAL_EXPERIMENT_WIFI_SSID, M5HAL_EXPERIMENT_WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(100);
    }

    if (g_listener.listen(LISTEN_PORT) != m5hal::error::error_t::OK) {
        Serial.printf("listen(%u) failed\n", static_cast<unsigned>(LISTEN_PORT));
        for (;;) {
            delay(1000);
        }
    }
    Serial.printf("listening on %s:%u\n", WiFi.localIP().toString().c_str(), static_cast<unsigned>(LISTEN_PORT));
}

// listen -> accept one peer -> serve until it disconnects -> repeat.
void loop()
{
    const int fd = g_listener.accept(1000);
    if (fd < 0) {
        return;
    }
    dev_tcp::TcpStream stream;
    if (stream.attach(fd) != m5hal::error::error_t::OK) {
        return;
    }
    stream.read_timeout_ms = 2;
    Serial.println("client connected");

    // Fresh per-connection plumbing: the frame reader's resync state and the
    // stream adapters' buffers must not leak across connections (the Server
    // itself persists; hello resets its per-connection management state).
    m5hal::data::StreamSource link_src{stream, m5hal::data::DataSpan{rx_scratch, sizeof(rx_scratch)}};
    m5hal::data::StreamSink link_snk{stream, m5hal::data::DataSpan{tx_scratch, sizeof(tx_scratch)}};
    m5hal::remote::RemoteServerService sv{*server_ptr, link_src, link_snk};

    while (stream.isOpen()) {
        for (int i = 0; i < 50; ++i) {
            sv.service(m5hal::service::ServiceContext{});
        }
        delay(1);
    }
    Serial.println("client disconnected");
}

#endif  // !M5HAL_FRAMEWORK_HAS_ARDUINO (TCP)

#endif  // M5HAL_EXPERIMENT_REMOTE_TCP
