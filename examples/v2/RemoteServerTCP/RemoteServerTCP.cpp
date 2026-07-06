// SPDX-License-Identifier: MIT
// =============================================================================
// M5HAL - RemoteServerTCP
//
// Device-side firmware for the M5HAL remote bus protocol over TCP. Bring up
// WiFi, listen with BsdTcpRemoteServer, then pump server.service() from the
// main loop. Wired Ethernet works with the same server code once its netif is
// up, because the transport uses the BSD socket API above lwIP.
//
// Override M5HAL_EXAMPLE_WIFI_SSID, M5HAL_EXAMPLE_WIFI_PASS, and
// M5HAL_EXAMPLE_TCP_PORT from build flags for your network.
//
// PIO envs:
//   RemoteServerTCP_esp32           WiFi + TCP / espidf
//   RemoteServerTCP_esp32_arduino   WiFi + TCP / arduino
// =============================================================================

#include <M5HAL_v2.hpp>

#if M5HAL_FRAMEWORK_HAS_ARDUINO
#include <Arduino.h>
#include <WiFi.h>
#else
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>

#include <cstdio>
#include <cstring>
#endif

// ---------------------------------------------------------------------------
// Defaults
// ---------------------------------------------------------------------------
#ifndef M5HAL_EXAMPLE_WIFI_SSID
#define M5HAL_EXAMPLE_WIFI_SSID "your-ssid"
#endif
#ifndef M5HAL_EXAMPLE_WIFI_PASS
#define M5HAL_EXAMPLE_WIFI_PASS "your-password"
#endif
#ifndef M5HAL_EXAMPLE_TCP_PORT
#define M5HAL_EXAMPLE_TCP_PORT 3333
#endif

// ---------------------------------------------------------------------------
// Namespace aliases
// ---------------------------------------------------------------------------
namespace m5hal = m5::hal::v2;

// ---------------------------------------------------------------------------
// Shared resources
// ---------------------------------------------------------------------------
static constexpr uint16_t TCP_PORT = M5HAL_EXAMPLE_TCP_PORT;

static m5hal::remote::BsdTcpRemoteServer g_server;

// Per-connection allowlist: expose the facade GPIO group so remote gpio
// read/write work (registration doubles as the safety allowlist; without
// it every GPIO request is rejected with INVALID_STATE).
static m5hal::result_t<void> onConnectionSetup(void*, m5hal::remote::Server& srv)
{
    srv.setGPIOGroup(m5hal::M5_Hal.Gpio);
    return {};
}

static void setupServerPolicy()
{
    (void)m5hal::M5_Hal.init();
#if defined(CONFIG_IDF_TARGET_ESP32)
    (void)m5hal::M5_Hal.Gpio.setDenyMask(0, 0, 0x00000FC0u);  // GPIO 6-11 = Flash SPI
#endif
    g_server.setConnectionSetupHandler(&onConnectionSetup, nullptr);
}

// BsdTcpRemoteServer creates a Server and ServerBusPool per connection. The
// dynamic BusCreate handler is installed by the hub for each accepted peer, so
// applications only need to keep the service loop running.

#if M5HAL_FRAMEWORK_HAS_ARDUINO

// ---- arduino ----
static bool connectWiFi()
{
    Serial.begin(115200);
    delay(100);

    WiFi.mode(WIFI_STA);
    // Modem power save adds 100-400 ms request latency; this server wants
    // to answer promptly, so keep the radio awake.
    WiFi.setSleep(false);
    WiFi.begin(M5HAL_EXAMPLE_WIFI_SSID, M5HAL_EXAMPLE_WIFI_PASS);

    const uint32_t start_ms   = millis();
    const uint32_t timeout_ms = 20000;
    while (WiFi.status() != WL_CONNECTED && (millis() - start_ms) < timeout_ms) {
        delay(250);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("WiFi connection failed after %lu ms; stopping\n", static_cast<unsigned long>(timeout_ms));
        return false;
    }

    Serial.printf("WiFi connected: %s\n", WiFi.localIP().toString().c_str());
    return true;
}

static void stopForever()
{
    while (true) {
        delay(1000);
    }
}

void setup()
{
    if (!connectWiFi()) {
        stopForever();
    }

    setupServerPolicy();
    auto r = g_server.begin(TCP_PORT);
    if (!r.has_value()) {
        Serial.printf("TCP listen failed: %s (%d)\n", m5hal::error::toString(r.error()), static_cast<int>(r.error()));
        stopForever();
    }

    Serial.printf("Remote TCP server listening at %s:%u\n", WiFi.localIP().toString().c_str(),
                  static_cast<unsigned>(g_server.boundPort()));
}

void loop()
{
    (void)g_server.service();
    delay(1);
}

#else  // espidf

// ---- espidf ----
static constexpr const char* TAG = "RemoteServerTCP";

static bool checkEsp(esp_err_t err, const char* what)
{
    if (err == ESP_OK) {
        return true;
    }
    ESP_LOGE(TAG, "%s failed: %s", what, esp_err_to_name(err));
    return false;
}

static bool initNvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if (!checkEsp(nvs_flash_erase(), "nvs_flash_erase")) {
            return false;
        }
        err = nvs_flash_init();
    }
    return checkEsp(err, "nvs_flash_init");
}

static bool connectWiFi(esp_netif_t** out_netif)
{
    if (!initNvs()) {
        return false;
    }
    if (!checkEsp(esp_netif_init(), "esp_netif_init")) {
        return false;
    }
    if (!checkEsp(esp_event_loop_create_default(), "esp_event_loop_create_default")) {
        return false;
    }

    esp_netif_t* netif = esp_netif_create_default_wifi_sta();
    if (netif == nullptr) {
        ESP_LOGE(TAG, "esp_netif_create_default_wifi_sta failed");
        return false;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (!checkEsp(esp_wifi_init(&init_cfg), "esp_wifi_init")) {
        return false;
    }

    wifi_config_t wifi_cfg = {};
    std::snprintf(reinterpret_cast<char*>(wifi_cfg.sta.ssid), sizeof(wifi_cfg.sta.ssid), "%s", M5HAL_EXAMPLE_WIFI_SSID);
    std::snprintf(reinterpret_cast<char*>(wifi_cfg.sta.password), sizeof(wifi_cfg.sta.password), "%s",
                  M5HAL_EXAMPLE_WIFI_PASS);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    if (!checkEsp(esp_wifi_set_mode(WIFI_MODE_STA), "esp_wifi_set_mode")) {
        return false;
    }
    if (!checkEsp(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg), "esp_wifi_set_config")) {
        return false;
    }
    if (!checkEsp(esp_wifi_start(), "esp_wifi_start")) {
        return false;
    }
    // Modem power save adds 100-400 ms request latency; keep the radio awake.
    (void)esp_wifi_set_ps(WIFI_PS_NONE);
    if (!checkEsp(esp_wifi_connect(), "esp_wifi_connect")) {
        return false;
    }

    const TickType_t start_tick   = xTaskGetTickCount();
    const TickType_t timeout_tick = pdMS_TO_TICKS(20000);
    esp_netif_ip_info_t ip_info   = {};
    while ((xTaskGetTickCount() - start_tick) < timeout_tick) {
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            *out_netif = netif;
            ESP_LOGI(TAG, "WiFi connected: " IPSTR, IP2STR(&ip_info.ip));
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }

    ESP_LOGE(TAG, "WiFi connection failed after 20000 ms; stopping");
    return false;
}

extern "C" void app_main(void)
{
    esp_netif_t* netif = nullptr;
    if (!connectWiFi(&netif)) {
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    setupServerPolicy();
    auto r = g_server.begin(TCP_PORT);
    if (!r.has_value()) {
        ESP_LOGE(TAG, "TCP listen failed: %s (%d)", m5hal::error::toString(r.error()), static_cast<int>(r.error()));
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    esp_netif_ip_info_t ip_info = {};
    (void)esp_netif_get_ip_info(netif, &ip_info);
    ESP_LOGI(TAG, "Remote TCP server listening at " IPSTR ":%u", IP2STR(&ip_info.ip),
             static_cast<unsigned>(g_server.boundPort()));

    // Serve from a dedicated task: service() executes bus transfers (I2S DMA
    // setup, mono expansion bounce buffers, ...) whose stack depth exceeds the
    // default main-task stack. 8 KiB matches the Arduino loopTask headroom.
    auto serve = [](void*) {
        while (true) {
            (void)g_server.service();
            vTaskDelay(1);
        }
    };
    if (::xTaskCreate(serve, "m5hal_srv", 8192, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "server task create failed");
    }
}

#endif  // M5HAL_FRAMEWORK_HAS_ARDUINO
