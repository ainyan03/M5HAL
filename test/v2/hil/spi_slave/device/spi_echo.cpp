// SPDX-License-Identifier: MIT
//
// HIL device firmware -- SPI slave echo via ESP-IDF spi_slave driver.
//
// Runs on a CoreS3 (ESP32-S3) as an SPI slave using Port B + Port C pins:
//   CLK  = GPIO 8  (Port B Pin1)
//   MISO = GPIO 9  (Port B Pin2, slave output)
//   MOSI = GPIO 18 (Port C Pin1)
//   CS   = GPIO 17 (Port C Pin2)
//
// Echo contract (two SPI transactions per round):
//   1. Master sends N data bytes (CS stays asserted for the entire write).
//      The slave captures the bytes into rx_buf via DMA.
//   2. Master then does a separate read (new CS assertion). The slave sends
//      the previously captured bytes from tx_buf.
//
// After each receive, the firmware copies rx_buf to tx_buf so the next
// transaction echoes the data. It also logs the byte count on the serial
// console for monitoring.
//
// Wiring (2-board HIL, straight GROVE cables):
//   Core2 PortB-Pin1(26) <-> CoreS3 PortB-Pin1(8)   CLK
//   Core2 PortB-Pin2(36) <-> CoreS3 PortB-Pin2(9)   MISO
//   Core2 PortC-Pin1(17) <-> CoreS3 PortC-Pin1(18)  MOSI
//   Core2 PortC-Pin2(16) <-> CoreS3 PortC-Pin2(17)  CS
//
// Build / flash:
//   export M5HAL_PIO_EXTRA_CONFIG=pio_envs/v2/hil.ini.cli
//   pio run -e v2_hil_spi_slave_echo_esp32s3 -t upload

#include <driver/spi_slave.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <string.h>

#define PIN_CLK  2  // CoreS3 PortA Pin1
#define PIN_MOSI 1  // CoreS3 PortA Pin2
#define PIN_MISO 8  // CoreS3 PortB → Core2 G36 (input-only = master MISO read)
#define PIN_CS   9  // CoreS3 PortB → Core2 G26 (output = master CS drive)

#define BUF_SIZE 4096

static const char* TAG = "spi_echo";

static uint8_t* rx_buf = nullptr;
static uint8_t* tx_buf = nullptr;

extern "C" void app_main(void)
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num      = PIN_MOSI;
    buscfg.miso_io_num      = PIN_MISO;
    buscfg.sclk_io_num      = PIN_CLK;
    buscfg.quadwp_io_num    = -1;
    buscfg.quadhd_io_num    = -1;
    buscfg.max_transfer_sz  = BUF_SIZE;

    spi_slave_interface_config_t slvcfg = {};
    slvcfg.mode                         = 0;
    slvcfg.spics_io_num                 = PIN_CS;
    slvcfg.queue_size                   = 1;
    slvcfg.flags                        = 0;

    esp_err_t ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_slave_initialize failed: %s", esp_err_to_name(ret));
        return;
    }
    rx_buf = static_cast<uint8_t*>(heap_caps_malloc(BUF_SIZE, MALLOC_CAP_DMA));
    tx_buf = static_cast<uint8_t*>(heap_caps_malloc(BUF_SIZE, MALLOC_CAP_DMA));
    if (rx_buf == nullptr || tx_buf == nullptr) {
        ESP_LOGE(TAG, "DMA buffer alloc failed");
        return;
    }
    ESP_LOGI(TAG, "SPI slave ready (CLK=%d MOSI=%d MISO=%d CS=%d)", PIN_CLK, PIN_MOSI, PIN_MISO, PIN_CS);

    memset(tx_buf, 0xA5, BUF_SIZE);
    memset(rx_buf, 0, BUF_SIZE);

    for (;;) {
        spi_slave_transaction_t t = {};
        t.length                  = BUF_SIZE * 8;
        t.rx_buffer               = rx_buf;
        t.tx_buffer               = tx_buf;

        ret = spi_slave_transmit(SPI2_HOST, &t, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit err: %s", esp_err_to_name(ret));
            continue;
        }

        size_t rx_bytes = t.trans_len / 8;
        ESP_LOGI(TAG, "rx %zu bytes  tx[0..3]=%02x %02x %02x %02x  rx[0..3]=%02x %02x %02x %02x", rx_bytes, tx_buf[0],
                 tx_buf[1], tx_buf[2], tx_buf[3], rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

        if (rx_bytes > 0 && rx_bytes <= BUF_SIZE) {
            memcpy(tx_buf, rx_buf, rx_bytes);
        }
    }
}
