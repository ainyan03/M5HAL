// SPDX-License-Identifier: MIT
//
// Public HIL device for the queue-driven M5HAL ESP-IDF SPI slave backend.
// A CoreS3 preloads a continuous byte ramp, opens one long-lived slave Access,
// and retains every CS-delimited RX frame for validation after the batch.

#include <M5HAL_v2.hpp>

#include <esp_log.h>
#include <driver/gpio.h>

#include <algorithm>
#include <stddef.h>
#include <stdint.h>

namespace m5hal = m5::hal::v2;

namespace {

constexpr int kPinClk  = 18;
constexpr int kPinMosi = 17;
constexpr int kPinMiso = 8;
constexpr int kPinCs   = 9;

constexpr uint8_t kMode          = 1;
constexpr size_t kLength         = 32;
constexpr size_t kTransactions   = 101;
constexpr size_t kObservedFrames = kTransactions + 1;
constexpr uint32_t kTimeoutMs    = 10000;

const char* const kTag = "spi_slave_hil";

using QueueStorage = m5hal::slave::StaticSlaveQueueStorage<kLength * kTransactions, kLength * kTransactions,
                                                           kTransactions, kObservedFrames>;

bool runBatch(m5hal::spi::SpiSlaveAccessor& accessor)
{
    auto cleared_tx = accessor.clearTx();
    auto cleared_rx = accessor.clearRx();
    if (!cleared_tx.has_value() || !cleared_rx.has_value()) {
        return false;
    }

    uint8_t chunk[kLength];
    uint8_t tx_counter = 0;
    for (size_t transaction = 0; transaction < kTransactions; ++transaction) {
        for (size_t i = 0; i < kLength; ++i) {
            chunk[i] = tx_counter++;
        }
        auto queued = accessor.write({chunk, sizeof(chunk)});
        if (!queued.has_value() || *queued != sizeof(chunk)) {
            return false;
        }
    }

    auto begun = accessor.beginAccess(kTimeoutMs);
    if (!begun.has_value()) {
        ESP_LOGE(kTag, "beginAccess failed error=%d", static_cast<int>(begun.error()));
        return false;
    }
    ESP_LOGI(kTag, "ARMED SPI_SLAVE_M5HAL");

    const uint32_t started = m5hal::runtime::millis();
    while (accessor.rxFrames().readableFrames() < kObservedFrames &&
           m5hal::runtime::millis() - started < kTimeoutMs * 2) {
        m5hal::runtime::delayMs(1);
    }
    auto ended = accessor.endAccess(kTimeoutMs);
    if (!ended.has_value()) {
        ESP_LOGE(kTag, "endAccess failed error=%d", static_cast<int>(ended.error()));
        return false;
    }

    int previous_last = -1;
    size_t intra_bad  = 0;
    size_t inter_bad  = 0;
    size_t received   = 0;
    uint8_t rx[kLength];
    while (received < kTransactions) {
        auto frame = accessor.rxFrames().peekFrame();
        if (frame.has_value() && received == 0 && frame->metadata.wire_bytes == 0 &&
            frame->metadata.stored_bytes == 0) {
            // ESP-IDF can report the first CS synchronization edge as an empty
            // boundary. It is real boundary evidence, but not a data frame.
            (void)accessor.rxFrames().popFrame();
            continue;
        }
        if (!frame.has_value() || frame->metadata.wire_bytes != kLength || frame->metadata.stored_bytes != kLength) {
            ESP_LOGE(kTag, "short/missing frame transaction=%u readable=%u cs=%d error=%d wire=%u stored=%u",
                     static_cast<unsigned>(received), static_cast<unsigned>(accessor.rxFrames().readableFrames()),
                     ::gpio_get_level(static_cast<::gpio_num_t>(kPinCs)),
                     frame.has_value() ? 0 : static_cast<int>(frame.error()),
                     frame.has_value() ? static_cast<unsigned>(frame->metadata.wire_bytes) : 0u,
                     frame.has_value() ? static_cast<unsigned>(frame->metadata.stored_bytes) : 0u);
            return false;
        }
        std::copy(frame->first.data, frame->first.data + frame->first.size, rx);
        std::copy(frame->second.data, frame->second.data + frame->second.size, rx + frame->first.size);
        if (!accessor.rxFrames().popFrame().has_value()) {
            return false;
        }
        ++received;

        // Exclude the first data frame from continuity checks because no prior
        // accepted frame exists yet.
        if (received == 1) {
            continue;
        }
        for (size_t i = 1; i < kLength; ++i) {
            if (static_cast<uint8_t>(rx[i] - rx[i - 1]) != 1u) {
                ++intra_bad;
                break;
            }
        }
        if (previous_last >= 0 && rx[0] != static_cast<uint8_t>(previous_last + 1)) {
            ++inter_bad;
        }
        previous_last = rx[kLength - 1];
    }

    ESP_LOGI(kTag, "RESULT SPI_SLAVE_M5HAL %s mode=%u len=%u transactions=%u received=%u intra_bad=%u inter_bad=%u",
             received == kTransactions && intra_bad == 0 && inter_bad == 0 ? "PASS" : "FAIL",
             static_cast<unsigned>(kMode), static_cast<unsigned>(kLength), static_cast<unsigned>(kTransactions),
             static_cast<unsigned>(received), static_cast<unsigned>(intra_bad), static_cast<unsigned>(inter_bad));
    return received == kTransactions && intra_bad == 0 && inter_bad == 0;
}

}  // namespace

extern "C" void app_main()
{
    auto claimed = m5hal::M5_Hal.SPI.claimController();
    if (!claimed.has_value()) {
        ESP_LOGE(kTag, "controller claim failed error=%d", static_cast<int>(claimed.error()));
        return;
    }

    m5hal::spi::SlaveBusConfig config;
    config.pin_clk    = kPinClk;
    config.pin_mosi   = kPinMosi;
    config.pin_miso   = kPinMiso;
    config.pin_cs     = kPinCs;
    config.spi_mode   = kMode;
    config.controller = *claimed;

    static m5hal::spi::SpiSlaveBus_espidf bus;
    auto initialized = bus.init(config);
    if (!initialized.has_value()) {
        ESP_LOGE(kTag, "init failed error=%d", static_cast<int>(initialized.error()));
        (void)m5hal::M5_Hal.SPI.releaseClaimedController(*claimed);
        return;
    }

    static QueueStorage storage;
    m5hal::spi::SlaveAccessConfig access_config;
    access_config.transaction_bytes = kLength;
    access_config.tx_mode           = m5hal::slave::QueueMode::Byte;
    access_config.rx_mode           = m5hal::slave::QueueMode::Frame;
    m5hal::spi::SpiSlaveAccessor accessor{bus, storage.tx(), storage.rx(), access_config};
    ESP_LOGI(kTag, "READY SPI_SLAVE_M5HAL clk=%d mosi=%d miso=%d cs=%d mode=%u len=%u transactions=%u", kPinClk,
             kPinMosi, kPinMiso, kPinCs, static_cast<unsigned>(kMode), static_cast<unsigned>(kLength),
             static_cast<unsigned>(kTransactions));

    m5hal::runtime::delayMs(5000);
    for (;;) {
        if (!runBatch(accessor)) {
            m5hal::runtime::delayMs(1000);
        }
    }
}
