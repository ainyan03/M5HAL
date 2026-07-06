// GPIO channel identification companion for CoreS3 (ESP32-S3).
// Sets all GROVE port pins to input (Hi-Z) so Core2 can drive them
// cleanly for logic analyzer channel mapping.

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const gpio_num_t pins[] = {
    GPIO_NUM_2,  GPIO_NUM_1,   // Port A
    GPIO_NUM_8,  GPIO_NUM_9,   // Port B
    GPIO_NUM_18, GPIO_NUM_17,  // Port C
};
static constexpr int kNumPins = sizeof(pins) / sizeof(pins[0]);

static const char* TAG = "gpio_ident_s3";

extern "C" void app_main(void)
{
    for (int i = 0; i < kNumPins; ++i) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_INPUT);
        gpio_pullup_dis(pins[i]);
        gpio_pulldown_dis(pins[i]);
    }

    ESP_LOGI(TAG, "all GROVE pins set to input (Hi-Z)");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
