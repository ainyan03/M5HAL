// GPIO channel identification firmware for Core2 (ESP32).
// Pulses each GROVE port pin with a unique count pattern so a logic
// analyzer can map Saleae channels to physical pins.
//
// Pattern: pin gets (index+1) pulses, 50ms HIGH / 50ms LOW per pulse,
// 500ms gap between pins, 2s gap between full cycles.
//
// Pin order (index):
//   0: G32  Port A Pin1   (1 pulse)
//   1: G33  Port A Pin2   (2 pulses)
//   2: G26  Port B Pin2   (3 pulses)  -- G36 is input-only, skipped
//   3: G13  Port C Pin1   (4 pulses)
//   4: G14  Port C Pin2   (5 pulses)
//   5: G1   UART TX       (6 pulses)
//   6: G3   UART RX       (7 pulses)

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const gpio_num_t pins[] = {
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_26, GPIO_NUM_13, GPIO_NUM_14, GPIO_NUM_1, GPIO_NUM_3,
};
static constexpr int kNumPins = sizeof(pins) / sizeof(pins[0]);

static const char* TAG = "gpio_ident";

extern "C" void app_main(void)
{
    for (int i = 0; i < kNumPins; ++i) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }

    ESP_LOGI(TAG, "GPIO ident started — %d pins, cycling", kNumPins);

    for (;;) {
        for (int p = 0; p < kNumPins; ++p) {
            int count = p + 1;
            ESP_LOGI(TAG, "pin %d: %d pulses", pins[p], count);
            for (int n = 0; n < count; ++n) {
                gpio_set_level(pins[p], 1);
                vTaskDelay(pdMS_TO_TICKS(50));
                gpio_set_level(pins[p], 0);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
