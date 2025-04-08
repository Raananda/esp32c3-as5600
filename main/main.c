#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "as5600.h"

#define TAG "SMART_MAG"
#define RAW_MAX 4096
#define SAMPLE_DELAY_MS 20
#define BULLET_MAX 10
#define BULLET_DEBOUNCE_RAW -10

float virtual_raw = 0;

// Hardcoded bullet calibration thresholds
const int bullet_thresholds[BULLET_MAX + 1] = {
    0, 210, 465, 723, 973, 1217, 1464, 1717, 1951, 2182, 2396
};

void sensor_task(void *pvParameters)
{
    as5600_init();
    uint16_t last_raw = as5600_read_raw_angle();
    int last_bullet = -1;

    while (1) {
        uint16_t raw = as5600_read_raw_angle();
        int delta = raw - last_raw;

        if (delta > RAW_MAX / 2) delta -= RAW_MAX;
        if (delta < -RAW_MAX / 2) delta += RAW_MAX;

        virtual_raw += delta;
        last_raw = raw;

        if (virtual_raw < 0) virtual_raw = 0;

        // Find bullet index based on thresholds
        int bullet = 0;
        for (int i = 1; i <= BULLET_MAX; i++) {
            if (virtual_raw >= bullet_thresholds[i] + BULLET_DEBOUNCE_RAW) {
                bullet = i;
            } else {
                break;
            }
        }

        if (bullet != last_bullet) {
            last_bullet = bullet;
            ESP_LOGW(TAG, "BULLET COUNT: %d", bullet);
        }

        ESP_LOGI(TAG, "Raw: %u, Δ: %+d, Virtual Raw: %.1f, Bullets: %d",
                 raw, delta, virtual_raw, bullet);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_DELAY_MS));
    }
}

void app_main(void)
{
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
