#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vl53l0x.h"

static const char *TAG = "basic_app";

#define PIN_SDA GPIO_NUM_8
#define PIN_SCL GPIO_NUM_9

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2C bus...");
    esp_err_t err = vl53l0x_i2c_master_init(PIN_SDA, PIN_SCL, 400000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }

    vl53l0x_dev_t dev = {
        .addr_7b = 0x29,
    };

    ESP_LOGI(TAG, "Init VL53L0X @0x29...");
    vTaskDelay(pdMS_TO_TICKS(50));
    err = vl53l0x_init(&dev, 33000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "vl53l0x_init failed: %s", esp_err_to_name(err));
        // boucle pour éviter reset en boucle
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (1) {
        uint16_t mm = 0;
        err = vl53l0x_read_mm(&dev, &mm);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Distance = %u mm", (unsigned)mm);
        } else {
            ESP_LOGW(TAG, "read failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
