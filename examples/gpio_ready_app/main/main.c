#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vl53l0x.h"

static const char *TAG = "gpio_ready_app";

#define PIN_SDA GPIO_NUM_8
#define PIN_SCL GPIO_NUM_9
#define PIN_INT GPIO_NUM_7

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
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    err = vl53l0x_enable_gpio_ready(&dev, PIN_INT, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ready init failed: %s", esp_err_to_name(err));
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    VL53L0X_Error st = VL53L0X_StartMeasurement(&dev.st);
    if (st != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "StartMeasurement failed: %d", (int)st);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (1) {
        VL53L0X_RangingMeasurementData_t data = {0};

        err = vl53l0x_wait_gpio_ready(&dev, pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "GPIO wait timeout: %s", esp_err_to_name(err));
            continue;
        }

        st = VL53L0X_GetRangingMeasurementData(&dev.st, &data);
        if (st == VL53L0X_ERROR_NONE && data.RangeStatus == 0) {
            ESP_LOGI(TAG, "Distance = %u mm", (unsigned)data.RangeMilliMeter);
        } else if (st == VL53L0X_ERROR_NONE) {
            ESP_LOGW(TAG, "Invalid range: status=%u mm=%u",
                     (unsigned)data.RangeStatus,
                     (unsigned)data.RangeMilliMeter);
        } else {
            ESP_LOGW(TAG, "GetRangingMeasurementData failed: %d", (int)st);
        }

        VL53L0X_ClearInterruptMask(&dev.st, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
