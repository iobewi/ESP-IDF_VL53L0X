#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "vl53l0x.h"

static const char *TAG = "multi_gpio_ready";

// Ajuste ces GPIO à ton PCB
#define PIN_SDA GPIO_NUM_8
#define PIN_SCL GPIO_NUM_9

#define XSHUT_0 GPIO_NUM_3
#define XSHUT_1 GPIO_NUM_5

#define INT_0 GPIO_NUM_1
#define INT_1 GPIO_NUM_2

static void log_measurement(const char *label,
                            VL53L0X_RangingMeasurementData_t *data)
{
    if (data->RangeStatus == 0) {
        ESP_LOGI(TAG, "%s = %u mm", label, (unsigned)data->RangeMilliMeter);
    } else {
        ESP_LOGW(TAG, "%s invalid range: status=%u mm=%u",
                 label,
                 (unsigned)data->RangeStatus,
                 (unsigned)data->RangeMilliMeter);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2C bus...");
    esp_err_t err = vl53l0x_i2c_master_init(PIN_SDA, PIN_SCL, 400000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }

    vl53l0x_slot_t slots[2] = {
        { .xshut_gpio = XSHUT_0, .new_addr_7b = 0x2A },
        { .xshut_gpio = XSHUT_1, .new_addr_7b = 0x2B },
    };

    ESP_LOGI(TAG, "Assign addresses (multi XSHUT)...");
    ESP_ERROR_CHECK(vl53l0x_multi_assign_addresses(slots, 2, 10));

    vl53l0x_dev_t dev0 = { .addr_7b = 0x2A };
    vl53l0x_dev_t dev1 = { .addr_7b = 0x2B };

    ESP_LOGI(TAG, "Init dev0...");
    ESP_ERROR_CHECK(vl53l0x_init(&dev0, 33000));

    ESP_LOGI(TAG, "Init dev1...");
    ESP_ERROR_CHECK(vl53l0x_init(&dev1, 33000));

    err = vl53l0x_enable_gpio_ready(&dev0, INT_0, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ready init dev0 failed: %s", esp_err_to_name(err));
        return;
    }

    err = vl53l0x_enable_gpio_ready(&dev1, INT_1, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO ready init dev1 failed: %s", esp_err_to_name(err));
        return;
    }

    VL53L0X_Error st = VL53L0X_StartMeasurement(&dev0.st);
    if (st != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "StartMeasurement dev0 failed: %d", (int)st);
        return;
    }

    st = VL53L0X_StartMeasurement(&dev1.st);
    if (st != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "StartMeasurement dev1 failed: %d", (int)st);
        return;
    }

    while (1) {
        VL53L0X_RangingMeasurementData_t data0 = {0};
        VL53L0X_RangingMeasurementData_t data1 = {0};

        err = vl53l0x_wait_gpio_ready(&dev0, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            st = VL53L0X_GetRangingMeasurementData(&dev0.st, &data0);
            if (st == VL53L0X_ERROR_NONE) {
                log_measurement("VL53[0]", &data0);
            } else {
                ESP_LOGW(TAG, "GetRangingMeasurementData dev0 failed: %d", (int)st);
            }
            VL53L0X_ClearInterruptMask(&dev0.st, 0);
        } else {
            ESP_LOGW(TAG, "GPIO wait timeout dev0: %s", esp_err_to_name(err));
        }

        err = vl53l0x_wait_gpio_ready(&dev1, pdMS_TO_TICKS(1000));
        if (err == ESP_OK) {
            st = VL53L0X_GetRangingMeasurementData(&dev1.st, &data1);
            if (st == VL53L0X_ERROR_NONE) {
                log_measurement("VL53[1]", &data1);
            } else {
                ESP_LOGW(TAG, "GetRangingMeasurementData dev1 failed: %d", (int)st);
            }
            VL53L0X_ClearInterruptMask(&dev1.st, 0);
        } else {
            ESP_LOGW(TAG, "GPIO wait timeout dev1: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
