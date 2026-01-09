#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "vl53l0x.h"

static const char *TAG = "basic_app";

// Adjust these GPIOs to match your PCB layout.
#define PIN_SDA    GPIO_NUM_8
#define PIN_SCL    GPIO_NUM_9

#define XSHUT_0    GPIO_NUM_3
#define XSHUT_1    GPIO_NUM_5

void app_main(void)
{
    ESP_LOGI(TAG, "Init I2C bus (new driver)...");
    ESP_ERROR_CHECK(vl53l0x_i2c_master_init(PIN_SDA, PIN_SCL, 400000));

    // Multi-sensor: assign unique addresses via XSHUT.
    vl53l0x_slot_t slots[2] = {
        { .xshut_gpio = XSHUT_0, .new_addr_7b = 0x2A },
        { .xshut_gpio = XSHUT_1, .new_addr_7b = 0x2B },
    };

    ESP_LOGI(TAG, "Assign addresses (multi XSHUT)...");
    ESP_ERROR_CHECK(vl53l0x_multi_assign_addresses(slots, 2, 10));

    // Initialize both devices at their assigned addresses.
    vl53l0x_dev_t dev0 = { .addr_7b = 0x2A };
    vl53l0x_dev_t dev1 = { .addr_7b = 0x2B };

    ESP_LOGI(TAG, "Init dev0...");
    ESP_ERROR_CHECK(vl53l0x_init(&dev0, 33000));

    ESP_LOGI(TAG, "Init dev1...");
    ESP_ERROR_CHECK(vl53l0x_init(&dev1, 33000));

    while (1) {
        uint16_t mm0 = 0, mm1 = 0;

        esp_err_t e0 = vl53l0x_read_mm(&dev0, &mm0);
        esp_err_t e1 = vl53l0x_read_mm(&dev1, &mm1);

        if (e0 == ESP_OK) ESP_LOGI(TAG, "VL53[0] = %u mm", (unsigned)mm0);
        else              ESP_LOGW(TAG, "VL53[0] read error (%d)", (int)e0);

        if (e1 == ESP_OK) ESP_LOGI(TAG, "VL53[1] = %u mm", (unsigned)mm1);
        else              ESP_LOGW(TAG, "VL53[1] read error (%d)", (int)e1);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
