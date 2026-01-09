#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "vl53l0x.h"

static const char *TAG = "multi_gpio_ready_mt";

// Ajuste ces GPIO à ton PCB
#define PIN_SDA GPIO_NUM_8
#define PIN_SCL GPIO_NUM_9

#define XSHUT_0 GPIO_NUM_3
#define XSHUT_1 GPIO_NUM_5

#define INT_0 GPIO_NUM_1
#define INT_1 GPIO_NUM_2

typedef struct {
    const char *label;
    bool ok;
    uint8_t range_status;
    uint16_t range_mm;
    esp_err_t err;
    VL53L0X_Error st_err;
} sensor_msg_t;

typedef struct {
    vl53l0x_dev_t *dev;
    const char *label;
    SemaphoreHandle_t i2c_mutex;
    QueueHandle_t out_queue;
    TickType_t timeout;
} sensor_task_ctx_t;

static void sensor_task(void *arg)
{
    sensor_task_ctx_t *ctx = (sensor_task_ctx_t *)arg;

    while (1) {
        VL53L0X_RangingMeasurementData_t data = {0};
        sensor_msg_t msg = {
            .label = ctx->label,
            .ok = false,
            .range_status = 0,
            .range_mm = 0,
            .err = ESP_OK,
            .st_err = VL53L0X_ERROR_NONE,
        };
        esp_err_t err = vl53l0x_wait_gpio_ready(ctx->dev, ctx->timeout);
        if (err == ESP_OK) {
            if (xSemaphoreTake(ctx->i2c_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                VL53L0X_Error st = VL53L0X_GetRangingMeasurementData(&ctx->dev->st, &data);
                if (st == VL53L0X_ERROR_NONE) {
                    msg.ok = true;
                    msg.range_status = data.RangeStatus;
                    msg.range_mm = data.RangeMilliMeter;
                } else {
                    msg.st_err = st;
                }
                VL53L0X_ClearInterruptMask(&ctx->dev->st, 0);
                xSemaphoreGive(ctx->i2c_mutex);
            } else {
                msg.err = ESP_ERR_TIMEOUT;
            }
        } else {
            msg.err = err;
        }

        if (ctx->out_queue != NULL) {
            (void)xQueueSend(ctx->out_queue, &msg, pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void display_task(void *arg)
{
    QueueHandle_t queue = (QueueHandle_t)arg;
    sensor_msg_t msg = {0};

    while (1) {
        if (xQueueReceive(queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.err != ESP_OK) {
                ESP_LOGW(TAG, "%s error: %s", msg.label, esp_err_to_name(msg.err));
                continue;
            }

            if (!msg.ok) {
                ESP_LOGW(TAG, "GetRangingMeasurementData %s failed: %d",
                         msg.label,
                         (int)msg.st_err);
                continue;
            }

            if (msg.range_status == 0) {
                ESP_LOGI(TAG, "%s = %u mm", msg.label, (unsigned)msg.range_mm);
            } else {
                ESP_LOGW(TAG, "%s invalid range: status=%u mm=%u",
                         msg.label,
                         (unsigned)msg.range_status,
                         (unsigned)msg.range_mm);
            }
        }
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

    SemaphoreHandle_t i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C mutex");
        return;
    }

    QueueHandle_t msg_queue = xQueueCreate(8, sizeof(sensor_msg_t));
    if (msg_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor queue");
        return;
    }

    static sensor_task_ctx_t ctx0;
    static sensor_task_ctx_t ctx1;

    ctx0.dev = &dev0;
    ctx0.label = "VL53[0]";
    ctx0.i2c_mutex = i2c_mutex;
    ctx0.out_queue = msg_queue;
    ctx0.timeout = pdMS_TO_TICKS(1000);

    ctx1.dev = &dev1;
    ctx1.label = "VL53[1]";
    ctx1.i2c_mutex = i2c_mutex;
    ctx1.out_queue = msg_queue;
    ctx1.timeout = pdMS_TO_TICKS(1000);

    xTaskCreate(display_task, "vl53_display", 4096, msg_queue, 5, NULL);
    xTaskCreate(sensor_task, "vl53_task_0", 4096, &ctx0, 5, NULL);
    xTaskCreate(sensor_task, "vl53_task_1", 4096, &ctx1, 5, NULL);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
