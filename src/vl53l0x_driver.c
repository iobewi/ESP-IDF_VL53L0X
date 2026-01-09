#include "vl53l0x.h"

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"

/*
 * Architecture note:
 * - This file provides the ESP-IDF driver layer (bus init, probe, sensor init, read).
 * - The ST API calls the "platform" layer (st_api/platform), which exposes
 *   I2C byte/word/dword wrappers via vl53l0x_i2c_{read,write}_reg().
 */

static const char *TAG = "vl53l0x";
static bool isr_service_installed = false;

/* These symbols are implemented in st_api/platform/src/vl53l0x_i2c_platform.c */
extern esp_err_t vl53l0x_i2c_master_init(gpio_num_t sda,
                                        gpio_num_t scl,
                                        uint32_t clk_hz);

extern esp_err_t vl53l0x_i2c_probe(uint8_t addr_7b);

/* ---- Implementations adapted from the legacy vl53l0x_i2c_platform.c ---- */
static esp_err_t st_init_sequence(VL53L0X_Dev_t *pDevice, uint32_t timing_budget_us)
{
    VL53L0X_Error st;
    uint8_t isApertureSpads = 0;
    uint32_t refSpadCount = 0;
    uint8_t VhvSettings = 0, PhaseCal = 0;

    st = VL53L0X_DataInit(pDevice);
    if (st != VL53L0X_ERROR_NONE) return ESP_FAIL;

    st = VL53L0X_StaticInit(pDevice);
    if (st != VL53L0X_ERROR_NONE) return ESP_FAIL;

    st = VL53L0X_PerformRefSpadManagement(pDevice, &refSpadCount, &isApertureSpads);
    if (st != VL53L0X_ERROR_NONE) return ESP_FAIL;

    st = VL53L0X_PerformRefCalibration(pDevice, &VhvSettings, &PhaseCal);
    if (st != VL53L0X_ERROR_NONE) return ESP_FAIL;

    st = VL53L0X_SetDeviceMode(pDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (st != VL53L0X_ERROR_NONE) return ESP_FAIL;

    st = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pDevice, timing_budget_us);
    if (st != VL53L0X_ERROR_NONE) return ESP_FAIL;

    return ESP_OK;
}

static void IRAM_ATTR vl53l0x_gpio_isr_handler(void *arg)
{
    vl53l0x_dev_t *dev = (vl53l0x_dev_t *)arg;
    BaseType_t task_woken = pdFALSE;

    if (dev && dev->gpio_ready_sem) {
        xSemaphoreGiveFromISR(dev->gpio_ready_sem, &task_woken);
    }

    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* =========================
 *  Multi-sensor address assign using XSHUT
 * ========================= */

esp_err_t vl53l0x_multi_assign_addresses(const vl53l0x_slot_t *slots,
                                        int slot_count,
                                        uint32_t boot_delay_ms)
{
    if (!slots || slot_count <= 0) return ESP_ERR_INVALID_ARG;

    bool seen_addr[128] = {0};
    for (int i = 0; i < slot_count; i++) {
        uint8_t addr = slots[i].new_addr_7b;

        if (addr < 0x08 || addr > 0x77) {
            ESP_LOGE(TAG, "Invalid 7-bit addr 0x%02X slot=%d", addr, i);
            return ESP_ERR_INVALID_ARG;
        }

        if (addr == VL53L0X_I2C_ADDRESS_DEFAULT_7B) {
            ESP_LOGE(TAG, "Default addr 0x%02X not allowed slot=%d", addr, i);
            return ESP_ERR_INVALID_ARG;
        }

        if (seen_addr[addr]) {
            ESP_LOGE(TAG, "Duplicate addr 0x%02X slot=%d", addr, i);
            return ESP_ERR_INVALID_ARG;
        }
        seen_addr[addr] = true;
    }

    /* 1) Initialize XSHUT GPIOs and power all sensors off. */
    for (int i = 0; i < slot_count; i++) {
        gpio_num_t g = slots[i].xshut_gpio;
        if (g == GPIO_NUM_MAX) return ESP_ERR_INVALID_ARG;

        gpio_config_t io = {0};
        io.mode = GPIO_MODE_OUTPUT;
        io.pin_bit_mask = (1ULL << g);
        esp_err_t err = gpio_config(&io);
        if (err != ESP_OK) return err;

        gpio_set_level(g, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 2) One-by-one: power on -> set address -> keep on. */
    for (int i = 0; i < slot_count; i++) {

        gpio_set_level(slots[i].xshut_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(boot_delay_ms));

        /* The sensor must respond at the default address. */
        esp_err_t err = vl53l0x_i2c_probe(VL53L0X_I2C_ADDRESS_DEFAULT_7B);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Probe default 0x%02X failed slot=%d err=%d",
                     VL53L0X_I2C_ADDRESS_DEFAULT_7B, i, (int)err);
            return err;
        }

        VL53L0X_Dev_t st = {0};
        st.I2cDevAddr = (uint8_t)(VL53L0X_I2C_ADDRESS_DEFAULT_7B << 1); /* 8-bit left-aligned */
        st.comms_speed_khz = 400;

        /* ST expects a left-aligned address (7-bit << 1). */
        VL53L0X_Error st_err = VL53L0X_SetDeviceAddress(&st, (uint8_t)(slots[i].new_addr_7b << 1));
        if (st_err != VL53L0X_ERROR_NONE) {
            ESP_LOGE(TAG, "SetDeviceAddress slot=%d failed st=%d", i, (int)st_err);
            return ESP_FAIL;
        }

        vTaskDelay(pdMS_TO_TICKS(5));

        err = vl53l0x_i2c_probe(slots[i].new_addr_7b);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Probe new 0x%02X failed slot=%d err=%d",
                     slots[i].new_addr_7b, i, (int)err);
            return err;
        }

        ESP_LOGI(TAG, "Slot %d addr=0x%02X XSHUT=%d",
                 i, slots[i].new_addr_7b, (int)slots[i].xshut_gpio);
    }

    return ESP_OK;
}

/* =========================
 *  Public sensor API
 * ========================= */

esp_err_t vl53l0x_init(vl53l0x_dev_t *dev, uint32_t timing_budget_us)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    esp_err_t err = vl53l0x_i2c_probe(dev->addr_7b);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No response at 0x%02X err=%d", dev->addr_7b, (int)err);
        return err;
    }

    memset(&dev->st, 0, sizeof(dev->st));
    dev->st.I2cDevAddr = (uint8_t)(dev->addr_7b << 1);
    dev->st.comms_speed_khz = 400;

    // (void)VL53L0X_ResetDevice(&st);
    vTaskDelay(pdMS_TO_TICKS(10));

    err = st_init_sequence(&dev->st, timing_budget_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ST init failed addr=0x%02X", dev->addr_7b);
        return err;
    }

    dev->gpio_ready_enabled = false;
    dev->gpio_active_high = false;
    dev->int_gpio = GPIO_NUM_MAX;
    dev->gpio_ready_sem = NULL;

    ESP_LOGI(TAG, "Init OK addr=0x%02X budget=%" PRIu32 " us", dev->addr_7b, timing_budget_us);
    dev->inited = true;
    return ESP_OK;
}

esp_err_t vl53l0x_read_mm(vl53l0x_dev_t *dev, uint16_t *mm)
{
    if (!dev || !mm) return ESP_ERR_INVALID_ARG;
    if (!dev->inited) return ESP_ERR_INVALID_STATE;

    VL53L0X_RangingMeasurementData_t data;
    VL53L0X_Error e = VL53L0X_PerformSingleRangingMeasurement(&dev->st, &data);
    if (e != VL53L0X_ERROR_NONE) return ESP_FAIL;

    if (data.RangeStatus != 0) {
        // Debug output (Q16.16 -> Mcps).
        float sig = (float)data.SignalRateRtnMegaCps  / 65536.0f;
        float amb = (float)data.AmbientRateRtnMegaCps / 65536.0f;

        ESP_LOGW(TAG, "Invalid range: status=%u mm=%u sig=%.2fMcps amb=%.2fMcps",
                 (unsigned)data.RangeStatus,
                 (unsigned)data.RangeMilliMeter,
                 sig, amb);

        return ESP_ERR_INVALID_RESPONSE;
    }

    *mm = (uint16_t)data.RangeMilliMeter;
    return ESP_OK;
}

esp_err_t vl53l0x_enable_gpio_ready(vl53l0x_dev_t *dev,
                                   gpio_num_t int_gpio,
                                   bool active_high)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->inited) return ESP_ERR_INVALID_STATE;
    if (int_gpio == GPIO_NUM_MAX) return ESP_ERR_INVALID_ARG;

    VL53L0X_Error st = VL53L0X_SetDeviceMode(&dev->st, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (st != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "SetDeviceMode failed st=%d", (int)st);
        return ESP_FAIL;
    }

    VL53L0X_InterruptPolarity polarity = active_high
        ? VL53L0X_INTERRUPTPOLARITY_HIGH
        : VL53L0X_INTERRUPTPOLARITY_LOW;

    st = VL53L0X_SetGpioConfig(&dev->st,
                              0,
                              VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                              VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY,
                              polarity);
    if (st != VL53L0X_ERROR_NONE) {
        ESP_LOGE(TAG, "SetGpioConfig failed st=%d", (int)st);
        return ESP_FAIL;
    }

    if (!dev->gpio_ready_sem) {
        dev->gpio_ready_sem = xSemaphoreCreateBinary();
        if (!dev->gpio_ready_sem) return ESP_ERR_NO_MEM;
    }

    if (dev->gpio_ready_enabled) {
        gpio_isr_handler_remove(dev->int_gpio);
    }

    gpio_config_t io = {0};
    io.mode = GPIO_MODE_INPUT;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_ENABLE ;
    io.pin_bit_mask = (1ULL << int_gpio);
    io.intr_type = active_high ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE;

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;

    if (!isr_service_installed) {
        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            return err;
        }
        isr_service_installed = true;
    }

    err = gpio_isr_handler_add(int_gpio, vl53l0x_gpio_isr_handler, dev);
    if (err != ESP_OK) return err;

    dev->gpio_ready_enabled = true;
    dev->gpio_active_high = active_high;
    dev->int_gpio = int_gpio;

    ESP_LOGI(TAG, "GPIO ready enabled int=%d active_%s",
             (int)int_gpio,
             active_high ? "high" : "low");
    return ESP_OK;
}

esp_err_t vl53l0x_wait_gpio_ready(vl53l0x_dev_t *dev, TickType_t timeout)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->gpio_ready_enabled || !dev->gpio_ready_sem) {
        return ESP_ERR_INVALID_STATE;
    }

    return (xSemaphoreTake(dev->gpio_ready_sem, timeout) == pdTRUE)
        ? ESP_OK
        : ESP_ERR_TIMEOUT;
}
