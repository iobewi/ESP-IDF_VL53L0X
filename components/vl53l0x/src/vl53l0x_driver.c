#include "vl53l0x.h"

#include <inttypes.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"

/*
 * NOTE ARCHI:
 * - Ce fichier contient la "driver layer" ESP-IDF (bus init, probe, init capteur, lecture).
 * - La ST API appelle la couche "platform" (st_api/platform) qui, elle, se contente
 *   de wrappers I2C byte/word/dword via vl53l0x_i2c_{read,write}_reg().
 */

static const char *TAG = "vl53l0x";

/* Ces symboles sont implémentés dans st_api/platform/src/vl53l0x_i2c_platform.c */
extern esp_err_t vl53l0x_i2c_master_init(gpio_num_t sda,
                                        gpio_num_t scl,
                                        uint32_t clk_hz);

extern esp_err_t vl53l0x_i2c_probe(uint8_t addr_7b);

/* ---- Implémentations issues de l'ancien vl53l0x_i2c_platform.c ---- */
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

    /* 1) XSHUT GPIO init + all OFF */
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

    /* 2) One-by-one: ON -> set addr -> keep ON */
    for (int i = 0; i < slot_count; i++) {

        gpio_set_level(slots[i].xshut_gpio, 1);
        vTaskDelay(pdMS_TO_TICKS(boot_delay_ms));

        /* Sensor must respond at default address */
        esp_err_t err = vl53l0x_i2c_probe(VL53L0X_I2C_ADDRESS_DEFAULT_7B);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Probe default 0x%02X failed slot=%d err=%d",
                     VL53L0X_I2C_ADDRESS_DEFAULT_7B, i, (int)err);
            return err;
        }

        VL53L0X_Dev_t st = {0};
        st.I2cDevAddr = (uint8_t)(VL53L0X_I2C_ADDRESS_DEFAULT_7B << 1); /* 8-bit left-aligned */
        st.comms_speed_khz = 400;

        /* ST expects left-aligned address => << 1 */
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
    dev->st.I2cDevAddr = dev->addr_7b;
    dev->st.comms_speed_khz = 400;

    //(void)VL53L0X_ResetDevice(&st);
    vTaskDelay(pdMS_TO_TICKS(10));

    err = st_init_sequence(&dev->st, timing_budget_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ST init failed addr=0x%02X", dev->addr_7b);
        return err;
    }

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
        // Debug (Q16.16 -> Mcps)
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
