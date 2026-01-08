#include "vl53l0x.h"

#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "vl53l0x_api.h"
#include "vl53l0x_def.h"

static const char *TAG = "vl53l0x";

/* =========================
 *  I2C (new driver) state
 * ========================= */

static i2c_master_bus_handle_t s_bus = NULL;
/* Cache simple 7-bit -> dev handle (0..127) */
static i2c_master_dev_handle_t s_dev[128] = {0};

/* Valeurs par défaut */
#define VL53_I2C_TIMEOUT_MS      (50)
#define VL53_I2C_ADDR_7B_DEFAULT (0x29)
#define VL53_I2C_CLK_DEFAULT_HZ  (400000)

/* =========================
 *  Internal I2C helpers
 * ========================= */

static esp_err_t i2c_bus_init_once(gpio_num_t sda, gpio_num_t scl)
{
    if (s_bus != NULL) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1, /* auto-alloc */
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 16,
        .flags = {
            .enable_internal_pullup = 1, /* mets 0 si pullups externes */
        },
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err == ESP_ERR_INVALID_STATE) {
        /* déjà créé (rare, selon usage), on considère OK */
        return ESP_OK;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %d", (int)err);
        return err;
    }

    return ESP_OK;
}

static esp_err_t get_dev_handle(uint8_t addr_7b, uint32_t clk_hz, i2c_master_dev_handle_t *out_dev)
{
    if (!out_dev) return ESP_ERR_INVALID_ARG;
    if (addr_7b >= 128) return ESP_ERR_INVALID_ARG;
    if (s_bus == NULL) return ESP_ERR_INVALID_STATE;

    if (clk_hz == 0) clk_hz = VL53_I2C_CLK_DEFAULT_HZ;

    if (s_dev[addr_7b] == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr_7b,
            .scl_speed_hz = clk_hz,
        };

        esp_err_t err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev[addr_7b]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_add_device addr=0x%02X failed: %d", addr_7b, (int)err);
            return err;
        }
    }

    *out_dev = s_dev[addr_7b];
    return ESP_OK;
}

/* =========================
 *  Exported I2C API for ST platform layer
 *  (vl53l0x_platform_espidf.c must call these)
 * ========================= */

esp_err_t vl53l0x_i2c_master_init(gpio_num_t sda, gpio_num_t scl, uint32_t clk_hz)
{
    (void)clk_hz; /* fréquence configurée au niveau device dans get_dev_handle() */

    /* Reset cache devices si besoin (optionnel). Ici, on conserve si déjà init. */
    return i2c_bus_init_once(sda, scl);
}

esp_err_t vl53l0x_i2c_probe(uint8_t addr_7b)
{
    if (s_bus == NULL) return ESP_ERR_INVALID_STATE;
    return i2c_master_probe(s_bus, addr_7b, VL53_I2C_TIMEOUT_MS);
}

esp_err_t vl53l0x_i2c_write_reg(uint8_t addr_7b, uint8_t reg,
                               const uint8_t *data, size_t len,
                               uint32_t clk_hz)
{
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = get_dev_handle(addr_7b, clk_hz, &dev);
    if (err != ESP_OK) return err;

    uint8_t tmp[1 + 256];
    if (len > 256) return ESP_ERR_INVALID_ARG;

    tmp[0] = reg;
    if (len && data) memcpy(&tmp[1], data, len);

    return i2c_master_transmit(dev, tmp, 1 + len, VL53_I2C_TIMEOUT_MS);
}

esp_err_t vl53l0x_i2c_read_reg(uint8_t addr_7b, uint8_t reg,
                              uint8_t *data, size_t len,
                              uint32_t clk_hz)
{
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = get_dev_handle(addr_7b, clk_hz, &dev);
    if (err != ESP_OK) return err;

    return i2c_master_transmit_receive(dev, &reg, 1, data, len, VL53_I2C_TIMEOUT_MS);
}

/* =========================
 *  ST init sequence
 * ========================= */

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
    if (s_bus == NULL) return ESP_ERR_INVALID_STATE;

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
        esp_err_t err = vl53l0x_i2c_probe(VL53_I2C_ADDR_7B_DEFAULT);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Probe default 0x%02X failed slot=%d err=%d",
                     VL53_I2C_ADDR_7B_DEFAULT, i, (int)err);
            return err;
        }

        VL53L0X_Dev_t st = {0};
        st.I2cDevAddr = VL53_I2C_ADDR_7B_DEFAULT; /* 7-bit */
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
    if (s_bus == NULL) return ESP_ERR_INVALID_STATE;

    esp_err_t err = vl53l0x_i2c_probe(dev->addr_7b);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No response at 0x%02X err=%d", dev->addr_7b, (int)err);
        return err;
    }

    VL53L0X_Dev_t st = {0};
    st.I2cDevAddr = dev->addr_7b;
    st.comms_speed_khz = 400;

    (void)VL53L0X_ResetDevice(&st);
    vTaskDelay(pdMS_TO_TICKS(5));

    err = st_init_sequence(&st, timing_budget_us);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ST init failed addr=0x%02X", dev->addr_7b);
        return err;
    }

    ESP_LOGI(TAG, "Init OK addr=0x%02X budget=%" PRIu32 " us", dev->addr_7b, timing_budget_us);
    return ESP_OK;
}

esp_err_t vl53l0x_read_mm(vl53l0x_dev_t *dev, uint16_t *out_mm)
{
    if (!dev || !out_mm) return ESP_ERR_INVALID_ARG;
    if (s_bus == NULL) return ESP_ERR_INVALID_STATE;

    VL53L0X_Dev_t st = {0};
    st.I2cDevAddr = dev->addr_7b;
    st.comms_speed_khz = 400;

    VL53L0X_RangingMeasurementData_t data = {0};
    VL53L0X_Error st_err = VL53L0X_PerformSingleRangingMeasurement(&st, &data);
    if (st_err != VL53L0X_ERROR_NONE) {
        ESP_LOGW(TAG, "Ranging failed st=%d addr=0x%02X", (int)st_err, dev->addr_7b);
        return ESP_FAIL;
    }

    if (data.RangeStatus != 0) {
        return ESP_FAIL;
    }

    *out_mm = data.RangeMilliMeter;
    return ESP_OK;
}
