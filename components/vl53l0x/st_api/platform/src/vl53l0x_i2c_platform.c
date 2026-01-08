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
#define VL53_I2C_CLK_DEFAULT_HZ  (100000)

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
        .flags = {
            .enable_internal_pullup = 0, /* mets 0 si pullups externes */
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

// ---- End of ESP-IDF I2C wrapper layer (used by ST platform) ----
