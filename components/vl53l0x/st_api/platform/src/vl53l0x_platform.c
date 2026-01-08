#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "vl53l0x_platform.h"
#include "vl53l0x_def.h"

// Wrappers I2C (new driver) exposés par ton composant
extern esp_err_t vl53l0x_i2c_write_reg(uint8_t addr_7b, uint8_t reg,
                                      const uint8_t *data, size_t len,
                                      uint32_t clk_hz);
extern esp_err_t vl53l0x_i2c_read_reg(uint8_t addr_7b, uint8_t reg,
                                     uint8_t *data, size_t len,
                                     uint32_t clk_hz);

static const char *TAG = "vl53_platform";

static inline uint32_t dev_clk_hz(const VL53L0X_DEV Dev)
{
    // ST stocke souvent des kHz dans comms_speed_khz
    uint32_t khz = (Dev && Dev->comms_speed_khz) ? Dev->comms_speed_khz : 400;
    return khz * 1000U;
}

static inline uint8_t dev_addr_7b(const VL53L0X_DEV Dev)
{
    // IMPORTANT:
    // Dans l’API ST, I2cDevAddr est historiquement une adresse 8-bit "left aligned" (7b << 1).
    // Ex: 0x29 -> 0x52
    // On convertit ici en 7-bit pour nos wrappers ESP-IDF.
    if (!Dev) return 0;

    uint8_t a = Dev->I2cDevAddr;

    // Si c’est déjà une 7-bit (rare, mais possible selon ports), on ne la décale pas.
    // Heuristique simple : si LSB=1, c’est souvent une 7-bit (ex: 0x29), sinon souvent 8-bit (0x52).
    // On privilégie le cas standard ST: 8-bit => >> 1.
    if ((a & 0x01) == 0) {
        return (uint8_t)(a >> 1);
    }
    return a;
}

/**
 * ST API platform: écrit plusieurs octets
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    const uint8_t addr7 = dev_addr_7b(Dev);
    const uint32_t clk  = dev_clk_hz(Dev);

    esp_err_t err = vl53l0x_i2c_write_reg(addr7, index, pdata, (size_t)count, clk);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WriteMulti err=%d addr7=0x%02X reg=0x%02X", (int)err, addr7, index);
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    const uint8_t addr7 = dev_addr_7b(Dev);
    const uint32_t clk  = dev_clk_hz(Dev);

    esp_err_t err = vl53l0x_i2c_read_reg(addr7, index, pdata, (size_t)count, clk);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ReadMulti err=%d addr7=0x%02X reg=0x%02X", (int)err, addr7, index);
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_ReadByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata)
{
    return VL53L0X_ReadMulti(Dev, index, pdata, 1);
}

VL53L0X_Error VL53L0X_WriteWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buf[2] = { (uint8_t)(data >> 8), (uint8_t)(data & 0xFF) };
    return VL53L0X_WriteMulti(Dev, index, buf, 2);
}

VL53L0X_Error VL53L0X_ReadWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *pdata)
{
    uint8_t buf[2] = {0};
    VL53L0X_Error st = VL53L0X_ReadMulti(Dev, index, buf, 2);
    if (st != VL53L0X_ERROR_NONE) return st;

    *pdata = (uint16_t)((buf[0] << 8) | buf[1]);
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buf[4] = {
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF)
    };
    return VL53L0X_WriteMulti(Dev, index, buf, 4);
}

VL53L0X_Error VL53L0X_ReadDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *pdata)
{
    uint8_t buf[4] = {0};
    VL53L0X_Error st = VL53L0X_ReadMulti(Dev, index, buf, 4);
    if (st != VL53L0X_ERROR_NONE) return st;

    *pdata = ((uint32_t)buf[0] << 24) |
             ((uint32_t)buf[1] << 16) |
             ((uint32_t)buf[2] << 8)  |
             ((uint32_t)buf[3]);
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    vTaskDelay(pdMS_TO_TICKS(1));
    return VL53L0X_ERROR_NONE;
}

void VL53L0X_WaitMs(VL53L0X_DEV Dev, int32_t wait_ms)
{
    (void)Dev;
    if (wait_ms <= 0) return;
    vTaskDelay(pdMS_TO_TICKS((uint32_t)wait_ms));
}
