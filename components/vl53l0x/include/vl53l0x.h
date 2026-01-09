#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "vl53l0x_api.h" 

/* =========================
 *  Constants
 * ========================= */

/* Default VL53L0X I2C address (7-bit). */
#define VL53L0X_I2C_ADDRESS_DEFAULT_7B  (0x29)

/* =========================
 *  Public data structures
 * ========================= */

/**
 * @brief Represents an already-addressed VL53L0X sensor.
 */
typedef struct {
    uint8_t     addr_7b;     /**< 7-bit I2C address. */
    VL53L0X_Dev_t st;   // Persistent ST device state.
    bool inited;
    bool gpio_ready_enabled;
    bool gpio_active_high;
    gpio_num_t int_gpio;
    SemaphoreHandle_t gpio_ready_sem;
} vl53l0x_dev_t;

/**
 * @brief Slot used for multi-sensor address assignment via XSHUT.
 */
typedef struct {
    gpio_num_t  xshut_gpio;  /**< GPIO connected to XSHUT. */
    uint8_t     new_addr_7b; /**< New 7-bit I2C address to assign. */
} vl53l0x_slot_t;

/* =========================
 *  I2C – initialization (new driver)
 * ========================= */

/**
 * @brief Initializes the I2C master bus (new ESP-IDF driver).
 *
 * Call once before any sensor usage.
 *
 * @param sda SDA GPIO.
 * @param scl SCL GPIO.
 * @param clk_hz I2C frequency (typically 400000).
 */
esp_err_t vl53l0x_i2c_master_init(gpio_num_t sda,
                                 gpio_num_t scl,
                                 uint32_t clk_hz);

/**
 * @brief Probes an I2C address (ACK/NACK).
 *
 * @param addr_7b 7-bit address.
 */
esp_err_t vl53l0x_i2c_probe(uint8_t addr_7b);

/* =========================
 *  I2C – primitives used by the ST platform layer
 * ========================= */

/**
 * @brief I2C register write (used by the ST API).
 */
esp_err_t vl53l0x_i2c_write_reg(uint8_t addr_7b,
                               uint8_t reg,
                               const uint8_t *data,
                               size_t len,
                               uint32_t clk_hz);

/**
 * @brief I2C register read (used by the ST API).
 */
esp_err_t vl53l0x_i2c_read_reg(uint8_t addr_7b,
                              uint8_t reg,
                              uint8_t *data,
                              size_t len,
                              uint32_t clk_hz);

/* =========================
 *  Multi-sensor (XSHUT)
 * ========================= */

/**
 * @brief Assigns unique I2C addresses to multiple VL53L0X sensors.
 *
 * All sensors are held in XSHUT low, then enabled one by one:
 *  - release XSHUT
 *  - use default address 0x29
 *  - call VL53L0X_SetDeviceAddress()
 *
 * @param slots Sensor slot array.
 * @param slot_count Number of sensors.
 * @param boot_delay_ms Delay after XSHUT release (typ. 2–10 ms).
 */
esp_err_t vl53l0x_multi_assign_addresses(const vl53l0x_slot_t *slots,
                                        int slot_count,
                                        uint32_t boot_delay_ms);

/* =========================
 *  Sensor – high-level API
 * ========================= */

/**
 * @brief Initializes an already-addressed VL53L0X sensor.
 *
 * @param dev Sensor handle.
 * @param timing_budget_us Measurement budget in microseconds (e.g., 33000).
 */
esp_err_t vl53l0x_init(vl53l0x_dev_t *dev,
                       uint32_t timing_budget_us);

/**
 * @brief Performs a distance measurement (mm).
 *
 * @param dev Sensor handle.
 * @param out_mm Measured distance in millimeters.
 */
esp_err_t vl53l0x_read_mm(vl53l0x_dev_t *dev,
                          uint16_t *out_mm);

/**
 * @brief Enables the GPIO "data ready" mode (GPIO/INT).
 *
 * Configures the sensor via the ST API to signal "new measure ready"
 * and sets up the ESP-IDF ISR/queue for event waiting.
 *
 * @param dev Sensor handle.
 * @param int_gpio ESP-IDF GPIO connected to the VL53L0X GPIO/INT pin.
 * @param active_high True if the signal is active high.
 */
esp_err_t vl53l0x_enable_gpio_ready(vl53l0x_dev_t *dev,
                                   gpio_num_t int_gpio,
                                   bool active_high);

/**
 * @brief Waits for a GPIO "data ready" edge.
 *
 * @param dev Sensor handle.
 * @param timeout FreeRTOS timeout (ticks).
 */
esp_err_t vl53l0x_wait_gpio_ready(vl53l0x_dev_t *dev,
                                 TickType_t timeout);
