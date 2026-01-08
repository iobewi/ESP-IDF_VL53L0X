#pragma once

#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"
#include "driver/gpio.h"

/* =========================
 *  Constants
 * ========================= */

/* Adresse I2C par défaut du VL53L0X (7-bit) */
#define VL53L0X_I2C_ADDRESS_DEFAULT_7B  (0x29)

/* =========================
 *  Public data structures
 * ========================= */

/**
 * @brief Représente un capteur VL53L0X déjà adressé
 */
typedef struct {
    uint8_t     addr_7b;     /**< Adresse I2C 7-bit */
} vl53l0x_dev_t;

/**
 * @brief Slot utilisé pour l’assignation multi-capteurs via XSHUT
 */
typedef struct {
    gpio_num_t  xshut_gpio;  /**< GPIO connecté à XSHUT */
    uint8_t     new_addr_7b; /**< Nouvelle adresse I2C 7-bit à assigner */
} vl53l0x_slot_t;

/* =========================
 *  I2C – initialisation (nouveau driver)
 * ========================= */

/**
 * @brief Initialise le bus I2C maître (nouveau driver ESP-IDF)
 *
 * À appeler UNE FOIS avant toute utilisation du capteur.
 *
 * @param sda GPIO SDA
 * @param scl GPIO SCL
 * @param clk_hz Fréquence I2C (typiquement 400000)
 */
esp_err_t vl53l0x_i2c_master_init(gpio_num_t sda,
                                 gpio_num_t scl,
                                 uint32_t clk_hz);

/**
 * @brief Probe une adresse I2C (ACK/NACK)
 *
 * @param addr_7b Adresse 7-bit
 */
esp_err_t vl53l0x_i2c_probe(uint8_t addr_7b);

/* =========================
 *  I2C – primitives utilisées par la couche ST (platform)
 * ========================= */

/**
 * @brief Écriture registre I2C (utilisée par la ST API)
 */
esp_err_t vl53l0x_i2c_write_reg(uint8_t addr_7b,
                               uint8_t reg,
                               const uint8_t *data,
                               size_t len,
                               uint32_t clk_hz);

/**
 * @brief Lecture registre I2C (utilisée par la ST API)
 */
esp_err_t vl53l0x_i2c_read_reg(uint8_t addr_7b,
                              uint8_t reg,
                              uint8_t *data,
                              size_t len,
                              uint32_t clk_hz);

/* =========================
 *  Multi-capteurs (XSHUT)
 * ========================= */

/**
 * @brief Assigne des adresses I2C uniques à plusieurs VL53L0X
 *
 * Tous les capteurs sont mis en XSHUT bas, puis activés un par un :
 *  - activation XSHUT
 *  - adresse par défaut 0x29
 *  - VL53L0X_SetDeviceAddress()
 *
 * @param slots Tableau des slots capteurs
 * @param slot_count Nombre de capteurs
 * @param boot_delay_ms Délai après sortie XSHUT (typ. 2–10 ms)
 */
esp_err_t vl53l0x_multi_assign_addresses(const vl53l0x_slot_t *slots,
                                        int slot_count,
                                        uint32_t boot_delay_ms);

/* =========================
 *  Capteur – API haut niveau
 * ========================= */

/**
 * @brief Initialise un capteur VL53L0X déjà adressé
 *
 * @param dev Capteur
 * @param timing_budget_us Budget de mesure en microsecondes (ex: 33000)
 */
esp_err_t vl53l0x_init(vl53l0x_dev_t *dev,
                       uint32_t timing_budget_us);

/**
 * @brief Effectue une mesure de distance (mm)
 *
 * @param dev Capteur
 * @param out_mm Distance mesurée en millimètres
 */
esp_err_t vl53l0x_read_mm(vl53l0x_dev_t *dev,
                          uint16_t *out_mm);
