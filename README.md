# ESP-IDF VL53L0X (ST API + platform ESP-IDF)

Ce dépôt fournit une intégration de la **ST VL53L0X API** (code `st_api/core`) avec une couche **platform** adaptée à **ESP-IDF** (I2C + logs), ainsi qu'une petite “driver layer” orientée usage.

## Organisation des couches

- `components/vl53l0x/st_api/core/*` : code ST “stock” (à garder inchangé autant que possible)
- `components/vl53l0x/st_api/platform/*` : **adapter** attendu par la ST API
  - `vl53l0x_platform.c` : implémente `VL53L0X_ReadMulti/WriteMulti/...` en s’appuyant sur des wrappers I2C ESP-IDF
  - `vl53l0x_i2c_platform.c` : implémente **uniquement** les wrappers I2C bas niveau utilisés par `vl53l0x_platform.c`
  - `vl53l0x_platform_log.c` : optionnel, mappe les traces ST vers `ESP_LOGx`
  - headers “ST platform” conservés : `vl53l0x_types.h`, `vl53l0x_platform.h`, `vl53l0x_i2c_platform.h`, `vl53l0x_platform_log.h`
- `components/vl53l0x/include/vl53l0x.h` + `components/vl53l0x/src/vl53l0x_driver.c` :
  - API ESP-IDF “pratique” (`vl53l0x_init`, `vl53l0x_read_mm`, adressage multi-capteurs via XSHUT, etc.)

L’objectif est de **séparer clairement** :
1) la ST API (intouchable),
2) la couche platform (contrat ST),
3) l’API “driver” ESP-IDF (ergonomie / multi-capteurs).

## API ESP-IDF (haut niveau)

### Initialisation du bus I2C

```c
#include "vl53l0x.h"

ESP_ERROR_CHECK(vl53l0x_i2c_master_init(GPIO_NUM_8, GPIO_NUM_9, 400000));
```

- Le wrapper utilise le **nouveau driver I2C** (`driver/i2c_master.h`).
- Une table de handles par adresse 7-bit est utilisée pour éviter de recréer les devices en boucle.

### Initialisation d’un capteur

```c
vl53l0x_dev_t dev = {
    .addr_7b = 0x29,
};

ESP_ERROR_CHECK(vl53l0x_init(&dev, /* timing_budget_us */ 33000));
```

### Lecture simple (mm)

```c
uint16_t mm = 0;
ESP_ERROR_CHECK(vl53l0x_read_mm(&dev, &mm));
```

## Multi-capteurs (XSHUT)

Le VL53L0X a une adresse par défaut **0x29**. Pour plusieurs capteurs sur le même bus, il faut :
1) mettre tous les XSHUT à 0,
2) réveiller **un seul** capteur,
3) lui assigner une nouvelle adresse (`VL53L0X_SetDeviceAddress`),
4) répéter.

L’helper `vl53l0x_multi_assign_addresses()` le fait pour vous.

```c
vl53l0x_slot_t slots[] = {
    { .xshut_gpio = GPIO_NUM_4, .new_addr_7b = 0x30 },
    { .xshut_gpio = GPIO_NUM_5, .new_addr_7b = 0x31 },
};

ESP_ERROR_CHECK(vl53l0x_multi_assign_addresses(slots, 2, /* boot_delay_ms */ 5));
```

Ensuite, vous pouvez créer 2 instances :

```c
vl53l0x_dev_t a = { .addr_7b = 0x30 };
vl53l0x_dev_t b = { .addr_7b = 0x31 };

ESP_ERROR_CHECK(vl53l0x_init(&a, 33000));
ESP_ERROR_CHECK(vl53l0x_init(&b, 33000));
```

## Logs ST -> ESP_LOGx (optionnel)

Par défaut, les logs ST ne sont pas activés.  
Si vous souhaitez activer les traces (pour debug), ajoutez une définition au composant :

```cmake
target_compile_definitions(${COMPONENT_LIB} PRIVATE VL53L0X_LOG_ENABLE=1)
```

Le mapping est réalisé dans `st_api/platform/src/vl53l0x_platform_log.c` via `esp_log_writev()`.

## Notes ESP-IDF

- La couche platform (ST) ne doit **pas** contenir de logique produit (multi-capteurs, init “user-friendly”, etc.).
- Le bus I2C “master” est préféré à une implémentation maison : il est maintenu par Espressif, thread-safe au bon niveau, et intégré au modèle de drivers ESP-IDF.

