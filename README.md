# ESP-IDF VL53L0X (ST API + ESP-IDF platform)

> 🇫🇷 Version française: [README_FR.md](README_FR.md)

This repository provides an integration of the **ST VL53L0X API** (`st_api/core`) with an **ESP-IDF**-specific **platform** layer (I2C + logs), plus a small usage-oriented driver layer.

## Layer organization

- `components/vl53l0x/st_api/core/*`: stock ST code (keep unchanged whenever possible)
- `components/vl53l0x/st_api/platform/*`: **adapter** required by the ST API
  - `vl53l0x_platform.c`: implements `VL53L0X_ReadMulti/WriteMulti/...` using ESP-IDF I2C wrappers
  - `vl53l0x_i2c_platform.c`: implements only the low-level I2C wrappers used by `vl53l0x_platform.c`
  - `vl53l0x_platform_log.c`: optional, maps ST traces to `ESP_LOGx`
  - preserved “ST platform” headers: `vl53l0x_types.h`, `vl53l0x_platform.h`, `vl53l0x_i2c_platform.h`, `vl53l0x_platform_log.h`
- `components/vl53l0x/include/vl53l0x.h` + `components/vl53l0x/src/vl53l0x_driver.c`:
  - ergonomic ESP-IDF API (`vl53l0x_init`, `vl53l0x_read_mm`, multi-sensor addressing via XSHUT, etc.)

The goal is to **clearly separate**:
1) the ST API (untouched),
2) the platform layer (ST contract),
3) the ESP-IDF driver API (ergonomics / multi-sensor).

## Prerequisites

- **ESP-IDF** (recent version with `driver/i2c_master.h`)
- A properly powered **VL53L0X** sensor (3.3V)

## Quick start

### I2C bus initialization

```c
#include "vl53l0x.h"

ESP_ERROR_CHECK(vl53l0x_i2c_master_init(GPIO_NUM_8, GPIO_NUM_9, 400000));
```

- The wrapper uses the **new I2C driver** (`driver/i2c_master.h`).
- A handle table per 7-bit address avoids recreating devices repeatedly.

### Sensor initialization

```c
vl53l0x_dev_t dev = {
    .addr_7b = 0x29,
};

ESP_ERROR_CHECK(vl53l0x_init(&dev, /* timing_budget_us */ 33000));
```

#### ST addressing specificity (`I2cDevAddr`)

The ST API expects `I2cDevAddr` stored as **left-aligned 8-bit address** (`7b << 1`).
The platform layer consistently converts to 7-bit addresses for I2C access.
If you manipulate ST structures directly, keep this format in mind to avoid
unexpected shifts (e.g., `0x29` stored as 7-bit becomes `0x14` after conversion).

### Single read (mm)

```c
uint16_t mm = 0;
ESP_ERROR_CHECK(vl53l0x_read_mm(&dev, &mm));
```

## GPIO “data ready” (GPIO/INT)

To drive measurements via the sensor interrupt:

```c
ESP_ERROR_CHECK(vl53l0x_enable_gpio_ready(&dev, GPIO_NUM_7, true));

VL53L0X_StartMeasurement(&dev.st);
ESP_ERROR_CHECK(vl53l0x_wait_gpio_ready(&dev, pdMS_TO_TICKS(1000)));
VL53L0X_GetRangingMeasurementData(&dev.st, &data);
VL53L0X_ClearInterruptMask(&dev.st, 0);
```

A full example is available in `examples/gpio_ready_app`.
An example combining **multi-sensor** + **GPIO ready** is available in
`examples/multi_gpio_ready_app`.
An equivalent **multi-task** example (each sensor in its own task) is available in
`examples/multi_gpio_ready_multitask_app`.

### GPIO/INT wiring

- Connect **GPIO/INT** on the VL53L0X to an ESP-IDF input GPIO (`int_gpio`).
- Power the sensor at **3.3V** (adapt if your module requires 5V).
- Common ground.
- Polarity is configurable via `active_high` (e.g., `true` = rising edge).
- If your module lacks an internal pull-up, add an external one or enable
  `GPIO_PULLUP_ENABLE` on the ESP side.

## Multi-sensor (XSHUT)

The VL53L0X default address is **0x29**. For multiple sensors on the same bus, you must:
1) pull all XSHUT low,
2) wake **only one** sensor,
3) assign a new address (`VL53L0X_SetDeviceAddress`),
4) repeat.

The helper `vl53l0x_multi_assign_addresses()` handles this for you.

```c
vl53l0x_slot_t slots[] = {
    { .xshut_gpio = GPIO_NUM_4, .new_addr_7b = 0x30 },
    { .xshut_gpio = GPIO_NUM_5, .new_addr_7b = 0x31 },
};

ESP_ERROR_CHECK(vl53l0x_multi_assign_addresses(slots, 2, /* boot_delay_ms */ 5));
```

Then create two instances:

```c
vl53l0x_dev_t a = { .addr_7b = 0x30 };
vl53l0x_dev_t b = { .addr_7b = 0x31 };

ESP_ERROR_CHECK(vl53l0x_init(&a, 33000));
ESP_ERROR_CHECK(vl53l0x_init(&b, 33000));
```

## ST logs -> ESP_LOGx (optional)

By default, ST logs are disabled.  
To enable traces (for debugging), add a compile definition to the component:

```cmake
target_compile_definitions(${COMPONENT_LIB} PRIVATE VL53L0X_LOG_ENABLE=1)
```

Mapping happens in `st_api/platform/src/vl53l0x_platform_log.c` via `esp_log_writev()`.

## ESP-IDF notes

- The ST platform layer must **not** include product logic (multi-sensor, user-friendly init, etc.).
- The I2C “master” bus is preferred over a custom implementation: it is maintained by Espressif, sufficiently thread-safe, and integrated into the ESP-IDF driver model.
