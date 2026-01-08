idf_component_register(
    SRCS
        platform/src/vl53l0x_platform.c
        platform/src/vl53l0x_i2c_platform.c

        # ---- ST API ----
        "st_api/core/src/vl53l0x_api.c"
        "st_api/core/src/vl53l0x_api_calibration.c"
        "st_api/core/src/vl53l0x_api_core.c"
        "st_api/core/src/vl53l0x_api_ranging.c"
        "st_api/core/src/vl53l0x_api_strings.c"

    INCLUDE_DIRS
        "include"

        # headers ST
        "st_api/core/inc"
        "st_api/platform/inc"
        "platform/inc"

    REQUIRES
        esp_driver_i2c
        esp_driver_gpio
        freertos
        esp_timer
        log
)

# Laisse -Wall/-Wextra, mais évite que des warnings ST cassent le build (IDF met -Werror globalement).
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall -Wextra
    -Wno-error=maybe-uninitialized
    -Wno-error=absolute-value
)
