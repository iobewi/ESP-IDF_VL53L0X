idf_component_register(
    SRCS
        "src/vl53l0x.c"
        "src/vl53l0x_platform_espidf.c"

        # ---- ST API ----
        "src/st_api/core/src/vl53l0x_api.c"
        "src/st_api/core/src/vl53l0x_core.c"
        "src/st_api/core/src/vl53l0x_api_strings.c"
        "src/st_api/core/src/vl53l0x_api_ranging.c"
        "src/st_api/core/src/vl53l0x_api_calibration.c"
        "src/st_api/core/src/vl53l0x_api_debug.c"

    INCLUDE_DIRS
        "include"

        # headers ST
        "src/st_api/core/inc"
        "src/st_api/platform/inc"

    REQUIRES
        driver
        freertos
        esp_timer
        log
)

# Bonnes pratiques warnings (optionnel)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra)
