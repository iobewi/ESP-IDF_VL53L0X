#include "vl53l0x_platform_log.h"

#ifdef VL53L0X_LOG_ENABLE

#include <stdarg.h>
#include <stdint.h>
#include "esp_log.h"

/*
 * La ST API utilise un système de trace optionnel.
 * Ici, on mappe vers ESP_LOGx.
 *
 * Pour activer:
 *   - ajouter -DVL53L0X_LOG_ENABLE=1 au composant (target_compile_definitions)
 *   - éventuellement définir VL53L0X_LOG_TAG
 */

#ifndef VL53L0X_LOG_TAG
#define VL53L0X_LOG_TAG "vl53_st"
#endif

uint32_t _trace_level = 0;

/* ST: configure un fichier de log; sur ESP-IDF on ignore "filename" et on garde level. */
int32_t VL53L0X_trace_config(char *filename, uint32_t modules, uint32_t level, uint32_t functions)
{
    (void)filename;
    (void)modules;
    (void)functions;
    _trace_level = level;
    return 0;
}

static void vl53_log_vprintf(uint32_t level, const char *fmt, va_list ap)
{
    /* Heuristique simple: TRACE_LEVEL_ERROR > WARN > INFO > DEBUG */
    if (level & TRACE_LEVEL_ERROR) {
        esp_log_writev(ESP_LOG_ERROR, VL53L0X_LOG_TAG, fmt, ap);
    } else if (level & TRACE_LEVEL_WARNING) {
        esp_log_writev(ESP_LOG_WARN,  VL53L0X_LOG_TAG, fmt, ap);
    } else if (level & TRACE_LEVEL_INFO) {
        esp_log_writev(ESP_LOG_INFO,  VL53L0X_LOG_TAG, fmt, ap);
    } else {
        esp_log_writev(ESP_LOG_DEBUG, VL53L0X_LOG_TAG, fmt, ap);
    }
}

void trace_print_module_function(uint32_t module, uint32_t level, uint32_t function, const char *format, ...)
{
    (void)module;
    (void)function;

    va_list ap;
    va_start(ap, format);
    vl53_log_vprintf(level, format, ap);
    va_end(ap);
}

#else  /* VL53L0X_LOG_ENABLE */

/* Compilé mais inactif: rien à faire (macros no-op dans le .h) */

#endif /* VL53L0X_LOG_ENABLE */
