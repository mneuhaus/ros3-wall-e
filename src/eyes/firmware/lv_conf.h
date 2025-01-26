/**
 * @file lv_conf.h
 * Configuration file for v8.3.9
 */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/* Color depth: 1 (1 byte per pixel), 8 (RGB332), 16 (RGB565), 32 (ARGB8888) */
#define LV_COLOR_DEPTH 16

/* Use a custom tick source instead of the built-in one */
#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
    #define LV_TICK_CUSTOM_INCLUDE "hardware/timer.h"
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR (time_us_64() / 1000)
#endif

/* Memory settings */
#define LV_MEM_CUSTOM 1
#if LV_MEM_CUSTOM == 0
    #define LV_MEM_SIZE (32U * 1024U)
#else
    #define LV_MEM_CUSTOM_INCLUDE <stdlib.h>
    #define LV_MEM_CUSTOM_ALLOC   malloc
    #define LV_MEM_CUSTOM_FREE    free
#endif

/* Screen resolution */
#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 240

/* Enable GPU */
#define LV_USE_GPU_RP2040_PIO 1

/* Default screen refresh period */
#define LV_DISP_DEF_REFR_PERIOD 30

/* Render Engine: Choose one */
#define LV_USE_DRAW_SW 1

/* Enable/disable built-in fonts */
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_DEFAULT &lv_font_montserrat_12

/* Enable logging */
#define LV_USE_LOG 1
#if LV_USE_LOG
    #define LV_LOG_LEVEL LV_LOG_LEVEL_INFO
    #define LV_LOG_PRINTF 1
#endif

#endif /* LV_CONF_H */
