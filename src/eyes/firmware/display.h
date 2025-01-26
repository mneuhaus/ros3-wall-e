#pragma once

#include "lvgl.h"
#include <cstdint>

// Display configuration
#define DISP_WIDTH 240
#define DISP_HEIGHT 240

// GC9A01A Commands
#define CMD_CASET   0x2A
#define CMD_RASET   0x2B
#define CMD_RAMWR   0x2C
#define CMD_SLPOUT  0x11
#define CMD_DISPON  0x29
#define CMD_COLMOD  0x3A
#define CMD_MADCTL  0x36

// Function declarations
void display_init();
void display_flush_ready();
void display_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
void display_set_pixel(uint16_t x, uint16_t y, lv_color_t color);
void display_rounder(lv_disp_drv_t * disp_drv, lv_area_t * area);

// Display driver structure
extern lv_disp_drv_t disp_drv;
extern lv_disp_draw_buf_t draw_buf;
extern lv_color_t buf_1[DISP_WIDTH * 10];
extern lv_color_t buf_2[DISP_WIDTH * 10];
