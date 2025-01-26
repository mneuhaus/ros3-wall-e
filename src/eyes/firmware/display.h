#pragma once

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
void write_cmd(uint8_t cmd);
void write_data(uint8_t data);
void set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void init_display();
void display_image();

// Default test pattern - replace with actual image data
// 16-bit RGB565 format, 240x240 pixels
extern const uint16_t default_image[];
