#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "DEV_Config.h"
#include <stdint.h>

// Display dimensions
#define DISPLAY_WIDTH  240
#define DISPLAY_HEIGHT 240

// Common colors in RGB565 format
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F

// Function declarations
void Display_Init(void);
void Display_Clear(uint16_t color);
void Display_SetWindow(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);
void Display_WriteData_16Bit(uint16_t data);
void Display_DrawImage(const uint16_t* image);

#endif // _DISPLAY_H_
