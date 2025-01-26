#include "Display.h"

// GC9A01 LCD commands
#define CMD_CASET   0x2A
#define CMD_RASET   0x2B
#define CMD_RAMWR   0x2C
#define CMD_MADCTL  0x36

static void Display_WriteCommand(uint8_t command) {
    DEV_Digital_Write(LCD_DC_PIN, 0);
    DEV_SPI_WriteByte(LCD_SPI_PORT, command);
}

static void Display_WriteData(uint8_t data) {
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_SPI_WriteByte(LCD_SPI_PORT, data);
}

void Display_Init(void) {
    // Hardware reset
    DEV_Digital_Write(LCD_RST_PIN, 1);
    DEV_Delay_ms(100);
    DEV_Digital_Write(LCD_RST_PIN, 0);
    DEV_Delay_ms(100);
    DEV_Digital_Write(LCD_RST_PIN, 1);
    DEV_Delay_ms(100);

    // Display initialization sequence
    Display_WriteCommand(0xEF);
    
    Display_WriteCommand(0xEB);
    Display_WriteData(0x14);
    
    Display_WriteCommand(0xFE);
    Display_WriteCommand(0xEF);
    
    Display_WriteCommand(0xEB);
    Display_WriteData(0x14);
    
    Display_WriteCommand(0x84);
    Display_WriteData(0x40);
    
    Display_WriteCommand(0x85);
    Display_WriteData(0xFF);
    
    Display_WriteCommand(0x86);
    Display_WriteData(0xFF);
    
    Display_WriteCommand(0x87);
    Display_WriteData(0xFF);
    
    Display_WriteCommand(0x88);
    Display_WriteData(0x0A);
    
    Display_WriteCommand(0x89);
    Display_WriteData(0x21);
    
    Display_WriteCommand(0x8A);
    Display_WriteData(0x00);
    
    Display_WriteCommand(0x8B);
    Display_WriteData(0x80);
    
    Display_WriteCommand(0x8C);
    Display_WriteData(0x01);
    
    Display_WriteCommand(0x8D);
    Display_WriteData(0x01);
    
    Display_WriteCommand(0x8E);
    Display_WriteData(0xFF);
    
    Display_WriteCommand(0x8F);
    Display_WriteData(0xFF);
    
    Display_WriteCommand(0xB6);
    Display_WriteData(0x00);
    Display_WriteData(0x00);
    
    Display_WriteCommand(CMD_MADCTL);
    Display_WriteData(0x00);
    
    Display_WriteCommand(0x3A);
    Display_WriteData(0x05);
    
    Display_WriteCommand(0x90);
    Display_WriteData(0x08);
    Display_WriteData(0x08);
    Display_WriteData(0x08);
    Display_WriteData(0x08);
    
    Display_WriteCommand(0xBD);
    Display_WriteData(0x06);
    
    Display_WriteCommand(0xBC);
    Display_WriteData(0x00);
    
    Display_WriteCommand(0xFF);
    Display_WriteData(0x60);
    Display_WriteData(0x01);
    Display_WriteData(0x04);
    
    Display_WriteCommand(0xC3);
    Display_WriteData(0x13);
    Display_WriteCommand(0xC4);
    Display_WriteData(0x13);
    
    Display_WriteCommand(0xC9);
    Display_WriteData(0x22);
    
    Display_WriteCommand(0xBE);
    Display_WriteData(0x11);
    
    Display_WriteCommand(0xE1);
    Display_WriteData(0x10);
    Display_WriteData(0x0E);
    
    Display_WriteCommand(0xDF);
    Display_WriteData(0x21);
    Display_WriteData(0x0c);
    Display_WriteData(0x02);
    
    Display_WriteCommand(0xF0);
    Display_WriteData(0x45);
    Display_WriteData(0x09);
    Display_WriteData(0x08);
    Display_WriteData(0x08);
    Display_WriteData(0x26);
    Display_WriteData(0x2A);
    
    Display_WriteCommand(0xF1);
    Display_WriteData(0x43);
    Display_WriteData(0x70);
    Display_WriteData(0x72);
    Display_WriteData(0x36);
    Display_WriteData(0x37);
    Display_WriteData(0x6F);
    
    Display_WriteCommand(0xF2);
    Display_WriteData(0x45);
    Display_WriteData(0x09);
    Display_WriteData(0x08);
    Display_WriteData(0x08);
    Display_WriteData(0x26);
    Display_WriteData(0x2A);
    
    Display_WriteCommand(0xF3);
    Display_WriteData(0x43);
    Display_WriteData(0x70);
    Display_WriteData(0x72);
    Display_WriteData(0x36);
    Display_WriteData(0x37);
    Display_WriteData(0x6F);
    
    Display_WriteCommand(0xED);
    Display_WriteData(0x1B);
    Display_WriteData(0x0B);
    
    Display_WriteCommand(0xAE);
    Display_WriteData(0x77);
    
    Display_WriteCommand(0xCD);
    Display_WriteData(0x63);
    
    Display_WriteCommand(0x70);
    Display_WriteData(0x07);
    Display_WriteData(0x07);
    Display_WriteData(0x04);
    Display_WriteData(0x0E);
    Display_WriteData(0x0F);
    Display_WriteData(0x09);
    Display_WriteData(0x07);
    Display_WriteData(0x08);
    Display_WriteData(0x03);
    
    Display_WriteCommand(0xE8);
    Display_WriteData(0x34);
    
    Display_WriteCommand(0x62);
    Display_WriteData(0x18);
    Display_WriteData(0x0D);
    Display_WriteData(0x71);
    Display_WriteData(0xED);
    Display_WriteData(0x70);
    Display_WriteData(0x70);
    Display_WriteData(0x18);
    Display_WriteData(0x0F);
    Display_WriteData(0x71);
    Display_WriteData(0xEF);
    Display_WriteData(0x70);
    Display_WriteData(0x70);
    
    Display_WriteCommand(0x63);
    Display_WriteData(0x18);
    Display_WriteData(0x11);
    Display_WriteData(0x71);
    Display_WriteData(0xF1);
    Display_WriteData(0x70);
    Display_WriteData(0x70);
    Display_WriteData(0x18);
    Display_WriteData(0x13);
    Display_WriteData(0x71);
    Display_WriteData(0xF3);
    Display_WriteData(0x70);
    Display_WriteData(0x70);
    
    Display_WriteCommand(0x64);
    Display_WriteData(0x28);
    Display_WriteData(0x29);
    Display_WriteData(0xF1);
    Display_WriteData(0x01);
    Display_WriteData(0xF1);
    Display_WriteData(0x00);
    Display_WriteData(0x07);
    
    Display_WriteCommand(0x66);
    Display_WriteData(0x3C);
    Display_WriteData(0x00);
    Display_WriteData(0xCD);
    Display_WriteData(0x67);
    Display_WriteData(0x45);
    Display_WriteData(0x45);
    Display_WriteData(0x10);
    Display_WriteData(0x00);
    Display_WriteData(0x00);
    Display_WriteData(0x00);
    
    Display_WriteCommand(0x67);
    Display_WriteData(0x00);
    Display_WriteData(0x3C);
    Display_WriteData(0x00);
    Display_WriteData(0x00);
    Display_WriteData(0x00);
    Display_WriteData(0x01);
    Display_WriteData(0x54);
    Display_WriteData(0x10);
    Display_WriteData(0x32);
    Display_WriteData(0x98);
    
    Display_WriteCommand(0x74);
    Display_WriteData(0x10);
    Display_WriteData(0x85);
    Display_WriteData(0x80);
    Display_WriteData(0x00);
    Display_WriteData(0x00);
    Display_WriteData(0x4E);
    Display_WriteData(0x00);
    
    Display_WriteCommand(0x98);
    Display_WriteData(0x3e);
    Display_WriteData(0x07);
    
    Display_WriteCommand(0x35);
    Display_WriteCommand(0x21);
    
    Display_WriteCommand(0x11);
    DEV_Delay_ms(120);
    Display_WriteCommand(0x29);
    DEV_Delay_ms(20);
}

void Display_Clear(uint16_t color) {
    uint16_t i, j;
    Display_SetWindow(0, 0, DISPLAY_WIDTH-1, DISPLAY_HEIGHT-1);
    
    for(i = 0; i < DISPLAY_WIDTH; i++) {
        for(j = 0; j < DISPLAY_HEIGHT; j++) {
            Display_WriteData_16Bit(color);
        }
    }
}

void Display_SetWindow(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end) {
    Display_WriteCommand(CMD_CASET);
    Display_WriteData(x_start >> 8);
    Display_WriteData(x_start & 0xFF);
    Display_WriteData(x_end >> 8);
    Display_WriteData(x_end & 0xFF);
    
    Display_WriteCommand(CMD_RASET);
    Display_WriteData(y_start >> 8);
    Display_WriteData(y_start & 0xFF);
    Display_WriteData(y_end >> 8);
    Display_WriteData(y_end & 0xFF);
    
    Display_WriteCommand(CMD_RAMWR);
}

void Display_WriteData_16Bit(uint16_t data) {
    Display_WriteData(data >> 8);
    Display_WriteData(data & 0xFF);
}

void Display_DrawImage(const uint16_t* image) {
    Display_SetWindow(0, 0, DISPLAY_WIDTH-1, DISPLAY_HEIGHT-1);
    
    for(int i = 0; i < DISPLAY_WIDTH * DISPLAY_HEIGHT; i++) {
        Display_WriteData_16Bit(image[i]);
    }
}
