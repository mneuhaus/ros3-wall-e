#include "DEV_Config.h"
#include "LCD_1in28.h"

// LCD is 240x240 pixels
#define LCD_WIDTH 240
#define LCD_HEIGHT 240

int main(void)
{
    // Initialize hardware
    if(DEV_Module_Init() != 0) {
        return -1;
    }
    
    // Initialize LCD
    LCD_1IN28_Init(0);
    LCD_1IN28_Clear(BLACK);  // Start with black background
    DEV_SET_PWM(100);  // Set backlight to 100%
    
    // Create a simple lens-like pattern
    uint16_t *image = (uint16_t *)malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(uint16_t));
    if (!image) {
        return -1;
    }
    
    // Draw concentric circles with color gradient
    for (int y = 0; y < LCD_HEIGHT; y++) {
        for (int x = 0; x < LCD_WIDTH; x++) {
            int dx = x - LCD_WIDTH/2;
            int dy = y - LCD_HEIGHT/2;
            int dist = (int)sqrt(dx*dx + dy*dy);
            
            // Create a blue-purple gradient based on distance
            uint8_t blue = (dist < 100) ? (255 - dist*2) : 0;
            uint8_t red = (dist < 100) ? (dist*2) : 0;
            
            // Convert to RGB565 format
            uint16_t color = ((red & 0xF8) << 8) | ((red & 0xFC) << 3) | (blue >> 3);
            image[y * LCD_WIDTH + x] = color;
        }
    }
    
    // Display the image
    LCD_1IN28_Display(image);
    
    // Main loop
    while(1) {
        DEV_Delay_ms(100);
    }

    // Cleanup
    free(image);
    DEV_Module_Exit();
    return 0;
}
