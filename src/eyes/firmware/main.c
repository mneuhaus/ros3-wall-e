#include "DEV_Config.h"
#include "LCD_1in28.h"
#include "../generated/image_data.h"

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
    
    // Display the lens image
    LCD_1IN28_Display((uint16_t *)image_data);
    
    // Main loop
    while(1) {
        DEV_Delay_ms(100);
    }

    DEV_Module_Exit();
    return 0;
}
