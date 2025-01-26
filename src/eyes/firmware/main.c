#include "DEV_Config.h"
#include "LCD_1in28.h"
#include "generated/image_data.h"

int main(void)
{
    // Initialize hardware
    if(DEV_Module_Init() != 0) {
        return -1;
    }
    
    // Initialize LCD with no rotation (0)
    LCD_1IN28_Init(0);
    LCD_1IN28_Clear(BLACK);
    DEV_SET_PWM(100);  // Full brightness
    
    // Display the lens image
    LCD_1IN28_Display((uint16_t *)image_data);
    
    // Main loop
    while(1) {
        DEV_Delay_ms(100);
    }

    DEV_Module_Exit();
    return 0;
}
