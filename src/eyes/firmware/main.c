#include "DEV_Config.h"
#include "LCD_1in28.h"

int main(void)
{
    // Initialize hardware
    if(DEV_Module_Init() != 0) {
        return -1;
    }
    
    // Initialize LCD
    LCD_1IN28_Init(0);
    LCD_1IN28_Clear(RED);  // Fill screen with red to test
    DEV_SET_PWM(100);  // Set backlight to 100%
    
    while(1) {
        DEV_Delay_ms(100);
    }

    DEV_Module_Exit();
    return 0;
}
