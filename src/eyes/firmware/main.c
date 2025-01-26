#include "DEV_Config.h"
#include "Display.h"
#include "generated/image_data.h"

int main(void)
{
    // Initialize hardware
    if(DEV_Module_Init() != 0) {
        return -1;
    }
    
    // Initialize display
    Display_Init();
    Display_Clear(COLOR_BLACK);
    DEV_SET_PWM(100);  // Full brightness
    
    // Display the lens image
    Display_DrawImage(image_data);
    
    // Main loop
    while(1) {
        DEV_Delay_ms(100);
    }

    DEV_Module_Exit();
    return 0;
}
