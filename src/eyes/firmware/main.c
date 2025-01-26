#include "LVGL_example.h"

int main(void)
{
    stdio_init_all();
    printf("Initializing Wall-E's Eyes...\n");

    // Initialize LVGL and display
    LVGL_Init();
    
    // Create a simple label
    lv_obj_t * label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "WALL-E");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    
    // Main loop
    while(1) {
        lv_timer_handler();
        sleep_ms(5);
    }
    
    return 0;
}
