#include <stdio.h>
#include "pico/stdlib.h"

int main(void)
{
    stdio_init_all();
    printf("Hello from Wall-E's Eyes!\n");
    
    while(1) {
        sleep_ms(1000);
        printf("Blink!\n");
    }
    
    return 0;
}
