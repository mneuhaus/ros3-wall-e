#include "pico/stdlib.h"
#include <hardware/pio.h>
#include "ws2812.pio.h"
#ifndef WS2812_PROGRAM_DEFINED
const pio_program_t ws2812_program = {
    .instructions = NULL,
    .length = 0,
    .origin = -1,
};
#define WS2812_PROGRAM_DEFINED
#endif

#define NEOPIXEL_PIN 16
#define NUM_PIXELS 1

int main() {
    stdio_init_all();
    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, NEOPIXEL_PIN, 800000, false);

    while (true) {
        // Set neopixel to green (0x00FF00)
        put_pixel(pio, sm, 0x00FF00);
        sleep_ms(500);
        // Turn neopixel off
        put_pixel(pio, sm, 0x000000);
        sleep_ms(500);
    }
    return 0;
}
