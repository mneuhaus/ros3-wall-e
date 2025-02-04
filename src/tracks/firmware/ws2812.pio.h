#ifndef WS2812_PIO_H
#define WS2812_PIO_H

#include "hardware/pio.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const pio_program_t ws2812_program;

void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, uint32_t freq, bool rgb);

static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

#ifdef __cplusplus
}
#endif

#endif // WS2812_PIO_H
