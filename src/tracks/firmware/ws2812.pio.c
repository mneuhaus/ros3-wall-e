#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

// Minimal WS2812 PIO program instructions, from Pico SDK examples.
static const uint16_t ws2812_program_instructions[] = {
    0x6221, // out x, 24
    0x1123, // set y, 31 ; delay
    0x80a4, // jmp x--, 0
    0x0000, // nop      ; fill delay slot
};

const pio_program_t ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1,
};

void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, uint32_t freq, bool rgb) {
    // Configure the state machine for WS2812 output.
    pio_sm_config c = pio_get_default_sm_config();
    // Set which pin the state machine will control.
    sm_config_set_set_pins(&c, pin, 1);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    // Configure the shift-out: shift left, autopull enabled, threshold 24 bits.
    sm_config_set_out_shift(&c, true, true, 24);
    // Join FIFO to make more efficient use of TX FIFO.
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    // Set clock divider for the desired frequency.
    float div = (float)clock_get_hz(clk_sys) / freq;
    sm_config_set_clkdiv(&c, div);
    // Initialize and enable the state machine.
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
