#include "display.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

// Pin definitions
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_DC   4
#define PIN_CS   5
#define PIN_RST  6

// Display buffers
lv_disp_draw_buf_t draw_buf;
lv_color_t buf_1[DISP_WIDTH * 10];
lv_color_t buf_2[DISP_WIDTH * 10];
lv_disp_drv_t disp_drv;

// DMA channel
static int dma_tx;

static void write_cmd(uint8_t cmd) {
    gpio_put(PIN_DC, 0);  // Command mode
    gpio_put(PIN_CS, 0);
    spi_write_blocking(spi0, &cmd, 1);
    gpio_put(PIN_CS, 1);
}

static void write_data(uint8_t data) {
    gpio_put(PIN_DC, 1);  // Data mode
    gpio_put(PIN_CS, 0);
    spi_write_blocking(spi0, &data, 1);
    gpio_put(PIN_CS, 1);
}

static void set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    write_cmd(CMD_CASET);
    write_data(x1 >> 8);
    write_data(x1 & 0xFF);
    write_data(x2 >> 8);
    write_data(x2 & 0xFF);
    
    write_cmd(CMD_RASET);
    write_data(y1 >> 8);
    write_data(y1 & 0xFF);
    write_data(y2 >> 8);
    write_data(y2 & 0xFF);
}

void display_init() {
    // Initialize SPI at 62.5MHz
    spi_init(spi0, 62500000);
    
    // Configure GPIO pins
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    gpio_init(PIN_DC);
    gpio_init(PIN_CS);
    gpio_init(PIN_RST);
    
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    
    // Setup DMA
    dma_tx = dma_claim_unused_channel(true);
    
    // Reset display
    gpio_put(PIN_RST, 0);
    sleep_ms(100);
    gpio_put(PIN_RST, 1);
    sleep_ms(100);
    
    // Initialize GC9A01A
    write_cmd(CMD_SLPOUT);    // Sleep out
    sleep_ms(120);
    
    write_cmd(CMD_COLMOD);    // Color mode
    write_data(0x55);         // 16-bit color
    
    write_cmd(CMD_MADCTL);    // Memory data access control
    write_data(0x00);         // Normal orientation
    
    write_cmd(CMD_DISPON);    // Display on
    sleep_ms(20);
    
    // Initialize LVGL draw buffers
    lv_disp_draw_buf_init(&draw_buf, buf_1, buf_2, DISP_WIDTH * 10);
    
    // Initialize display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_WIDTH;
    disp_drv.ver_res = DISP_HEIGHT;
    disp_drv.flush_cb = display_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.rounder_cb = display_rounder;
    
    lv_disp_drv_register(&disp_drv);
}

void display_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
    set_window(area->x1, area->y1, area->x2, area->y2);
    
    write_cmd(CMD_RAMWR);
    gpio_put(PIN_DC, 1);  // Data mode
    gpio_put(PIN_CS, 0);
    
    uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);
    
    // Configure DMA transfer
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, spi_get_dreq(spi0, true));
    
    dma_channel_configure(
        dma_tx,
        &c,
        &spi_get_hw(spi0)->dr,
        color_p,
        size,
        true
    );
    
    dma_channel_wait_for_finish_blocking(dma_tx);
    gpio_put(PIN_CS, 1);
    
    lv_disp_flush_ready(disp_drv);
}

void display_rounder(lv_disp_drv_t * disp_drv, lv_area_t * area) {
    // Round coordinates to display boundaries
    area->x1 = 0;
    area->y1 = 0;
    area->x2 = DISP_WIDTH - 1;
    area->y2 = DISP_HEIGHT - 1;
}

void display_set_pixel(uint16_t x, uint16_t y, lv_color_t color) {
    if (x >= DISP_WIDTH || y >= DISP_HEIGHT) return;
    
    set_window(x, y, x, y);
    write_cmd(CMD_RAMWR);
    
    uint16_t color_value = color.full;
    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);
    spi_write_blocking(spi0, (uint8_t*)&color_value, 2);
    gpio_put(PIN_CS, 1);
}
