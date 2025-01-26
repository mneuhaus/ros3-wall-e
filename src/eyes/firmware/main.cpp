/**
 * Basic Eyes Display Controller
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "pico/bootrom.h"
#include "display.h"

#define MAX_CMD_LENGTH 128
#define BOOTLOADER_DELAY_MS 500

// Pin definitions for GC9A01A display
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_DC   4  // Data/Command control
#define PIN_CS   5  // Chip select
#define PIN_RST  6  // Reset

// DMA channel for display updates
int dma_tx;

// Basic test pattern - checkerboard
const uint16_t default_image[240 * 240] = {
    // Initialize with alternating colors
    [0 ... (240*240/2)-1] = 0xF800,   // Red
    [240*240/2 ... 240*240-1] = 0x07E0 // Green
};

void send_response(const char* cmd, const char* status, const char* message) {
    printf("%s: %s %s\n", cmd, status, message);
}

bool init_gpio(uint pin, const char* mode) {
    if (pin >= 30) {
        return false;
    }

    if (strcmp(mode, "SPI") == 0) {
        if (pin == PIN_SCK) {
            gpio_set_function(pin, GPIO_FUNC_SPI);
        } else if (pin == PIN_MOSI) {
            gpio_set_function(pin, GPIO_FUNC_SPI);
        }
        return true;
    }
    else if (strcmp(mode, "OUTPUT") == 0) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        return true;
    }
    
    return false;
}

void write_cmd(uint8_t cmd) {
    gpio_put(PIN_DC, 0);  // Command mode
    gpio_put(PIN_CS, 0);
    spi_write_blocking(spi0, &cmd, 1);
    gpio_put(PIN_CS, 1);
}

void write_data(uint8_t data) {
    gpio_put(PIN_DC, 1);  // Data mode
    gpio_put(PIN_CS, 0);
    spi_write_blocking(spi0, &data, 1);
    gpio_put(PIN_CS, 1);
}

void set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
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

void init_display() {
    // Initialize SPI at 62.5MHz
    spi_init(spi0, 62500000);
    
    // Initialize control pins
    init_gpio(PIN_SCK, "SPI");
    init_gpio(PIN_MOSI, "SPI");
    init_gpio(PIN_DC, "OUTPUT");
    init_gpio(PIN_CS, "OUTPUT");
    init_gpio(PIN_RST, "OUTPUT");
    
    // Setup DMA channel
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
}

void display_image() {
    set_window(0, 0, DISP_WIDTH-1, DISP_HEIGHT-1);
    
    write_cmd(CMD_RAMWR);
    gpio_put(PIN_DC, 1);  // Data mode
    gpio_put(PIN_CS, 0);
    
    // Send image data using DMA
    dma_channel_config c = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, spi_get_dreq(spi0, true));
    
    dma_channel_configure(
        dma_tx,
        &c,
        &spi_get_hw(spi0)->dr,
        default_image,
        DISP_WIDTH * DISP_HEIGHT,
        true
    );
    
    dma_channel_wait_for_finish_blocking(dma_tx);
    gpio_put(PIN_CS, 1);
}

bool process_command(char* command) {
    char cmd_copy[MAX_CMD_LENGTH];
    strncpy(cmd_copy, command, MAX_CMD_LENGTH);
    
    // Split command into parts
    char* saveptr;
    char* cmd = strtok_r(command, " ", &saveptr);
    
    if (strcmp(cmd, "INIT_GPIO") == 0) {
        uint pin = 0;
        char mode[10] = {0};
        
        char* arg;
        bool has_pin = false, has_mode = false;
        while (saveptr != NULL && (arg = strtok_r(NULL, " ", &saveptr))) {
            if (strncmp(arg, "PIN=", 4) == 0) {
                if (sscanf(arg + 4, "%u", &pin) != 1) {
                    send_response(cmd_copy, "ERROR", "INVALID_PIN_FORMAT");
                    return false;
                }
                has_pin = true;
            } else if (strncmp(arg, "MODE=", 5) == 0) {
                strncpy(mode, arg + 5, sizeof(mode) - 1);
                has_mode = true;
            }
        }
        
        if (!has_pin || !has_mode) {
            send_response(cmd_copy, "ERROR", "MISSING_REQUIRED_PARAMS");
            return false;
        }
        
        if (init_gpio(pin, mode)) {
            send_response(cmd_copy, "OK", "");
        } else {
            send_response(cmd_copy, "ERROR", "INIT_FAILED");
        }
        return true;
    }
    else if (strcmp(cmd, "DRAW_IMAGE") == 0) {
        uint16_t x = 0, y = 0, width = 0, height = 0;
        
        char* arg;
        bool has_pos = false, has_size = false;
        while (saveptr != NULL && (arg = strtok_r(NULL, " ", &saveptr))) {
            if (strncmp(arg, "X=", 2) == 0) {
                sscanf(arg + 2, "%hu", &x);
                has_pos = true;
            } else if (strncmp(arg, "Y=", 2) == 0) {
                sscanf(arg + 2, "%hu", &y);
            } else if (strncmp(arg, "WIDTH=", 6) == 0) {
                sscanf(arg + 6, "%hu", &width);
                has_size = true;
            } else if (strncmp(arg, "HEIGHT=", 7) == 0) {
                sscanf(arg + 7, "%hu", &height);
            }
        }
        
        if (!has_pos || !has_size) {
            send_response(cmd_copy, "ERROR", "MISSING_REQUIRED_PARAMS");
            return false;
        }
        
        // TODO: Implement actual image drawing
        send_response(cmd_copy, "OK", "");
        return true;
    }
    else if (strcmp(cmd, "RESET_FIRMWARE") == 0) {
        send_response(cmd_copy, "OK", "");
        sleep_ms(BOOTLOADER_DELAY_MS);
        reset_usb_boot(0, 0);
        return true;
    }
    else if (strcmp(cmd, "PING") == 0) {
        send_response(cmd_copy, "OK", "");
        return true;
    }
    
    send_response(cmd_copy, "ERROR", "UNKNOWN_COMMAND");
    return false;
}

int main() {
    stdio_init_all();
    
    // Initialize and show image
    init_display();
    display_image();
    
    printf("Eyes Display Ready\n");
    
    char buffer[MAX_CMD_LENGTH] = {0};
    uint8_t buf_pos = 0;
    
    while (true) {
        int c = getchar_timeout_us(1000);
        if (c == PICO_ERROR_TIMEOUT) continue;
        
        if (c == '\n' || c == '\r') {
            if (buf_pos > 0) {
                buffer[buf_pos] = '\0';
                process_command(buffer);
                buf_pos = 0;
            }
        } else if (buf_pos < MAX_CMD_LENGTH - 1) {
            buffer[buf_pos++] = c;
        }
    }
    
    return 0;
}
