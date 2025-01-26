/**
 * EVE Protocol-compliant Eyes Display Controller
 * Implements docs/eve-protokoll.md specification
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/bootrom.h"

// Display configuration
#define DISP_WIDTH 240
#define DISP_HEIGHT 240
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
    
    // TODO: Add GC9A01A initialization sequence
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
        while ((arg = strtok_r(NULL, " ", &saveptr))) {
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
        while ((arg = strtok_r(NULL, " ", &saveptr))) {
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
    
    printf("EVE Protocol Eyes Display Ready\n");
    
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
