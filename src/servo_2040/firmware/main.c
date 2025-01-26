/**
 * EVE Protocol-compliant Servo Controller Firmware
 * Implements docs/eve-protokoll.md specification
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/bootrom.h"

// Hardware configuration
#define LEFT_TRACK_PWM 13
#define LEFT_TRACK_DIR 14
#define RIGHT_TRACK_PWM 17
#define RIGHT_TRACK_DIR 18
#define MAX_CMD_LENGTH 128
#define BOOTLOADER_DELAY_MS 500

typedef struct {
    uint slice;
    uint channel;
    uint freq;
    bool initialized;
    float current_pos;
} pwm_channel;

pwm_channel pwm_channels[30] = {0}; // Support up to pin 29

void send_response(const char* cmd, const char* status, const char* message) {
    printf("%s: %s %s\n", cmd, status, message);
}

bool init_gpio(uint pin, const char* mode, uint count, uint freq) {
    if (pin >= 30) {
        return false;
    }

    if (strcmp(mode, "PWM") == 0 || strcmp(mode, "SERVO") == 0) {
        gpio_set_function(pin, GPIO_FUNC_PWM);
        pwm_channels[pin].slice = pwm_gpio_to_slice_num(pin);
        pwm_channels[pin].channel = pwm_gpio_to_channel(pin);
        pwm_channels[pin].freq = freq;
        
        // Calculate clock divider from frequency (125MHz base clock)
        float div = 125000000.0f / (65536 * freq);
        pwm_set_clkdiv(pwm_channels[pin].slice, div);
        pwm_set_wrap(pwm_channels[pin].slice, 65535);
        pwm_set_enabled(pwm_channels[pin].slice, true);
        
        pwm_channels[pin].initialized = true;
        return true;
    }
    else if (strcmp(mode, "OUTPUT") == 0) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_OUT);
        return true;
    }
    
    return false;
}

void set_servo_position(uint pin, float degrees) {
    if (!pwm_channels[pin].initialized || degrees < 0 || degrees > 180) {
        return;
    }

    // Convert degrees to pulse width (500-2500µs)
    uint16_t level = (uint16_t)(500 + (degrees / 180.0f) * 2000) * 65535 / 20000;
    pwm_set_chan_level(pwm_channels[pin].slice, pwm_channels[pin].channel, level);
    pwm_channels[pin].current_pos = degrees;
}

void set_track_speed(uint pwm_pin, uint dir_pin, int speed) {
    if (!pwm_channels[pwm_pin].initialized) return;

    speed = (speed < -100) ? -100 : (speed > 100) ? 100 : speed;
    
    // Set direction
    gpio_put(dir_pin, speed >= 0);
    
    // Set PWM value
    uint16_t level = (abs(speed) * 65535) / 100;
    pwm_set_chan_level(pwm_config[pwm_pin].slice, 
                      pwm_config[pwm_pin].channel, 
                      level);
}

bool process_command(char* command) {
    char cmd_copy[MAX_CMD_LENGTH];
    strncpy(cmd_copy, command, MAX_CMD_LENGTH);
    
    // Split command into parts
    char* saveptr;
    char* cmd = strtok_r(command, " ", &saveptr);
    
    if (strcmp(cmd, "INIT_GPIO") == 0) {
        uint pin, count = 0, freq = 0;
        char mode[10] = {0};
        
        char* arg;
        while ((arg = strtok_r(NULL, " ", &saveptr))) {
            if (sscanf(arg, "PIN=%u", &pin)) continue;
            if (sscanf(arg, "MODE=%9s", mode)) continue;
            sscanf(arg, "COUNT=%u", &count);
            sscanf(arg, "FREQ=%u", &freq);
        }
        
        if (init_gpio(pin, mode, count, freq)) {
            send_response(cmd_copy, "OK", "");
        } else {
            send_response(cmd_copy, "ERROR", "INIT_FAILED");
        }
        return true;
    }
    else if (strcmp(cmd, "MOVE_SERVO") == 0) {
        uint pin;
        float pos;
        int speed = 0;
        
        char* arg;
        while ((arg = strtok_r(NULL, " ", &saveptr))) {
            sscanf(arg, "PIN=%u", &pin);
            sscanf(arg, "POS=%f", &pos);
            sscanf(arg, "SPEED=%d", &speed);
        }
        
        if (pin < 30 && pwm_channels[pin].initialized) {
            set_servo_position(pin, pos);
            send_response(cmd_copy, "OK", "");
        } else {
            send_response(cmd_copy, "ERROR", "INVALID_PIN");
        }
        return true;
    }
    else if (strcmp(cmd, "SET_GPIO") == 0) {
        // Handle track controls as GPIO extensions
        uint pin, pwm_val;
        char state[5] = {0};
        
        char* arg;
        while ((arg = strtok_r(NULL, " ", &saveptr))) {
            if (sscanf(arg, "PIN=%u", &pin)) continue;
            if (sscanf(arg, "STATE=%4s", state)) continue;
            if (sscanf(arg, "PWM=%u", &pwm_val)) continue;
        }
        
        if (pin == LEFT_TRACK_PWM || pin == RIGHT_TRACK_PWM) {
            int speed = (pin == LEFT_TRACK_PWM) ? 
                       (pwm_val * 100 / 65535) : 
                       -(pwm_val * 100 / 65535);
            set_track_speed(pin, 
                           (pin == LEFT_TRACK_PWM) ? LEFT_TRACK_DIR : RIGHT_TRACK_DIR,
                           speed);
            send_response(cmd_copy, "OK", "");
        } else if (strlen(state) > 0) {
            gpio_put(pin, strcmp(state, "HIGH") == 0);
            send_response(cmd_copy, "OK", "");
        } else {
            send_response(cmd_copy, "ERROR", "INVALID_PIN");
        }
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
    
    // Initialize track controls with default 1kHz PWM
    init_gpio(LEFT_TRACK_PWM, "PWM", 0, 1000);
    init_gpio(RIGHT_TRACK_PWM, "PWM", 0, 1000);
    gpio_init(LEFT_TRACK_DIR);
    gpio_init(RIGHT_TRACK_DIR);
    gpio_set_dir(LEFT_TRACK_DIR, GPIO_OUT);
    gpio_set_dir(RIGHT_TRACK_DIR, GPIO_OUT);
    
    printf("EVE Protocol Controller Ready\n");
    
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
