/**
 * EVE Protocol-compliant Servo Controller Firmware
 * Implements docs/eve-protokoll.md specification
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "pico/bootrom.h"

// PWM configuration for servos
struct PWMChannel {
    bool initialized;
    uint slice_num;
    uint channel;
    float min_us;
    float max_us;
};

PWMChannel pwm_channels[30] = {0};

// Hardware configuration
#define LEFT_TRACK_PWM 13
#define LEFT_TRACK_DIR 14
#define RIGHT_TRACK_PWM 17
#define RIGHT_TRACK_DIR 18
#define MAX_CMD_LENGTH 128
#define BOOTLOADER_DELAY_MS 500

// Create a servo cluster for all possible servos, using PIO 0 and State Machine 0
servo::ServoCluster* servos = nullptr;

void send_response(const char* cmd, const char* status, const char* message) {
    printf("%s: %s %s\n", cmd, status, message);
}

bool init_gpio(uint pin, const char* mode, uint count, uint freq) {
    if (pin >= 30) {
        return false;
    }

    if (strcmp(mode, "PWM") == 0 || strcmp(mode, "SERVO") == 0) {
        servos->init();
        servos->enable(pin);
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
    if (degrees < 0 || degrees > 180) {
        return;
    }

    servos->to_value(pin, degrees);
}

void set_track_speed(uint pwm_pin, uint dir_pin, int speed) {
    speed = (speed < -100) ? -100 : (speed > 100) ? 100 : speed;
    
    // Set direction
    gpio_put(dir_pin, speed >= 0);
    
    // Set PWM value
    servos->to_percent(pwm_pin, abs(speed), 0, 100);
}

bool process_command(char* command) {
    char cmd_copy[MAX_CMD_LENGTH];
    strncpy(cmd_copy, command, MAX_CMD_LENGTH);
    
    // Split command into parts
    char* saveptr;
    char* cmd = strtok_r(command, " ", &saveptr);
    
    if (strcmp(cmd, "INIT_GPIO") == 0) {
        uint pin = 0, count = 0, freq = 0;
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
            } else if (strncmp(arg, "COUNT=", 6) == 0) {
                if (sscanf(arg + 6, "%u", &count) != 1) {
                    send_response(cmd_copy, "ERROR", "INVALID_COUNT_FORMAT");
                    return false;
                }
            } else if (strncmp(arg, "FREQ=", 5) == 0) {
                if (sscanf(arg + 5, "%u", &freq) != 1) {
                    send_response(cmd_copy, "ERROR", "INVALID_FREQ_FORMAT");
                    return false;
                }
            }
        }
        
        if (!has_pin || !has_mode) {
            send_response(cmd_copy, "ERROR", "MISSING_REQUIRED_PARAMS");
            return false;
        }
        
        if (init_gpio(pin, mode, count, freq)) {
            send_response(cmd_copy, "OK", "");
        } else {
            send_response(cmd_copy, "ERROR", "INIT_FAILED");
        }
        return true;
    }
    else if (strcmp(cmd, "MOVE_SERVO") == 0) {
        uint pin = 0;
        float pos = 0.0f;
        int speed = 0;
        
        char* arg;
        bool has_pin = false, has_pos = false;
        while ((arg = strtok_r(NULL, " ", &saveptr))) {
            if (strncmp(arg, "PIN=", 4) == 0) {
                if (sscanf(arg + 4, "%u", &pin) != 1) {
                    send_response(cmd_copy, "ERROR", "INVALID_PIN_FORMAT");
                    return false;
                }
                has_pin = true;
            } else if (strncmp(arg, "POS=", 4) == 0) {
                if (sscanf(arg + 4, "%f", &pos) != 1) {
                    send_response(cmd_copy, "ERROR", "INVALID_POS_FORMAT");
                    return false;
                }
                has_pos = true;
            } else if (strncmp(arg, "SPEED=", 6) == 0) {
                if (sscanf(arg + 6, "%d", &speed) != 1) {
                    send_response(cmd_copy, "ERROR", "INVALID_SPEED_FORMAT");
                    return false;
                }
            }
        }
        
        if (!has_pin || !has_pos) {
            send_response(cmd_copy, "ERROR", "MISSING_REQUIRED_PARAMS");
            return false;
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
        uint pin = 0, pwm_val = 0;
        char state[5] = {0};
        
        char* arg;
        bool has_pin = false, has_state = false, has_pwm = false;
        while ((arg = strtok_r(NULL, " ", &saveptr))) {
            if (strncmp(arg, "PIN=", 4) == 0) {
                if (sscanf(arg + 4, "%u", &pin) != 1) {
                    send_response(cmd_copy, "ERROR", "INVALID_PIN_FORMAT");
                    return false;
                }
                has_pin = true;
            } else if (strncmp(arg, "STATE=", 6) == 0) {
                strncpy(state, arg + 6, sizeof(state) - 1);
                has_state = true;
            } else if (strncmp(arg, "PWM=", 4) == 0) {
                if (sscanf(arg + 4, "%u", &pwm_val) != 1) {
                    send_response(cmd_copy, "ERROR", "INVALID_PWM_FORMAT");
                    return false;
                }
                has_pwm = true;
            }
        }
        
        if (!has_pin || (!has_state && !has_pwm)) {
            send_response(cmd_copy, "ERROR", "MISSING_REQUIRED_PARAMS");
            return false;
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
    
    // Initialize servo cluster
    servos = new servo::ServoCluster(pio0, 0, 0, 30);
    
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
