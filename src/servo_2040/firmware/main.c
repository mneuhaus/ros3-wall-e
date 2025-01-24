/**
 * Servo 2040 Controller Firmware
 * C implementation of servo control with JSON command interface
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "pico/bootrom.h"
#include "hardware/uart.h"

#define NUM_SERVOS 9
#define MAX_BUFFER_SIZE 256
#define PWM_FREQ 50
#define MIN_PULSE_US 500   // 0 degrees
#define MAX_PULSE_US 2500  // 180 degrees

// Servo pins (matching servo2040.SERVO_X)
const uint8_t SERVO_PINS[NUM_SERVOS] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

typedef struct {
    uint slice_num;
    uint channel;
    float current_angle;
    float target_angle;
} servo_t;

#define EASING_STEP 1.0f  // Degrees per update

servo_t servos[NUM_SERVOS];

// Convert degrees to PWM value
uint16_t degrees_to_pwm(float degrees) {
    if (degrees < 0) degrees = 0;
    if (degrees > 180) degrees = 180;
    
    float pulse_width = MIN_PULSE_US + (degrees / 180.0) * (MAX_PULSE_US - MIN_PULSE_US);
    return (uint16_t)((pulse_width / 20000.0) * 65535);
}

void init_servo(int index) {
    uint pin = SERVO_PINS[index];
    gpio_set_function(pin, GPIO_FUNC_PWM);
    
    servos[index].slice_num = pwm_gpio_to_slice_num(pin);
    servos[index].channel = pwm_gpio_to_channel(pin);
    servos[index].current_angle = 90;
    
    pwm_set_wrap(servos[index].slice_num, 65535);
    pwm_set_clkdiv(servos[index].slice_num, 125.0); // 1MHz
    pwm_set_enabled(servos[index].slice_num, true);
    
    // Set initial position
    pwm_set_chan_level(servos[index].slice_num, servos[index].channel, degrees_to_pwm(90));
}

void set_servo_position(int index, float degrees) {
    if (index < 0 || index >= NUM_SERVOS) return;
    
    if (degrees < 0) degrees = 0;
    if (degrees > 180) degrees = 180;
    
    servos[index].target_angle = degrees;
}

void update_servo_positions(void) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        if (servos[i].current_angle != servos[i].target_angle) {
            float diff = servos[i].target_angle - servos[i].current_angle;
            float step = (fabsf(diff) < EASING_STEP) ? diff : 
                        (diff > 0 ? EASING_STEP : -EASING_STEP);
            
            servos[i].current_angle += step;
            pwm_set_chan_level(servos[i].slice_num, servos[i].channel,
                             degrees_to_pwm(servos[i].current_angle));
        }
    }
}

bool process_command(char* cmd) {
    // Simple JSON parsing (you might want to use a proper JSON parser in production)
    if (strstr(cmd, "enter_bootloader") != NULL) {
        printf("Entering bootloader mode...\n");
        sleep_ms(500);
        reset_usb_boot(0, 0);
        return true;
    }
    
    char* servo_start = strstr(cmd, "\"servos\":");
    if (servo_start) {
        char* array_start = strchr(servo_start, '[');
        if (array_start) {
            char* ptr = array_start + 1;
            while (*ptr) {
                if (*ptr == '[') {
                    int index;
                    float position;
                    if (sscanf(ptr, "[%d,%f]", &index, &position) == 2) {
                        if (index >= 0 && index < NUM_SERVOS) {
                            set_servo_position(index, position);
                            printf("Servo %d -> %.1f°\n", index, position);
                        }
                    }
                }
                ptr++;
            }
            return true;
        }
    }
    return false;
}

int main() {
    stdio_init_all();
    
    // Initialize all servos
    for (int i = 0; i < NUM_SERVOS; i++) {
        init_servo(i);
    }
    
    printf("Servo controller ready\n");
    
    char buffer[MAX_BUFFER_SIZE];
    int buf_pos = 0;
    
    while (1) {
        // Non-blocking read from USB serial
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\n' || c == '\r') {
                if (buf_pos > 0) {
                    buffer[buf_pos] = '\0';
                    process_command(buffer);
                    buf_pos = 0;
                }
            } else if (buf_pos < MAX_BUFFER_SIZE - 1) {
                buffer[buf_pos++] = c;
            }
        }
        
        // Update servo positions and add small delay
        update_servo_positions();
        sleep_ms(10);  // 100Hz update rate
    }
    
    return 0;
}
