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

// Track control pins
#define LEFT_TRACK_PWM 15
#define LEFT_TRACK_DIR 16
#define RIGHT_TRACK_PWM 17
#define RIGHT_TRACK_DIR 18

// Servo pins (matching servo2040.SERVO_X)
const uint8_t SERVO_PINS[NUM_SERVOS] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

typedef struct {
    uint slice_num;
    uint channel;
    float current_angle;
} servo_t;


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

void init_tracks(void) {
    // Setup PWM pins
    gpio_set_function(LEFT_TRACK_PWM, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_TRACK_PWM, GPIO_FUNC_PWM);
    
    // Setup direction pins as regular GPIO
    gpio_init(LEFT_TRACK_DIR);
    gpio_init(RIGHT_TRACK_DIR);
    gpio_set_dir(LEFT_TRACK_DIR, GPIO_OUT);
    gpio_set_dir(RIGHT_TRACK_DIR, GPIO_OUT);
    
    // Configure PWM for tracks (20kHz frequency)
    uint left_slice = pwm_gpio_to_slice_num(LEFT_TRACK_PWM);
    uint right_slice = pwm_gpio_to_slice_num(RIGHT_TRACK_PWM);
    
    pwm_set_wrap(left_slice, 65535);
    pwm_set_wrap(right_slice, 65535);
    pwm_set_clkdiv(left_slice, 62.5); // 20kHz @ 125MHz
    pwm_set_clkdiv(right_slice, 62.5);
    
    pwm_set_enabled(left_slice, true);
    pwm_set_enabled(right_slice, true);
    
    // Set initial state (stopped)
    gpio_put(LEFT_TRACK_DIR, 0);
    gpio_put(RIGHT_TRACK_DIR, 0);
    pwm_set_chan_level(left_slice, pwm_gpio_to_channel(LEFT_TRACK_PWM), 0);
    pwm_set_chan_level(right_slice, pwm_gpio_to_channel(RIGHT_TRACK_PWM), 0);
}

void set_track_speed(uint pin_pwm, uint pin_dir, int speed) {
    // speed: -100 to +100
    bool direction = speed >= 0;
    uint16_t pwm_value = (uint16_t)(abs(speed) * 655.35); // Scale -100,100 to 0,65535
    
    // Invert direction for right track
    bool actual_direction = (pin_dir == RIGHT_TRACK_DIR) ? direction : !direction;
    gpio_put(pin_dir, actual_direction);
    pwm_set_chan_level(pwm_gpio_to_slice_num(pin_pwm), 
                       pwm_gpio_to_channel(pin_pwm), 
                       pwm_value);
}

void set_servo_position(int index, float degrees) {
    if (index < 0 || index >= NUM_SERVOS) return;
    
    if (degrees < 0) degrees = 0;
    if (degrees > 180) degrees = 180;
    
    servos[index].current_angle = degrees;
    pwm_set_chan_level(servos[index].slice_num, servos[index].channel,
                      degrees_to_pwm(degrees));
}


bool process_command(char* cmd) {
    // Simple JSON parsing (you might want to use a proper JSON parser in production)
    if (strstr(cmd, "enter_bootloader") != NULL) {
        printf("Entering bootloader mode...\n");
        sleep_ms(500);
        reset_usb_boot(0, 0);
        return true;
    }
    
    // Handle track commands
    if (strstr(cmd, "\"tracks\":") != NULL) {
        int left_speed, right_speed;
        if (sscanf(cmd, "{\"tracks\":[%d,%d]}", &left_speed, &right_speed) == 2) {
            set_track_speed(LEFT_TRACK_PWM, LEFT_TRACK_DIR, left_speed);
            set_track_speed(RIGHT_TRACK_PWM, RIGHT_TRACK_DIR, right_speed);
            printf("Tracks L:%d R:%d\n", left_speed, right_speed);
            return true;
        }
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
    
    // Initialize tracks
    init_tracks();
    
    printf("Servo and track controller ready\n");
    
    char buffer[MAX_BUFFER_SIZE];
    int buf_pos = 0;
    
    while (1) {
        // Process up to 32 characters per iteration
        for (int i = 0; i < 32; i++) {
            int c = getchar_timeout_us(0);
            if (c == PICO_ERROR_TIMEOUT) break;
            
            if (c == '\n' || c == '\r') {
                if (buf_pos > 0) {
                    buffer[buf_pos] = '\0';
                    process_command(buffer);
                    buf_pos = 0;
                    break;  // Process command immediately
                }
            } else if (buf_pos < MAX_BUFFER_SIZE - 1) {
                buffer[buf_pos++] = c;
            }
        }
        
        // Minimal delay only if no data was processed
        if (buf_pos == 0) {
            sleep_us(100);  // 100µs delay instead of 1ms
        }
    }
    
    return 0;
}
