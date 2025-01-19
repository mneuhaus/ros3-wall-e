import time
import math
from pimoroni import Button
from plasma import WS2812
from servo import servo2040
from machine import Pin, PWM, UART, Timer

# Create and start the LED bar
led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)
led_bar.start()

# Variables for rainbow effect
HUE_START = 0
HUE_END = 360
hue_offset = 0

def hsv_to_rgb(h, s, v):
    """Convert HSV color values to RGB."""
    h = float(h)
    s = float(s)
    v = float(v)
    h60 = h / 60.0
    h60f = math.floor(h60)
    hi = int(h60f) % 6
    f = h60 - h60f
    p = v * (1 - s)
    q = v * (1 - f * s)
    t = v * (1 - (1 - f) * s)
    
    if hi == 0: r, g, b = v, t, p
    elif hi == 1: r, g, b = q, v, p
    elif hi == 2: r, g, b = p, v, t
    elif hi == 3: r, g, b = p, q, v
    elif hi == 4: r, g, b = t, p, v
    else: r, g, b = v, p, q
    
    return int(r * 255), int(g * 255), int(b * 255)

def update_rainbow():
    """Update the LED bar with rainbow colors."""
    global hue_offset
    for i in range(servo2040.NUM_LEDS):
        # Calculate hue for this LED
        hue = (HUE_START + (i * 360 / servo2040.NUM_LEDS) + hue_offset) % 360
        r, g, b = hsv_to_rgb(hue, 1.0, 0.5)  # Full saturation, half brightness
        led_bar.set_rgb(i, r, g, b)
    
    # Update the offset for next time
    hue_offset = (hue_offset + 5) % 360  # Speed of color cycling

# Initialize UART on UART0 (GP0 and GP1)
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

#white: pwm
#yellow: dir
# Initialize Left Motor Pins
# enable_left = Pin(11, Pin.OUT)
pwm_left_pin = Pin(17)
dir_left = Pin(16, Pin.OUT)

# Initialize Right Motor Pins
# enable_right = Pin(7, Pin.OUT)
pwm_right_pin = Pin(15)
dir_right = Pin(14, Pin.OUT)

# Configure PWM for Left Motor
pwm_left = PWM(pwm_left_pin)
pwm_left.freq(1000)  # Set frequency to 1 kHz

# Configure PWM for Right Motor
pwm_right = PWM(pwm_right_pin)
pwm_right.freq(1000)  # Set frequency to 1 kHz

pwm_left.deinit()
pwm_right.deinit()
#enable_left.value(0)
#enable_right.value(0)

pwm_left.duty_u16(0)
pwm_right.duty_u16(0)

def set_motor(pwm, dir_pin, speed):
    """
    Set motor speed and direction.
    :param enable_pin: Pin object for the enable pin
    :param pwm: PWM object for the PWM pin
    :param dir_pin: Pin object for the direction pin
    :param speed: Speed value between -100 and 100
    """
    if speed == 0:
        # enable_pin.value(0)
        pwm.duty_u16(0)
    else:
        # enable_pin.value(1)
        if speed > 0:
            dir_pin.value(0)
        else:
            dir_pin.value(1)
            speed = -speed
        duty = int((speed / 100) * 65535)
        pwm.duty_u16(duty)

# Main loop
try:
    while True:
        if uart.any():
            data = uart.readline()
            print(data)
            if data:
                try:
                    command = data.decode('utf-8').strip()
                    print(command)
                    left_speed_str, right_speed_str = command.split(',')
                    left_speed = int(left_speed_str)
                    right_speed = int(right_speed_str)
                    # Set motors
                    set_motor(pwm_left, dir_left, left_speed)
                    set_motor(pwm_right, dir_right, right_speed)
                except Exception as e:
                    print("Error parsing command:", e)
        update_rainbow()  # Update the rainbow effect
        time.sleep(0.02)  # Small delay to prevent high CPU usage
except KeyboardInterrupt:
    # Clean up on Ctrl+C
    pwm_left.deinit()
    pwm_right.deinit()
    # enable_left.value(0)
    # enable_right.value(0)
    print("Program stopped.")
