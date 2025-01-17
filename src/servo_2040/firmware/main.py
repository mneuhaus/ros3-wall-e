import time
from pimoroni import Button
from plasma import WS2812
from servo import servo2040
from machine import Pin, PWM, UART, Timer
import time

# """
# Displays a rotating rainbow pattern on the Servo 2040's onboard LED bar.

# Press "Boot" to exit the program.

# NOTE: Plasma WS2812 uses the RP2040's PIO system, and as
# such may have problems when running code multiple times.
# If you encounter issues, try resetting your board.
# """

# SPEED = 5           # The speed that the LEDs will cycle at
# BRIGHTNESS = 0.4    # The brightness of the LEDs
# UPDATES = 50        # How many times the LEDs will be updated per second

# # Create the LED bar, using PIO 1 and State Machine 0
# led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)

# # Create the user button
# user_sw = Button(servo2040.USER_SW)

# # Start updating the LED bar
# led_bar.start()


# offset = 0.0

# # Make rainbows until the user button is pressed
# while not user_sw.raw():

#     offset += SPEED / 1000.0

#     # Update all the LEDs
#     for i in range(servo2040.NUM_LEDS):
#         hue = float(i) / servo2040.NUM_LEDS
#         led_bar.set_hsv(i, hue + offset, 1.0, BRIGHTNESS)

#     time.sleep(1.0 / UPDATES)

# # Turn off the LED bar
# led_bar.clear()

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
        time.sleep(0.01)  # Small delay to prevent high CPU usage
except KeyboardInterrupt:
    # Clean up on Ctrl+C
    pwm_left.deinit()
    pwm_right.deinit()
    # enable_left.value(0)
    # enable_right.value(0)
    print("Program stopped.")
