import time
from plasma import WS2812
from servo import Servo, servo2040

# Create and start the LED bar
led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)
led_bar.start()

# Set first LED to green to indicate power (30% brightness)
led_bar.set_rgb(0, 0, int(255 * 0.3), 0)

# Define all servos with their ranges
eyebrow_left = Servo(servo2040.SERVO_1)    # 0-90 degrees
eyebrow_right = Servo(servo2040.SERVO_2)   # 0-90 degrees
head_left = Servo(servo2040.SERVO_3)       # 0-40 degrees
head_right = Servo(servo2040.SERVO_4)      # 0-40 degrees
neck_tilt = Servo(servo2040.SERVO_5)       # 0-90 degrees
neck_raise = Servo(servo2040.SERVO_6)      # 0-180 degrees
neck_pan = Servo(servo2040.SERVO_7)        # 0-180 degrees
arm_left = Servo(servo2040.SERVO_8)        # 0-180 degrees
arm_right = Servo(servo2040.SERVO_9)       # 0-180 degrees

# Function to convert degrees to servo value (0.0 to 1.0)
def degrees_to_value(degrees, max_degrees):
    return degrees / max_degrees

# Function to set servo angle in degrees
def set_servo_degrees(servo, degrees, max_degrees):
    servo.value(degrees_to_value(degrees, max_degrees))

# Enable all servos
servos = [eyebrow_left, eyebrow_right, head_left, head_right,
          neck_tilt, neck_raise, neck_pan, arm_left, arm_right]

try:
    # Enable all servos and center them
    for servo in servos:
        servo.enable()
        servo.to_mid()
    
    while True:
        # Example movement sequence - replace with your control logic
        time.sleep(0.1)  # Small delay to prevent busy-waiting

except KeyboardInterrupt:
    # Disable all servos on Ctrl+C
    for servo in servos:
        servo.disable()
    print("Program stopped.")
