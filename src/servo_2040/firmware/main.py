import time
import json
import sys
from plasma import WS2812
from servo import Servo, servo2040

# Create and start the LED bar
led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)
led_bar.start()

# Set first LED to green to indicate power (30% brightness)
led_bar.set_rgb(0, 0, int(255 * 0.3), 0)

# Define all servos with their ranges
servos = {
    'eyebrow_left': {'servo': Servo(servo2040.SERVO_1), 'max': 90},    # 0-90 degrees
    'eyebrow_right': {'servo': Servo(servo2040.SERVO_2), 'max': 90},   # 0-90 degrees
    'head_left': {'servo': Servo(servo2040.SERVO_3), 'max': 40},       # 0-40 degrees
    'head_right': {'servo': Servo(servo2040.SERVO_4), 'max': 40},      # 0-40 degrees
    'neck_tilt': {'servo': Servo(servo2040.SERVO_5), 'max': 90},       # 0-90 degrees
    'neck_raise': {'servo': Servo(servo2040.SERVO_6), 'max': 180},     # 0-180 degrees
    'neck_pan': {'servo': Servo(servo2040.SERVO_7), 'max': 180},       # 0-180 degrees
    'arm_left': {'servo': Servo(servo2040.SERVO_8), 'max': 180},       # 0-180 degrees
    'arm_right': {'servo': Servo(servo2040.SERVO_9), 'max': 180}       # 0-180 degrees
}

# Enable all servos and center them
for servo_info in servos.values():
    servo_info['servo'].enable()
    servo_info['servo'].to_mid()

try:
    while True:
        sys.stdout.write('woot\n')
        sys.stdout.flush()
        try:
            # Read command from serial
            if input_data := input():
                command = json.loads(input_data)
                if 'servos' in command:
                    for name, degrees in command['servos'].items():
                        if name in servos:
                            # Clamp degrees to servo's max range
                            degrees = min(degrees, servos[name]['max'])
                            # Servo.value() already expects degrees
                            servos[name]['servo'].value(degrees)
                            
        except Exception as e:
            sys.stdout.write(f"Error: {str(e)}\n")
            sys.stdout.flush()
            
        time.sleep(0.01)  # Small delay to prevent busy-waiting

except KeyboardInterrupt:
    # Disable all servos on Ctrl+C
    for servo_info in servos.values():
        servo_info['servo'].disable()
    sys.stdout.write("Program stopped.\n")
    sys.stdout.flush()
