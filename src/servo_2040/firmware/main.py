import time
import json
import machine
import math
from plasma import WS2812
from servo import Servo, servo2040

# Set up UART for debug output with larger TX/RX buffers
uart = machine.UART(0, baudrate=115200, tx_buffer_size=2048, rx_buffer_size=2048)

# Create and start the LED bar
led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)
led_bar.start()

def degree_to_rgb(degree, max_degree):
    """Convert a degree value to a rainbow color (R,G,B)."""
    # Convert degree to hue (0-360)
    hue = (degree / max_degree) * 360
    
    # Convert hue to RGB
    h = hue / 60
    c = 255
    x = int(c * (1 - abs((h % 2) - 1)))
    
    if h < 1:
        return (c, x, 0)
    elif h < 2:
        return (x, c, 0)
    elif h < 3:
        return (0, c, x)
    elif h < 4:
        return (0, x, c)
    elif h < 5:
        return (x, 0, c)
    else:
        return (c, 0, x)

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
        try:
            # Read command from serial
            if uart.any():  # Check if there's data available
                input_data = uart.readline().decode('utf-8').strip()
                if input_data:
                    command = json.loads(input_data)
                    if 'servos' in command:
                        for name, degrees in command['servos'].items():
                            if name in servos:
                                # Clamp degrees to servo's max range
                                degrees = min(degrees, servos[name]['max'])
                                # Update servo position
                                servos[name]['servo'].value(degrees)
                                
                                # Update corresponding LED with rainbow color
                                # Map servo names to LED indices (skipping LED 0 which shows power)
                                led_map = {
                                'eyebrow_left': 1,
                                'eyebrow_right': 2,
                                'head_left': 3,
                                'head_right': 4,
                                'neck_tilt': 5,
                                'neck_raise': 6,
                                'neck_pan': 7,
                                'arm_left': 8,
                                'arm_right': 9
                            }
                            if name in led_map:
                                r, g, b = degree_to_rgb(degrees, servos[name]['max'])
                                led_bar.set_rgb(led_map[name], r, g, b)
                            
        except Exception as e:
            uart.write(f"Error: {str(e)}\n")
            
        time.sleep(0.01)  # Small delay to prevent busy-waiting

except KeyboardInterrupt:
    # Disable all servos on Ctrl+C
    for servo_info in servos.values():
        servo_info['servo'].disable()
    uart.write("Program stopped.\n")
