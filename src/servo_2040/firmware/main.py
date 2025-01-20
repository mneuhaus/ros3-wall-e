import time
import json
import machine
import math
from plasma import WS2812
from servo_pio import ServoController

# Set up UART for debug output
uart = machine.UART(0, baudrate=115200)
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=machine.Pin(0), rx=machine.Pin(1))

# Create and start the LED bar
uart.write(b"Starting up...\n")
NUM_LEDS = 10
LED_DATA = 18
led_bar = WS2812(NUM_LEDS, 1, 0, LED_DATA)
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

# Initialize servo controller (starting from pin 0 for servos)
servo_controller = ServoController(pin_base=0, num_servos=9)

# Define servo configurations
servos = {
    'eyebrow_left': {'index': 0, 'max': 90},    # 0-90 degrees
    'eyebrow_right': {'index': 1, 'max': 90},   # 0-90 degrees
    'head_left': {'index': 2, 'max': 40},       # 0-40 degrees
    'head_right': {'index': 3, 'max': 40},      # 0-40 degrees
    'neck_tilt': {'index': 4, 'max': 90},       # 0-90 degrees
    'neck_raise': {'index': 5, 'max': 180},     # 0-180 degrees
    'neck_pan': {'index': 6, 'max': 180},       # 0-180 degrees
    'arm_left': {'index': 7, 'max': 180},       # 0-180 degrees
    'arm_right': {'index': 8, 'max': 180}       # 0-180 degrees
}

# Center all servos
for name, config in servos.items():
    servo_controller.set_servo(config['index'], config['max'] / 2)

try:
    while True:
        # Read command from serial
        if uart.any():  # Check if there's data available
            try:
                input_data = uart.readline()
                if input_data:
                    uart.write(b"Received data: ")
                    uart.write(input_data)  # Echo back what we received
                    input_str = input_data.decode('utf-8').strip()
                    command = json.loads(input_str)
                    if 'servos' in command:
                        uart.write(b"Processing servo command\n")
                        for name, degrees in command['servos'].items():
                            if name in servos:
                                # Clamp degrees to servo's max range
                                degrees = min(degrees, servos[name]['max'])
                                # Update servo position
                                servo_controller.set_servo(servos[name]['index'], degrees)
                                
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
                uart.write(b"Error: ")
                uart.write(str(e).encode())
                uart.write(b"\n")
        
        time.sleep(0.01)  # Small delay to prevent busy-waiting

except KeyboardInterrupt:
    # Disable all servos on Ctrl+C
    servo_controller.disable_all()
    uart.write(b"Program stopped.\n")
