from plasma import WS2812
from machine import Pin, UART
import random
import time
import json
import array
from rp2 import PIO, StateMachine
from servo import ServoController

# Set up UART for debug output
uart = UART(0, baudrate=115200)
uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=Pin(0), rx=Pin(1))

# Initialize single neopixel for power indicator
NUM_LEDS = 1
LED_DATA = 18
led = WS2812(NUM_LEDS, 1, 0, LED_DATA)
led.start()

# Set random color at 40% brightness
brightness = 0.4
r = random.randint(0, 255)
g = random.randint(0, 255)
b = random.randint(0, 255)
led.set_rgb(0, int(r * brightness), int(g * brightness), int(b * brightness))

# Initialize servo controller
servo_controller = ServoController(pin_base=19, num_servos=9)

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

# Test each servo with a 10 degree movement
uart.write(b"Starting servo test pattern...\n")
for name, config in servos.items():
    uart.write(f"Testing {name}\n".encode())
    center = config['max'] / 2
    
    # Move to center
    servo_controller.set_servo(config['index'], center)
    servo_controller.update()
    time.sleep(0.5)
    
    # Move +/- 5 degrees from center
    for pos in [center + 5, center - 5, center]:
        servo_controller.set_servo(config['index'], pos)
        servo_controller.update()
        time.sleep(0.2)

uart.write(b"Test pattern complete.\n")

# Disable all servos
servo_controller.disable_all()
def servo_program():
    """PIO program for generating servo PWM signals."""
    wrap_target()  # noqa: F821
    pull(noblock)            .side(0)    # Pull from FIFO to OSR if available  # noqa: F821
    mov(x, osr)                          # Copy OSR to X scratch register  # noqa: F821
    mov(y, isr)                          # Initialize Y with period count  # noqa: F821
    label("loop")                        # noqa: F821
    jmp(not_x, "skip")                   # Skip if X == 0 (pulse not active)  # noqa: F821
    nop()                    .side(1)    # Set output high  # noqa: F821
    label("skip")                        # noqa: F821
    jmp(y_dec, "loop")                   # Decrement Y, continue if not zero  # noqa: F821
    wrap()                               # noqa: F821
    
class ServoController:
    def __init__(self, pin_base, num_servos):
        self.num_servos = num_servos
        self.positions = array.array('H', [1500] * num_servos)  # Default 1500µs
        self.current_group = 0
        self.update_interval = 10  # ms between group switches - faster updates for smoother motion
        self.last_update = time.ticks_ms()
        
        # Split servos into two groups
        self.group_size = (num_servos + 1) // 2
        
        # Configure two state machines
        self.state_machines = []
        for i in range(2):  # Only use 2 PIOs
            sm = StateMachine(
                i,
                servo_program,
                freq=2000000,  # 2MHz
                sideset_base=Pin(pin_base + i)
            )
            sm.active(1)
            self.state_machines.append(sm)
        
        # Initialize all servo pins as outputs (they'll be multiplexed)
        self.servo_pins = []
        for i in range(num_servos):
            pin = Pin(pin_base + i, Pin.OUT)
            pin.value(0)  # Start disabled
            self.servo_pins.append(pin)
    
    def update(self):
        """Update servo outputs, switching between groups."""
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_update) >= self.update_interval:
            # Disable current group
            start_idx = self.current_group * self.group_size
            end_idx = min(start_idx + self.group_size, self.num_servos)
            for i in range(start_idx, end_idx):
                self.servo_pins[i].value(0)
            
            # Switch to next group
            self.current_group = (self.current_group + 1) % 2
            
            # Enable and update new group
            start_idx = self.current_group * self.group_size
            end_idx = min(start_idx + self.group_size, self.num_servos)
            for i in range(start_idx, end_idx):
                sm_idx = i % 2  # Use PIO 0 or 1
                # Convert pulse width to PIO cycles
                cycles = self.positions[i] * 2
                self.state_machines[sm_idx].put(cycles)
                self.servo_pins[i].value(1)
            
            self.last_update = current_time
    
    def set_servo(self, index, degrees):
        """Set servo position in degrees (0-180)."""
        if 0 <= index < self.num_servos:
            # Convert degrees to pulse width (500-2500µs)
            pulse_width = int(500 + (degrees * 2000 / 180))
            self.positions[index] = pulse_width
    
    def disable_all(self):
        """Disable all servo outputs."""
        for sm in self.state_machines:
            sm.active(0)
        for pin in self.servo_pins:
            pin.value(0)

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

# Set first LED to random color to indicate power (30% brightness)
r = random.randint(0, 255)
g = random.randint(0, 255)
b = random.randint(0, 255)
brightness = 0.3
led_bar.set_rgb(0, int(r * brightness), int(g * brightness), int(b * brightness))

# Initialize servo controller (starting from pin 19 for servos to avoid conflicts)
servo_controller = ServoController(pin_base=19, num_servos=9)

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

# Center all servos and do a small movement pattern
for name, config in servos.items():
    center = config['max'] / 2
    servo_controller.set_servo(config['index'], center)

# Small movement pattern on startup
def startup_pattern():
    """Move servos in a small pattern to show they're working."""
    uart.write(b"Running startup pattern...\n")
    for _ in range(3):  # Repeat 3 times
        for name, config in servos.items():
            center = config['max'] / 2
            # Move +/- 5 degrees from center
            for pos in [center + 5, center - 5, center]:
                servo_controller.set_servo(config['index'], pos)
                # Update LEDs
                if name in led_map:
                    r, g, b = degree_to_rgb(pos, config['max'])
                    led_bar.set_rgb(led_map[name], r, g, b)
                time.sleep(0.05)
                servo_controller.update()

startup_pattern()
uart.write(b"Startup pattern complete.\n")

try:
    while True:
        # Read command from serial
        if uart.any():  # Check if there's data available
            try:
                input_data = uart.readline()
                if input_data:
                    uart.write(b"Received data: ")
                    uart.write(input_data)  # Echo back what we received
                    try:
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
                                if name in led_map:
                                    r, g, b = degree_to_rgb(degrees, servos[name]['max'])
                                    led_bar.set_rgb(led_map[name], r, g, b)
                    except json.JSONDecodeError as e:
                        uart.write(b"JSON decode error: ")
                        uart.write(str(e).encode())
                        uart.write(b"\n")
                    except UnicodeDecodeError as e:
                        uart.write(b"Unicode decode error: ")
                        uart.write(str(e).encode())
                        uart.write(b"\n")
            except Exception as e:
                uart.write(b"Unexpected error: ")
                uart.write(str(e).encode())
                uart.write(b"\n")
        
        # Update servo multiplexing
        servo_controller.update()
        # No explicit sleep needed - the servo update and UART handling provide natural timing

except KeyboardInterrupt:
    # Disable all servos on Ctrl+C
    servo_controller.disable_all()
    uart.write(b"Program stopped.\n")
