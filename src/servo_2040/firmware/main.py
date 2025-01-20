from plasma import WS2812
from machine import Pin, UART
import random
import time
import json
import array
from rp2 import PIO, StateMachine, asm_pio

class LEDController:
    def __init__(self, num_leds, led_pin):
        self.num_leds = num_leds
        self.strip = WS2812(num_leds, 1, 0, led_pin)
        self.strip.start()
    
    def set_random_color(self, index, brightness=0.4):
        r = random.randint(0, 255)
        g = random.randint(0, 255)
        b = random.randint(0, 255)
        self.strip.set_rgb(index, int(r * brightness), int(g * brightness), int(b * brightness))
    
    def set_rgb(self, index, r, g, b):
        self.strip.set_rgb(index, r, g, b)
    
    def degree_to_rgb(self, degree, max_degree):
        """Convert a degree value to a rainbow color (R,G,B)."""
        hue = (degree / max_degree) * 360
        h = hue / 60
        c = 255
        x = int(c * (1 - abs((h % 2) - 1)))
        
        if h < 1: return (c, x, 0)
        elif h < 2: return (x, c, 0)
        elif h < 3: return (0, c, x)
        elif h < 4: return (0, x, c)
        elif h < 5: return (x, 0, c)
        else: return (c, 0, x)

@asm_pio(sideset_init=PIO.OUT_LOW)
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
        self.update_interval = 10  # ms between group switches
        self.last_update = time.ticks_ms()
        
        # Use a single state machine for all servos
        self.sm = StateMachine(
            0,  # Use PIO 0
            servo_program,
            freq=2000000,  # 2MHz
            sideset_base=Pin(pin_base)
        )
        self.sm.active(1)
        
        self.servo_pins = []
        for i in range(num_servos):
            pin = Pin(pin_base + i, Pin.OUT)
            pin.value(0)
            self.servo_pins.append(pin)
    
    def update(self):
        current_time = time.ticks_ms()
        if time.ticks_diff(current_time, self.last_update) >= self.update_interval:
            # Update all servos sequentially
            for i in range(self.num_servos):
                # Disable all other servos
                for j in range(self.num_servos):
                    self.servo_pins[j].value(0)
                
                # Enable current servo and update its position
                cycles = self.positions[i] * 2
                self.sm.put(cycles)
                self.servo_pins[i].value(1)
                time.sleep_ms(2)  # Short delay for pulse to complete
            
            self.last_update = current_time
    
    def set_servo(self, index, degrees):
        if 0 <= index < self.num_servos:
            pulse_width = int(500 + (degrees * 2000 / 180))
            self.positions[index] = pulse_width
    
    def disable_all(self):
        self.sm.active(0)
        for pin in self.servo_pins:
            pin.value(0)

class RobotController:
    def __init__(self):
        # Set up UART for debug output
        self.uart = UART(0, baudrate=115200)
        self.uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=Pin(0), rx=Pin(1))
        self.uart.write(b"Initializing robot controller...\n")
        
        # Initialize LED controller
        self.leds = LEDController(10, 18)  # 10 LEDs on pin 18
        self.leds.set_random_color(0, brightness=0.3)  # Power indicator LED
        
        # Initialize servo controller
        self.servo = ServoController(pin_base=19, num_servos=9)
        
        # Define servo configurations
        self.servo_config = {
            'eyebrow_left': {'index': 0, 'max': 90},
            'eyebrow_right': {'index': 1, 'max': 90},
            'head_left': {'index': 2, 'max': 40},
            'head_right': {'index': 3, 'max': 40},
            'neck_tilt': {'index': 4, 'max': 90},
            'neck_raise': {'index': 5, 'max': 180},
            'neck_pan': {'index': 6, 'max': 180},
            'arm_left': {'index': 7, 'max': 180},
            'arm_right': {'index': 8, 'max': 180}
        }
        
        # Map servo names to LED indices
        self.led_map = {
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
    
    def startup_pattern(self):
        self.uart.write(b"Running startup pattern...\n")
        for name, config in self.servo_config.items():
            center = config['max'] / 2
            self.servo.set_servo(config['index'], center)
            
            if name in self.led_map:
                r, g, b = self.leds.degree_to_rgb(center, config['max'])
                self.leds.set_rgb(self.led_map[name], r, g, b)
        
        for _ in range(3):
            for name, config in self.servo_config.items():
                center = config['max'] / 2
                for pos in [center + 5, center - 5, center]:
                    self.servo.set_servo(config['index'], pos)
                    if name in self.led_map:
                        r, g, b = self.leds.degree_to_rgb(pos, config['max'])
                        self.leds.set_rgb(self.led_map[name], r, g, b)
                    time.sleep(0.05)
                    self.servo.update()
        
        self.uart.write(b"Startup pattern complete.\n")
    
    def process_command(self, command):
        if 'servos' in command:
            self.uart.write(b"Processing servo command\n")
            for name, degrees in command['servos'].items():
                if name in self.servo_config:
                    degrees = min(degrees, self.servo_config[name]['max'])
                    self.servo.set_servo(self.servo_config[name]['index'], degrees)
                    
                    if name in self.led_map:
                        r, g, b = self.leds.degree_to_rgb(degrees, self.servo_config[name]['max'])
                        self.leds.set_rgb(self.led_map[name], r, g, b)
    
    def run(self):
        self.startup_pattern()
        
        try:
            while True:
                if self.uart.any():
                    try:
                        input_data = self.uart.readline()
                        if input_data:
                            self.uart.write(b"Received data: ")
                            self.uart.write(input_data)
                            
                            try:
                                input_str = input_data.decode('utf-8').strip()
                                command = json.loads(input_str)
                                self.process_command(command)
                            except json.JSONDecodeError as e:
                                self.uart.write(b"JSON decode error: ")
                                self.uart.write(str(e).encode())
                                self.uart.write(b"\n")
                            except UnicodeDecodeError as e:
                                self.uart.write(b"Unicode decode error: ")
                                self.uart.write(str(e).encode())
                                self.uart.write(b"\n")
                    except Exception as e:
                        self.uart.write(b"Unexpected error: ")
                        self.uart.write(str(e).encode())
                        self.uart.write(b"\n")
                
                self.servo.update()
        
        except KeyboardInterrupt:
            self.servo.disable_all()
            self.uart.write(b"Program stopped.\n")

# Create and run the robot controller
robot = RobotController()
robot.run()
