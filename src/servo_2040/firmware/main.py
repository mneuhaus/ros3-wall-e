from plasma import WS2812
from machine import Pin, UART, PWM
import random
import time
import json

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


class ServoController:
    def __init__(self, pin_base, num_servos):
        self.num_servos = num_servos
        self.servos = []
        
        # Initialize PWM for each servo
        for i in range(num_servos):
            pwm = PWM(Pin(pin_base + i))
            pwm.freq(50)  # 50Hz for standard servos
            pwm.duty_u16(4915)  # Center position (1.5ms pulse)
            self.servos.append(pwm)
    
    def set_servo(self, index, degrees):
        """Set servo position in degrees (0-180)."""
        if 0 <= index < self.num_servos:
            # Convert degrees to duty cycle (0.5ms to 2.5ms pulse)
            # At 50Hz, period is 20ms (65535 units)
            # 0.5ms = 1638 units, 2.5ms = 8192 units
            duty = int(1638 + (degrees * (8192 - 1638) / 180))
            self.servos[index].duty_u16(duty)
    
    def update(self):
        """No need for explicit updates with PWM."""
        pass
    
    def disable_all(self):
        """Disable all servo outputs."""
        for pwm in self.servos:
            pwm.duty_u16(0)

class RobotController:
    def __init__(self):
        # Set up UART for debug output
        self.uart = UART(0, baudrate=115200)
        self.uart.init(baudrate=115200, bits=8, parity=None, stop=1, tx=Pin(0), rx=Pin(1))
        self.uart.write(b"Initializing robot controller...\n")
        
        # Initialize LED controller
        self.leds = LEDController(10, 18)  # 10 LEDs on pin 18
        self.leds.set_random_color(0, brightness=0.5)  # Power indicator LED
        
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
            
            # if name in self.led_map:
            #     r, g, b = self.leds.degree_to_rgb(center, config['max'])
            #     self.leds.set_rgb(self.led_map[name], r, g, b)
        
        for _ in range(3):
            for name, config in self.servo_config.items():
                center = config['max'] / 2
                for pos in [center + 30, center - 30, center]:
                    self.servo.set_servo(config['index'], pos)
                    # if name in self.led_map:
                    #     r, g, b = self.leds.degree_to_rgb(pos, config['max'])
                    #     self.leds.set_rgb(self.led_map[name], r, g, b)
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
                time.sleep_ms(10)  # Small delay to prevent tight loop
        
        except KeyboardInterrupt:
            self.servo.disable_all()
            self.uart.write(b"Program stopped.\n")

# Create and run the robot controller
robot = RobotController()
robot.run()
