from servo import Servo, servo2040
from machine import UART, Pin
import json

class SimpleServoController:
    def __init__(self):
        # Initialize UART for communication
        self.uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))
        
        # Initialize all servo pins from servo2040 board
        self.servos = [
            Servo(servo2040.SERVO_1),
            Servo(servo2040.SERVO_2),
            Servo(servo2040.SERVO_3),
            Servo(servo2040.SERVO_4),
            Servo(servo2040.SERVO_5),
            Servo(servo2040.SERVO_6),
            Servo(servo2040.SERVO_7),
            Servo(servo2040.SERVO_8),
            Servo(servo2040.SERVO_9)
        ]
        
        # Enable all servos on startup
        for servo in self.servos:
            servo.enable()
        
        self.uart.write(b"Servo controller ready\n")

    def set_servo_position(self, servo_index, degrees):
        """Set servo position with safety checks"""
        if 0 <= servo_index < len(self.servos):
            degrees = max(0, min(180, degrees))  # Clamp to 0-180
            self.servos[servo_index].value(degrees - 90)  # Convert to -90..+90
            return True
        return False

    def process_command(self, command):
        """Handle incoming JSON commands"""
        try:
            data = json.loads(command)
            
            if 'servos' in data:
                for servo_cmd in data['servos']:
                    index, position = servo_cmd
                    if self.set_servo_position(index, position):
                        self.uart.write(f"Servo {index} -> {position}°\n".encode())
                    else:
                        self.uart.write(f"Invalid servo index: {index}\n".encode())
                return True
                
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.uart.write(f"Error: {str(e)}\n".encode())
            return False

    def run(self):
        """Main processing loop"""
        while True:
            if self.uart.any():
                try:
                    command = self.uart.readline().decode().strip()
                    if command:
                        self.process_command(command)
                except UnicodeDecodeError:
                    self.uart.write(b"Invalid command encoding\n")

# Start the controller
controller = SimpleServoController()
controller.run()
