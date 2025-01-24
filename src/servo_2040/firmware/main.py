from servo import Servo, servo2040
from machine import Pin, bootloader
import json
import sys
import select
import time

class SimpleServoController:
    def __init__(self):
        # Initialize USB serial
        self.poll = select.poll()
        self.poll.register(sys.stdin, select.POLLIN)
        
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
        
        print("Servo controller ready\n")

    def set_servo_position(self, servo_index, degrees):
        """Set servo position with safety checks"""
        if 0 <= servo_index < len(self.servos):
            degrees = max(0, min(180, degrees))  # Clamp to 0-180
            self.servos[servo_index].value(degrees - 90)  # Convert to -90..+90
            return True
        return False

    def process_command(self, command):
        """Handle incoming JSON commands"""
        if len(command) > 256:  # Add size limit
            print("Command too long\n")
            return False
            
        try:
            data = json.loads(command)
            
            if 'command' in data and data['command'] == 'enter_bootloader':
                print("Entering bootloader mode...")
                time.sleep(0.5)  # Allow message to send
                bootloader()  # Jump to UF2 bootloader
                return True
                
            if 'servos' in data:
                for servo_cmd in data['servos']:
                    index, position = servo_cmd
                    if self.set_servo_position(index, position):
                        print(f"Servo {index} -> {position}°\n")
                    else:
                        print(f"Invalid servo index: {index}\n")
                return True
                
        except Exception as e:
            print(f"Error: {str(e)}\n")
            return False

    def run(self):
        """Main processing loop"""
        buffer = ''
        while True:
            # Non-blocking read with buffer
            while self.poll.poll(0):
                try:
                    buffer += sys.stdin.read(1)
                    if '\n' in buffer:
                        command, _, buffer = buffer.partition('\n')
                        self.process_command(command.strip())
                except Exception as e:
                    print(f"Read error: {str(e)}\n")
            
            # Process servos at fixed rate
            time.sleep(0.01)  # 100Hz loop

# Start the controller
controller = SimpleServoController()
controller.run()
