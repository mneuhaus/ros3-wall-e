import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time
import json
import threading


class Servo2040Node(Node):
    def __init__(self):
        super().__init__('servo_2040')
        self.declare_parameter('serial_port', '/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e6617c93e3617129-if00')
        self.declare_parameter('baudrate', 115200)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.serial = None

        # Default servo positions (middle positions)
        self.servo_positions = [45.0, 45.0, 20.0, 20.0, 45.0, 90.0, 90.0, 90.0, 90.0]
        
        # Subscribe to joy messages
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Button mappings for testing - move all servos
        self.button_map = {
            0: {'direction': 1},   # A button - all servos up/right
            1: {'direction': -1},  # B button - all servos down/left
        }
        
        # Movement increment in degrees
        self.movement_increment = 5.0
        
        # Servo limits
        self.servo_limits = [
            (0, 90),   # eyebrow_left
            (0, 90),   # eyebrow_right
            (0, 40),   # head_left
            (0, 40),   # head_right
            (0, 90),   # neck_tilt
            (0, 180),  # neck_raise
            (0, 180),  # neck_pan
            (0, 180),  # arm_left
            (0, 180),  # arm_right
        ]

        # Initial attempt to connect
        self.connect_serial()

        # Start serial reader thread
        self.reader_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.reader_thread.start()

        # Servo ranges
        self.servo_ranges = {
            'eyebrow_left': 90,    # SERVO_1
            'eyebrow_right': 90,   # SERVO_2
            'head_left': 40,       # SERVO_3
            'head_right': 40,      # SERVO_4
            'neck_tilt': 90,       # SERVO_5
            'neck_raise': 180,     # SERVO_6
            'neck_pan': 180,       # SERVO_7
            'arm_left': 180,       # SERVO_8
            'arm_right': 180       # SERVO_9
        }

    def connect_serial(self):
        """Attempts to connect to the serial port."""
        while self.serial is None:
            try:
                self.serial = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    timeout=1,
                    write_timeout=1
                )
                self.get_logger().info(f"Connected to {self.serial_port} at {self.baudrate} baud")
                # Give the device time to initialize
                time.sleep(2.0)
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
                time.sleep(1)  # Wait 1 second before retrying

    def joy_callback(self, msg):
        """Handle incoming joy messages."""
        position_changed = False
        
        # Check each mapped button
        for button_idx, mapping in self.button_map.items():
            if button_idx < len(msg.buttons) and msg.buttons[button_idx]:
                direction = mapping['direction']
                
                # Update all servo positions
                for servo_idx in range(len(self.servo_positions)):
                    # Calculate new position
                    new_position = self.servo_positions[servo_idx] + (direction * self.movement_increment)
                    
                    # Apply limits
                    min_val, max_val = self.servo_limits[servo_idx]
                    new_position = max(min_val, min(max_val, new_position))
                    
                    # Update position if changed
                    if new_position != self.servo_positions[servo_idx]:
                        self.get_logger().info(f"Moving servo {servo_idx} from {self.servo_positions[servo_idx]} to {new_position} degrees")
                        self.servo_positions[servo_idx] = new_position
                        position_changed = True
        
        # Send command if positions changed
        if position_changed:
            try:
                command = {
                    'servos': {
                        'eyebrow_left': self.servo_positions[0],
                        'eyebrow_right': self.servo_positions[1],
                        'head_left': self.servo_positions[2],
                        'head_right': self.servo_positions[3],
                        'neck_tilt': self.servo_positions[4],
                        'neck_raise': self.servo_positions[5],
                        'neck_pan': self.servo_positions[6],
                        'arm_left': self.servo_positions[7],
                        'arm_right': self.servo_positions[8]
                    }
                }
                
                if self.serial:
                    command_str = json.dumps(command) + '\n'
                    try:
                        self.serial.write(command_str.encode())
                        self.serial.flush()  # Ensure the data is sent
                    except serial.SerialTimeoutException:
                        self.get_logger().error("Write timeout occurred!")
                    except serial.SerialException as e:
                        self.get_logger().error(f"Serial error during write: {e}")
            except Exception as e:
                self.get_logger().error(f"Error sending command: {e}")

    def read_serial_data(self):
        """Continuously reads data from the serial port in a separate thread."""
        while rclpy.ok():
            if self.serial is None:
                self.connect_serial()
                continue

            try:
                data = self.serial.readline().decode('utf-8').strip()
                if data:  # Only log if there's actual data
                    self.get_logger().info(f"Firmware: {data}")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error: {e}. Reconnecting...")
                self.serial = None
            except Exception as e:
                self.get_logger().error(f"Error reading serial: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = Servo2040Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()
