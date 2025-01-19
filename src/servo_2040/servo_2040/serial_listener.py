import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import json


class SerialListener(Node):
    def __init__(self):
        super().__init__('serial_listener')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.serial = None

        # Create subscription for servo commands
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'servo_commands',
            self.servo_command_callback,
            10
        )

        # Initial attempt to connect
        self.connect_serial()

        # Timer for polling serial data (for debugging/feedback)
        self.timer = self.create_timer(0.1, self.read_serial_data)

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
                self.serial = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                self.get_logger().info(f"Connected to {self.serial_port} at {self.baudrate} baud.")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
                time.sleep(1)  # Wait 1 second before retrying

    def servo_command_callback(self, msg):
        """Handle incoming servo commands."""
        if len(msg.data) != 9:
            self.get_logger().error("Expected 9 servo values")
            return
        
        try:
            # Create command dictionary
            command = {
                'servos': {
                    'eyebrow_left': msg.data[0],
                    'eyebrow_right': msg.data[1],
                    'head_left': msg.data[2],
                    'head_right': msg.data[3],
                    'neck_tilt': msg.data[4],
                    'neck_raise': msg.data[5],
                    'neck_pan': msg.data[6],
                    'arm_left': msg.data[7],
                    'arm_right': msg.data[8]
                }
            }
            
            # Send command over serial
            if self.serial:
                command_str = json.dumps(command) + '\n'
                self.serial.write(command_str.encode())
                self.get_logger().debug(f"Sent: {command_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")

    def read_serial_data(self):
        """Reads data from the serial port for debugging/feedback."""
        if self.serial is None:
            self.connect_serial()
            return

        try:
            if self.serial.in_waiting > 0:
                data = self.serial.readline().decode('utf-8').strip()
                self.get_logger().debug(f"Received: {data}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}. Reconnecting...")
            self.serial = None


def main(args=None):
    rclpy.init(args=args)
    node = SerialListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()
