import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time
import threading
import json
from typing import Dict, List, Tuple


class Servo2040Node(Node):
    def __init__(self):
        super().__init__('servo_2040')
        # Parameter declarations
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.serial_lock = threading.Lock()
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('servo_limits', [180]*9)  # All servos 0-180 by default
        self.declare_parameter('wheel_base', 0.2)  # Distance between tracks in meters
        self.declare_parameter('max_speed', 1.0)  # Maximum speed in m/s
        
        # Get parameters
        self.serial_port: str = self.get_parameter('serial_port').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.baudrate: int = self.get_parameter('baudrate').value
        self.servo_limits: List[Tuple[float, float]] = [
            (0, float(limit)) for limit in self.get_parameter('servo_limits').value
        ]
        
        # Serial connection
        self.serial: serial.Serial = None
        self.connect_serial()

        # Servo state
        self.servo_positions: List[float] = [90.0] * 9  # Default to center position
        self.prev_positions = self.servo_positions.copy()  # Track previous positions
        self.movement_increment: float = 5.0
        
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
        
        # Add rate limiting parameters
        self.declare_parameter('max_command_rate', 30.0)  # Hz
        self.declare_parameter('deadband_threshold', 0.05)
        
        # Add rate limiting members
        self.last_command_time = 0.0
        self.command_period = 1.0 / self.get_parameter('max_command_rate').value
        
        # Movement increment in degrees
        self.movement_increment = 5
        
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

    def connect_serial(self) -> None:
        """Establish serial connection with automatic retries"""
        while not self.serial:
            try:
                self.serial = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    timeout=2,
                    write_timeout=2
                )
                self.serial.reset_input_buffer()
                self.serial.reset_output_buffer()
                self.get_logger().info(f"Connected to {self.serial_port}")
                time.sleep(2.5)  # Increased firmware init time
            except serial.SerialException as e:
                self.get_logger().error(f"Connection failed: {e}, retrying...")
                time.sleep(1.5)

    def joy_callback(self, msg: Joy) -> None:
        """Handle joystick input with rate limiting"""
        if not self.serial:
            return
            
        now = time.monotonic()
        if (now - self.last_command_time) < self.command_period:
            return

        # Map buttons to control all servos
        if len(msg.buttons) >= 2:
            # Check for meaningful input
            if abs(msg.buttons[0] - msg.buttons[1]) < self.get_parameter('deadband_threshold').value:
                return
                
            self.last_command_time = now
            direction = msg.buttons[0] - msg.buttons[1]  # A=up, B=down
            if direction != 0:
                self.update_servo_positions(direction)
                self.send_servo_command()
        
    def update_servo_positions(self, direction: int) -> None:
        """Calculate new positions within limits"""
        for i in range(9):
            new_pos = self.servo_positions[i] + direction * self.movement_increment
            min_limit, max_limit = self.servo_limits[i]
            self.servo_positions[i] = max(min_limit, min(new_pos, max_limit))

    def send_servo_command(self) -> None:
        """Send JSON command to firmware with flow control"""
        with self.serial_lock:
            if not self.serial or not self.serial.is_open:
                self.reconnect_serial()
                return

        try:
            # Create command
            command_data = {
                "servos": [[idx, float(pos)] 
                          for idx, pos in enumerate(self.servo_positions)
                          if pos != self.prev_positions[idx]]
            }
            
            # Only send if there are changes
            if not command_data["servos"]:
                return

            command = json.dumps(command_data) + "\n"
            
            # Check output buffer capacity
            if self.serial.out_waiting > 1024:
                self.get_logger().warning("Output buffer full, clearing...")
                self.serial.reset_output_buffer()

            # Write with timeout handling
            self.serial.write(command.encode())
            self.serial.flush()
            self.prev_positions = self.servo_positions.copy()
            self.get_logger().debug(f"Sent command: {command.strip()}")

        except serial.SerialTimeoutException:
            self.get_logger().warning("Write timeout - reconnecting...")
            self.reconnect_serial()
        except (serial.SerialException, TypeError) as e:
            self.get_logger().error(f"Command failed: {str(e)}")
            self.reconnect_serial()

    def read_serial_data(self) -> None:
        """Handle incoming serial data"""
        while rclpy.ok():
            with self.serial_lock:
                if not self.serial or not self.serial.is_open:
                    time.sleep(0.1)
                    continue
                
                try:
                    if self.serial.in_waiting:
                        line = self.serial.readline().decode().strip()
                        if line:
                            self.get_logger().info(f"Firmware: {line}")
                except (UnicodeDecodeError, OSError, serial.SerialException) as e:
                    self.get_logger().warn(f"Serial read error: {str(e)}")
                    self.reconnect_serial()
            
            time.sleep(0.01)

    def enter_bootloader(self) -> None:
        """Trigger firmware bootloader mode"""
        try:
            self.serial.write(b'{"command": "enter_bootloader"}\n')
            self.serial.flush()
            time.sleep(0.6)
            self.serial.close()
        except Exception:
            pass
            
    def reconnect_serial(self) -> None:
        """Handle serial connection recovery"""
        with self.serial_lock:
            self.get_logger().warn("Reconnecting to serial...")
            try:
                if self.serial:
                    self.serial.close()
            except Exception:
                pass
            self.serial = None
            self.connect_serial()


def main(args=None):
    rclpy.init(args=args)
    node = Servo2040Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        if node.serial:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()
