import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
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
        self.declare_parameter('servo_limits', [180]*9)
        self.declare_parameter('wheel_base', 0.2)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('servo_pins', [0, 1, 2, 3, 4, 5, 6, 7, 8])  # GPIO pins for servos
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.baudrate = self.get_parameter('baudrate').value
        self.servo_limits = [float(limit) for limit in self.get_parameter('servo_limits').value]
        self.servo_pins = self.get_parameter('servo_pins').value
        
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
        
        # Subscribe to movement commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Button mappings for testing - move all servos
        self.button_map = {
            0: {'direction': 1},   # A button - all servos up/right
            1: {'direction': -1},  # B button - all servos down/left
        }
        
        # Add rate limiting parameters
        self.declare_parameter('max_command_rate', 100.0)  # Hz
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

        # Initialize all hardware pins
        self.init_hardware()
        
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

    def init_hardware(self) -> None:
        """Initialize all hardware pins and configurations"""
        if not self.serial:
            self.connect_serial()
            
        # Initialize all servo pins
        for pin in self.servo_pins:
            cmd = f"INIT_GPIO PIN={pin} MODE=SERVO FREQ=50\n"
            try:
                self.serial.write(cmd.encode())
                time.sleep(0.1)  # Allow time for initialization
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to initialize servo pin {pin}: {e}")
                raise RuntimeError(f"Hardware initialization failed: {e}")
                
        # Initialize track control pins
        track_init_commands = [
            # PWM pins for speed control
            "INIT_GPIO PIN=13 MODE=PWM FREQ=1000\n",  # Left track PWM
            "INIT_GPIO PIN=17 MODE=PWM FREQ=1000\n",  # Right track PWM
            # Direction control pins
            "INIT_GPIO PIN=14 MODE=OUTPUT\n",  # Left track direction
            "INIT_GPIO PIN=18 MODE=OUTPUT\n",  # Right track direction
        ]
        
        for cmd in track_init_commands:
            try:
                self.serial.write(cmd.encode())
                time.sleep(0.1)  # Allow time for initialization
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to initialize track control: {e}")
        
        # Set all servos to center position
        for pin in self.servo_pins:
            cmd = f"MOVE_SERVO PIN={pin} POS=90.0\n"
            try:
                self.serial.write(cmd.encode())
                time.sleep(0.1)
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to center servo {pin}: {e}")

    def connect_serial(self) -> None:
        """
        Establish serial connection with automatic retries.
        
        Raises:
            RuntimeError: If connection fails after max retries
        """
        max_retries = 5
        retry_count = 0
        
        while not self.serial and retry_count < max_retries:
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
                time.sleep(0.5)  # Brief init time
            except serial.SerialException as e:
                retry_count += 1
                self.get_logger().error(f"Connection failed ({retry_count}/{max_retries}): {e}")
                if retry_count >= max_retries:
                    raise RuntimeError(f"Failed to connect after {max_retries} attempts")
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
        """Send EVE Protocol commands to firmware"""
        with self.serial_lock:
            if not self.serial or not self.serial.is_open:
                self.reconnect_serial()
                return

        try:
            # Build EVE Protocol commands for changed positions
            commands = []
            for idx, (current, prev) in enumerate(zip(self.servo_positions, self.prev_positions)):
                if current != prev:
                    pin = self.servo_pins[idx]
                    commands.append(f"MOVE_SERVO PIN={pin} POS={current:.1f} SPEED=50\n")

            # Only send if there are changes
            if not commands:
                return

            # Join commands and send
            command_str = "".join(commands)
            self.serial.write_timeout = 0
            bytes_written = self.serial.write(command_str.encode())
            
            if bytes_written != len(command_str):
                self.get_logger().warning("Partial write - clearing buffers")
                self.serial.reset_output_buffer()
            else:
                self.prev_positions = self.servo_positions.copy()
                self.get_logger().debug(f"Sent commands: {command_str.strip().replace('\n', '; ')}")

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
                            if line.startswith("Track"):
                                self.get_logger().debug(f"Track Debug: {line}")
                            else:
                                self.get_logger().info(f"Firmware: {line}")
                except (UnicodeDecodeError, OSError, serial.SerialException) as e:
                    self.get_logger().warn(f"Serial read error: {str(e)}")
                    self.reconnect_serial()
            
            time.sleep(0.0001)  # Poll at 10kHz

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
            
    def cmd_vel_callback(self, msg: Twist) -> None:
        """Handle movement commands using EVE Protocol"""
        self.get_logger().debug(f"Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        
        # Convert Twist to differential drive speeds
        linear = msg.linear.x
        angular = msg.angular.z
        left_speed = linear + (angular * self.wheel_base / 2)
        right_speed = linear - (angular * self.wheel_base / 2)

        # Scale to PWM values (0-65535)
        left_pwm = int(abs(left_speed / self.max_speed) * 65535)
        right_pwm = int(abs(right_speed / self.max_speed) * 65535)

        # Build EVE Protocol commands
        commands = [
            f"SET_GPIO PIN=13 PWM={left_pwm}\n",
            f"SET_GPIO PIN=17 PWM={right_pwm}\n",
            f"SET_GPIO PIN=14 STATE={'HIGH' if left_speed >= 0 else 'LOW'}\n",
            f"SET_GPIO PIN=18 STATE={'HIGH' if right_speed >= 0 else 'LOW'}\n"
        ]

        # Send commands
        with self.serial_lock:
            try:
                command_str = "".join(commands)
                self.serial.write(command_str.encode('utf-8'))
                self.get_logger().info(f"Sent track commands: {command_str.strip().replace('\n', '; ')}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send track command: {e}")
                self.reconnect_serial()


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
