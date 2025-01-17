import rclpy
from rclpy.node import Node
import serial
import time


class SerialListener(Node):
    def __init__(self):
        super().__init__('serial_listener')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.serial = None

        # Initial attempt to connect
        self.connect_serial()

        # Timer for polling serial data
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def connect_serial(self):
        """Attempts to connect to the serial port."""
        while self.serial is None:
            try:
                self.serial = serial.Serial(self.serial_port, self.baudrate, timeout=1)
                self.get_logger().info(f"Connected to {self.serial_port} at {self.baudrate} baud.")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to {self.serial_port}: {e}")
                time.sleep(1)  # Wait 1 second before retrying

    def read_serial_data(self):
        """Reads data from the serial port."""
        if self.serial is None:
            self.connect_serial()
            return

        try:
            if self.serial.in_waiting > 0:
                data = self.serial.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received: {data}")
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
