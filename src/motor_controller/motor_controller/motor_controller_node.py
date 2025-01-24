#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading
import time  # Add this line

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller')
        # Parameters
        self.declare_parameter('port', '/dev/ttyAMA2')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_base', 0.2)
        self.declare_parameter('max_speed', 1.0)

        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        # Initialize serial communication
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to serial port {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port}: {e}")
            rclpy.shutdown()
            return

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Start a thread to read from the Pico (if needed)
        self.read_thread = threading.Thread(target=self.read_from_pico)
        self.read_thread.daemon = True
        self.read_thread.start()

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f"Received cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        # Convert Twist message to left and right motor speeds
        linear = msg.linear.x  # Forward/backward speed
        angular = msg.angular.z  # Rotation speed

        # Compute motor speeds (differential drive)
        left_speed = linear - (angular * self.wheel_base / 2)
        right_speed = linear + (angular * self.wheel_base / 2)

        # Scale speeds to -100 to 100
        left_speed = int(max(min(left_speed / self.max_speed * 100, 100), -100))
        right_speed = int(max(min(right_speed / self.max_speed * 100, 100), -100))

        # Send track control command to Servo2040
        command = f'{{"tracks":[{left_speed},{right_speed}]}}\n'
        self.ser.write(command.encode('utf-8'))

    def read_from_pico(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8').strip()
                    # Process data if the Pico sends any feedback (optional)
                    self.get_logger().info(f"Received from Pico: {data}")
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().warn(f"Error reading from Pico: {e}")

def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()
    rclpy.spin(motor_controller_node)
    motor_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
