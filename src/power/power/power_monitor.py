#!/usr/bin/env python3
"""
Power monitoring node using INA226 sensor.
Reads voltage and current from 12V battery system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import board
import busio
from adafruit_ina226 import INA226

class PowerMonitorNode(Node):
    def __init__(self):
        super().__init__('power_monitor')
        
        # Create I2C interface
        i2c = busio.I2C(board.SCL, board.SDA)
        self.ina226 = INA226(i2c, address=0x40)
        
        # Configure INA226
        self.ina226.averaging_count = 128  # Average over 128 samples
        self.ina226.conversion_time = 0.008398  # 8.398ms conversion time
        
        # Create publisher
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        
        # Create timer for regular updates
        self.create_timer(1.0, self.publish_power_state)
        
        self.get_logger().info('Power monitor initialized')

    def publish_power_state(self):
        try:
            voltage = self.ina226.bus_voltage
            current = self.ina226.current
            power = self.ina226.power
            
            msg = BatteryState()
            msg.voltage = voltage
            msg.current = current
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.present = True
            
            self.battery_pub.publish(msg)
            
            self.get_logger().info(
                f'Battery: {voltage:.2f}V, Current: {current:.2f}A, Power: {power:.2f}W'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error reading power data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
