#!/usr/bin/env python3
"""
Power monitoring node using INA226 sensor.
Reads voltage and current from 12V battery system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import smbus2
import time

class PowerMonitorNode(Node):
    def __init__(self):
        super().__init__('power_monitor')
        
        # Create I2C interface
        self.bus = smbus2.SMBus(1)  # Use I2C bus 1
        self.address = 0x40  # INA226 address
        
        # Configure INA226
        self.write_register(0x00, 0x4127)  # Config register: 128 samples avg, 8.244ms conversion
        self.write_register(0x05, 0x0001)  # Enable continuous measurements
        
        # Wait for first conversion
        time.sleep(0.1)
        
        # Create publisher
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        
        # Create timer for regular updates
        self.create_timer(1.0, self.publish_power_state)
        
        self.get_logger().info('Power monitor initialized')

    def write_register(self, register, value):
        """Write 16-bit value to register."""
        bytes_val = value.to_bytes(2, byteorder='big')
        self.bus.write_i2c_block_data(self.address, register, list(bytes_val))

    def read_register(self, register):
        """Read 16-bit value from register."""
        result = self.bus.read_i2c_block_data(self.address, register, 2)
        return int.from_bytes(bytes(result), byteorder='big')

    def publish_power_state(self):
        try:
            # Read voltage (mV)
            voltage_raw = self.read_register(0x02)
            voltage = voltage_raw * 1.25 / 1000.0  # Convert to volts
            
            # Read current (mA)
            current_raw = self.read_register(0x04)
            current = current_raw * 1.0 / 1000.0  # Convert to amps
            
            # Calculate power
            power = voltage * current
            
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
