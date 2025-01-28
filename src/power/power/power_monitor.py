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
        # Config register: 128 samples avg, 8.244ms conversion
        self.write_register(0x00, 0x4127)
        # Calibration register: Set for 15A max at 1.25mA/bit
        self.write_register(0x05, 0x0800)
        # Enable continuous measurements
        self.write_register(0x06, 0x4527)
        
        # Verify configuration
        config = self.read_register(0x00)
        if config != 0x4127:
            self.get_logger().error(f'INA226 config failed! Got: {config:04x}')
        
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
            
            # Read current (mA) - INA226 LSB is 1.25mA by default
            current_raw = self.read_register(0x04)
            if current_raw > 32767:  # Handle negative values (2's complement)
                current_raw -= 65536
            current = current_raw * 1.25 / 1000.0  # Convert to amps
            
            # Calculate power
            power = voltage * current if voltage > 0 and abs(current) < 15.0 else 0.0
            
            self.get_logger().debug(
                f'Raw values - Voltage: {voltage_raw}, Current: {current_raw}'
            )
            
            # Calculate battery percentage (3S Li-ion: 9.0V empty to 12.6V full)
            percentage = max(0.0, min(100.0, (voltage - 9.0) * 100.0 / (12.6 - 9.0)))
            
            msg = BatteryState()
            msg.voltage = voltage
            msg.current = current
            msg.percentage = percentage
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.present = True
            
            self.battery_pub.publish(msg)
            
            self.get_logger().info(
                f'Battery: {voltage:.2f}V ({percentage:.1f}%), Current: {current:.2f}A, Power: {power:.2f}W'
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
