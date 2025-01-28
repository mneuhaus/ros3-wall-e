#!/usr/bin/env python3
"""
Power monitoring node using INA226 sensor.
Reads voltage and current from 12V battery system.
Tracks power usage and estimates remaining runtime.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import smbus2
import time
from collections import deque
from datetime import datetime

class BatteryTracker:
    def __init__(self, window_size=300):  # 5 minutes at 1Hz
        self.voltage_history = deque(maxlen=window_size)
        self.power_history = deque(maxlen=window_size)
        self.timestamps = deque(maxlen=window_size)
        self.drop_rates = deque(maxlen=window_size)  # Store historical voltage drop rates
        self.alpha = 0.1  # Exponential moving average factor
        self.last_voltage = None
        self.last_timestamp = None
        self.avg_drop_rate = None
        
    def add_reading(self, voltage, current, power):
        now = datetime.now()
        
        # Calculate instantaneous voltage drop rate
        if self.last_voltage is not None and self.last_timestamp is not None:
            time_diff = (now - self.last_timestamp).total_seconds()
            if time_diff > 0:
                voltage_diff = self.last_voltage - voltage
                instant_drop_rate = voltage_diff / time_diff
                
                # Update exponential moving average of drop rate
                if instant_drop_rate > 0:  # Only consider positive drop rates
                    if self.avg_drop_rate is None:
                        self.avg_drop_rate = instant_drop_rate
                    else:
                        self.avg_drop_rate = (self.alpha * instant_drop_rate + 
                                            (1 - self.alpha) * self.avg_drop_rate)
        
        self.last_voltage = voltage
        self.last_timestamp = now
        
        # Store history
        self.voltage_history.append(voltage)
        self.power_history.append(power)
        self.timestamps.append(now)
        
    def get_average_power(self):
        if not self.power_history:
            return 0.0
        # Use exponential moving average for power too
        weights = [(1 - self.alpha) ** i for i in range(len(self.power_history))]
        weighted_sum = sum(p * w for p, w in zip(self.power_history, reversed(weights)))
        weight_sum = sum(weights)
        return weighted_sum / weight_sum if weight_sum > 0 else 0.0
    
    def estimate_remaining_time(self, current_voltage):
        if (self.avg_drop_rate is None or 
            self.avg_drop_rate <= 0 or 
            not self.voltage_history):
            return float('inf')
        
        # Calculate time until 20% (9.72V for 3S Li-ion)
        voltage_until_20 = current_voltage - 9.72
        seconds_remaining = voltage_until_20 / self.avg_drop_rate
        
        return max(0, seconds_remaining)

class PowerMonitorNode(Node):
    def __init__(self):
        super().__init__('power_monitor')
        self.battery_tracker = BatteryTracker()
        
        # Create I2C interface
        self.bus = smbus2.SMBus(1)  # Use I2C bus 1
        self.address = 0x40  # INA226 address
        
        try:
            # Configure INA226
            # Config register: 128 samples avg, 8.244ms conversion
            self.write_register(0x00, 0x4127)
            self.get_logger().info('Set config register')
            
            # Calibration register: Set for 15A max
            # Cal = 0.00512/(Current_LSB * Rshunt)
            # Current_LSB = 15A/32767 = 0.457mA
            # Rshunt = 0.002 ohm
            # Cal = 0.00512/(0.000457 * 0.002) = 5599 (0x15E7)
            self.write_register(0x05, 0x15E7)
            self.get_logger().info('Set calibration register')
            
            # Enable continuous measurements
            self.write_register(0x06, 0x4527)
            self.get_logger().info('Enabled continuous measurements')
            
            # Verify configuration
            config = self.read_register(0x00)
            cal = self.read_register(0x05)
            self.get_logger().info(f'Config: 0x{config:04x}, Cal: 0x{cal:04x}')
            
            if config != 0x4127:
                self.get_logger().error(f'INA226 config failed! Got: 0x{config:04x}')
            if cal != 0x15E7:
                self.get_logger().error(f'INA226 calibration failed! Got: 0x{cal:04x}')
                
        except Exception as e:
            self.get_logger().error(f'Error configuring INA226: {str(e)}')
        
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
            # Bus voltage LSB is 1.25mV
            voltage = voltage_raw * 1.25 / 1000.0  # Convert to volts
            
            # Read current (mA)
            current_raw = self.read_register(0x04)
            if current_raw > 32767:  # Handle negative values (2's complement)
                current_raw -= 65536
            # Current LSB is 0.457mA (15A/32767)
            current = current_raw * 0.457 / 1000.0  # Convert to amps
            
            # Read power register directly (W)
            power_raw = self.read_register(0x03)
            # Power LSB is 25 times the current LSB
            power = power_raw * (0.457 * 25.0 / 1000.0)  # Convert to watts
            
            self.get_logger().info(
                f'Raw: V=0x{voltage_raw:04x} I=0x{current_raw:04x} P=0x{power_raw:04x}'
            )
            self.get_logger().info(
                f'Converted: {voltage:.4f}V {current:.4f}A {power:.4f}W'
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
            
            # Track battery usage
            self.battery_tracker.add_reading(voltage, current, power)
            avg_power = self.battery_tracker.get_average_power()
            remaining_seconds = self.battery_tracker.estimate_remaining_time(voltage)
            
            # Convert seconds to hours:minutes, handling inf/nan
            if remaining_seconds == float('inf') or remaining_seconds != remaining_seconds:  # Check for inf/nan
                time_str = "--:--"
            else:
                hours = int(remaining_seconds // 3600)
                minutes = int((remaining_seconds % 3600) // 60)
                time_str = f"{hours:02d}:{minutes:02d}"
            
            self.get_logger().info(
                f'Battery: {voltage:.4f}V ({percentage:.1f}%), Current: {current:.4f}A, Power: {power:.4f}W\n'
                f'Average Power: {avg_power:.4f}W, Estimated Runtime: {time_str}'
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
