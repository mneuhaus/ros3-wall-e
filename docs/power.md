# Power Monitoring Package

## Overview
The power package provides battery monitoring and power management for the Wall-E robot using an INA226 current/voltage sensor. It tracks battery state, estimates remaining runtime, and implements safety shutdown at low battery levels.

## Hardware Requirements
- INA226 current/voltage sensor
- I2C connection to Raspberry Pi (bus 1)
- 12V LiPo battery system (3S configuration)
- Shunt resistor: 0.002Ω

## Configuration

### INA226 Settings
| Register | Value    | Description                    |
|----------|----------|--------------------------------|
| Config   | 0x4127   | 128 samples avg, 8.244ms conv  |
| Cal      | 0x15E7   | Calibrated for 15A max        |
| Mask/En  | 0x4527   | Continuous measurements        |

### Battery Specifications
| Parameter          | Value | Description              |
|-------------------|-------|--------------------------|
| Nominal Voltage   | 11.1V | 3S Li-ion configuration |
| Maximum Voltage   | 12.6V | 4.2V per cell           |
| Minimum Voltage   | 9.0V  | 3.0V per cell           |
| Capacity         | 2.5Ah | Per cell capacity        |
| Shutdown Threshold| 10%   | Auto shutdown trigger    |

## ROS2 Interface

### Node: power_monitor
The main node providing power monitoring functionality.

### Published Topics
| Topic           | Type                  | Description          |
|-----------------|----------------------|---------------------|
| /battery_state  | sensor_msgs/BatteryState | Battery status info |

### Parameters
None currently implemented.

### Services Used
| Service    | Type            | Description        |
|-----------|-----------------|-------------------|
| /shutdown | std_srvs/Trigger| System shutdown    |

## Usage

1. Start the power monitor:
```bash
ros2 run power power_monitor
```

2. Monitor battery state:
```bash
ros2 topic echo /battery_state
```

3. View power statistics:
```bash
ros2 topic hz /battery_state
```

## Battery State Message
The node publishes BatteryState messages with:
- Voltage (V)
- Current (A)
- Power (W)
- Percentage remaining
- Power supply status
- Presence indicator

## Features

### Power Tracking
- Real-time voltage and current monitoring
- Power consumption calculation
- Rolling average power usage
- Remaining runtime estimation

### Safety Features
- Automatic shutdown at 10% battery
- Voltage-based SoC validation
- Over/under voltage detection
- Current monitoring

### Data Smoothing
- EMA filtering for current (α=0.1)
- EMA filtering for power (α=0.01)
- 60-second rolling average window
- Coulomb counting with voltage correction

## Troubleshooting

### Common Issues

1. **I2C Communication Errors**
   - Check I2C bus connection
   - Verify device address (0x40)
   - Check permissions on /dev/i2c-1

2. **Inaccurate Readings**
   - Verify shunt resistor value
   - Check calibration register
   - Allow warm-up period

3. **Shutdown Not Working**
   - Check shutdown service availability
   - Verify battery percentage calculation
   - Check system permissions

### LED Status
The power monitoring system uses the following LED patterns:
- Solid Green: Normal operation
- Blinking Red: Low battery warning
- Solid Red: Critical battery level

## Dependencies
- ROS2 Humble
- smbus2 Python package
- std_srvs
- sensor_msgs

## Future Improvements
1. Configurable thresholds via ROS parameters
2. Battery health monitoring
3. Temperature sensing integration
4. Power consumption logging
5. Web interface integration

## Safety Notes
1. Never discharge battery below 9V
2. Monitor temperature during high current draw
3. Ensure proper ventilation
4. Regular calibration checks recommended
