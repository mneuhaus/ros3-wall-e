# Servo 2040 Package Documentation

## Overview
The servo_2040 package provides control over Wall-E's servos and motors using a Pimoroni Servo 2040 board. It implements the EVE protocol for communication and offers ROS2 interfaces for motion control.

## Hardware Requirements
- Pimoroni Servo 2040 board
- Servos (up to 18 channels)
- DC motors for tracks (2 channels)
- USB connection to main computer

## Software Dependencies
- ROS2 Humble
- Python 3.8+
- pyserial
- Custom firmware (built with Pico SDK)

## Configuration Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| serial_port | string | /dev/ttyACM0 | Serial port for Servo 2040 |
| baudrate | int | 115200 | Serial communication speed |
| servo_limits | float[] | [180]*9 | Maximum angle for each servo |
| wheel_base | float | 0.2 | Distance between tracks (meters) |
| max_speed | float | 1.0 | Maximum track speed (m/s) |
| servo_pins | int[] | [0-8] | GPIO pins for servos |

## Usage Instructions

### Starting the Node
```bash
ros2 run servo_2040 servo_2040_node
```

### Topics
| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| /cmd_vel | geometry_msgs/Twist | Sub | Track motion control |
| /servo_position | std_msgs/Float32MultiArray | Sub | Set servo angles |
| /servo_state | std_msgs/Float32MultiArray | Pub | Current servo positions |

### Services
| Service | Type | Description |
|---------|------|-------------|
| /reset_servos | std_srvs/Trigger | Reset all servos to home |
| /calibrate_servos | std_srvs/Trigger | Run servo calibration |

### Example Commands
```bash
# Move forward at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}"

# Set servo positions
ros2 topic pub /servo_position std_msgs/Float32MultiArray "data: [90.0, 45.0, 135.0]"

# Reset servos
ros2 service call /reset_servos std_srvs/Trigger
```

## Firmware Protocol
The package uses the EVE protocol for communication with the Servo 2040 board. See `docs/eve-protokoll.md` for detailed protocol specification.

### Key Commands
- `INIT_GPIO PIN=<pin> MODE=<SERVO|PWM>`
- `MOVE_SERVO PIN=<pin> POS=<angle>`
- `SET_GPIO PIN=<pin> PWM=<0-255>`

## Implementation Notes
- Uses threading for non-blocking serial communication
- Implements automatic reconnection on serial errors
- Provides smooth acceleration/deceleration for tracks
- Supports emergency stop functionality

## Troubleshooting Guide

### Common Issues
1. **No Serial Connection**
   - Check USB connection
   - Verify serial port permissions
   - Try reconnecting the USB cable

2. **Servo Not Moving**
   - Verify servo power supply
   - Check servo pin configuration
   - Ensure angle is within limits

3. **Track Control Issues**
   - Verify motor driver connections
   - Check PWM frequency settings
   - Validate track speed parameters

### LED Status Indicators
- Solid Green: Normal operation
- Blinking Green: Processing command
- Red Flash: Error condition
- Blue: Calibration mode

## API Documentation

### Servo2040Node Class
Main ROS2 node class handling servo and motor control.

#### Methods
- `init_hardware()`: Initialize serial connection and hardware
- `connect_serial()`: Establish serial communication
- `update_servo_positions()`: Update servo angles
- `send_servo_command()`: Send commands via EVE protocol
- `reconnect_serial()`: Handle serial reconnection

#### Parameters
All parameters can be set via ROS2 parameter system:
```bash
ros2 param set /servo_2040 max_speed 0.8
```

## Safety Features
- Software limits on servo angles
- Current monitoring for servos
- Emergency stop functionality
- Automatic servo shutdown on errors

## Performance Considerations
- Serial communication at 115200 baud
- Update rate: 50Hz for servos
- PWM frequency: 50Hz for servos, 1kHz for motors
