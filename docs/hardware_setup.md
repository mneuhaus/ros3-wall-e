# Hardware Setup Guide

## Components List

### Core Components
- Raspberry Pi CM4 (4GB RAM, 32GB eMMC, WiFi)
- Pimoroni Servo 2040
- Cytron MD13S Motor Controller
- 12V 10000mAh LiPo Battery
- 5V 2000mAh Auxiliary Battery
- 8BitDo Lite 2 Controller

### Servos
- 9x MG996R Servos
- 2x DC Motors with Encoders

### Display & Audio
- GC9A01A 240x240 Round LCD
- I2S MEMS Microphone
- I2S Audio Amplifier
- 3W Speaker

## Assembly Instructions

### 1. Power System

1. Main Battery Connection:
   - Connect 12V battery to power distribution board
   - Install voltage monitoring circuit
   - Add protection circuitry:
     - Over-voltage cutoff: 12.6V
     - Under-voltage cutoff: 9V
     - Current limit: 15A

2. Logic Power:
   - Connect 5V auxiliary battery
   - Wire to Raspberry Pi and logic circuits
   - Install monitoring circuit

### 2. Motor System

1. Motor Controller Setup:
   - Mount MD13S controller
   - Connect motor power (12V)
   - Wire control signals:
     - PWM inputs (GPIO 13, 17)
     - Direction controls (GPIO 14, 18)

2. Motor Installation:
   - Mount motors in chassis
   - Connect encoders
   - Verify free movement

### 3. Servo System

1. Servo 2040 Setup:
   - Mount controller board
   - Connect USB to Raspberry Pi
   - Wire servo power (6V)

2. Servo Connections:
   | Servo Function    | Pin | Power Rail |
   |------------------|-----|------------|
   | Head Tilt        | 0   | 6V        |
   | Head Pan         | 1   | 6V        |
   | Left Arm         | 2   | 6V        |
   | Right Arm        | 3   | 6V        |
   | Left Gripper     | 4   | 6V        |
   | Right Gripper    | 5   | 6V        |
   | Left Eye         | 6   | 6V        |
   | Right Eye        | 7   | 6V        |
   | Neck Tilt        | 8   | 6V        |

### 4. Display & Audio

1. Display Setup:
   - Connect GC9A01A:
     - SPI0: GPIO 2-6
     - Backlight: GPIO 7
   - Mount in eye assembly

2. Audio System:
   - Connect I2S microphone
   - Wire amplifier
   - Mount speaker

### 5. Control System

1. Raspberry Pi Setup:
   - Install CM4 in carrier board
   - Connect peripherals:
     - USB hub
     - WiFi antennas
     - Debug serial port

2. Controller Pairing:
   - Enable Bluetooth
   - Pair 8BitDo controller
   - Test inputs

## Wiring Diagram

```
                                   [Raspberry Pi CM4]
                                          |
                 +------------------------+------------------------+
                 |                        |                       |
          [Servo 2040]             [Motor Controller]     [Display/Audio]
                |                         |                       |
        [Servo Array]              [Drive Motors]         [Speakers/Mic]
```

## Testing Procedure

1. Power Systems:
   - Check voltage levels
   - Verify protection circuits
   - Test current draw

2. Motion Control:
   - Calibrate servos
   - Test motor operation
   - Verify encoder feedback

3. Sensor Systems:
   - Test display
   - Check audio input/output
   - Verify all connections

## Maintenance

### Regular Checks
- Battery voltage
- Servo calibration
- Motor encoder operation
- Connection security

### Lubrication Schedule
- Drive motors: Every 6 months
- Servo gears: Annually
- Wheel bearings: Every 3 months

## Safety Considerations

1. Battery Safety:
   - Never exceed voltage limits
   - Monitor temperature
   - Proper charging procedures

2. Mechanical Safety:
   - Torque limits on servos
   - Motor current monitoring
   - Emergency stop procedure

3. Electrical Safety:
   - Proper insulation
   - Ground connections
   - Short circuit protection
