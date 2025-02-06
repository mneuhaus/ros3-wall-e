# Wall-E Robot Project

## Main Hardware Components

### Raspberry Pi CM4
- **Model:** Compute Module 4 with WiFi
- **CPU:** Quad-core Cortex-A72 (ARM v8) 64-bit @ 1.5GHz
- **RAM:** 4GB LPDDR4-3200
- **Storage:** 32GB eMMC Flash
- **Connectivity:** 2.4GHz and 5.0GHz IEEE 802.11ac wireless, Bluetooth 5.0, BLE
- **Purpose:** Main robot controller running ROS2 nodes

### Pimoroni Servo 2040
- **Microcontroller:** RP2040 (Dual-core ARM Cortex M0+ @ 133MHz)
- **Servo Channels:** 18 independent channels
- **Current Monitoring:** Per-channel current sensing
- **Interface:** USB Serial for commands and feedback
- **Purpose:** Precise servo control for robot movements

### Motor Controller Board
- **Type:** Cytron MD13S Dual Channel
- **Voltage:** 12V DC operation
- **Current:** 13A continuous per channel
- **Control:** PWM input for speed control
- **Purpose:** Differential drive control for robot movement

### Power System
- **Main Battery:** 12V 10000mAh LiPo
- **Auxiliary Battery:** 5V 2000mAh for logic circuits
- **Protection:** Over-voltage, under-voltage, and current protection
- **Monitoring:** Battery voltage and current monitoring via I2C

## Technical Specifications

### Hardware Components

#### Servos (Servo 2040 Controller)
- **Number of Servos:** 9
- **Control Interface:** USB Serial (115200 baud)
- **Angular Range:** 0-180 degrees
- **Update Rate:** 100Hz with motion smoothing
- **Servo Assignments:**
  - Servos 1-2: Head tilt/rotation
  - Servos 3-4: Arm joints
  - Servos 5-6: Hand grippers
  - Servos 7-8: Eye mechanisms
  - Servo 9: Neck tilt

#### Drive System (Motor Controller)
- **Motors:** 2x 12V DC motors with encoders
- **Control Interface:** UART (ttyAMA2, 115200 baud)
- **Drive Configuration:** Differential drive
- **Wheel Base:** 200mm
- **Max Speed:** 1.0 m/s
- **Control Rate:** 50Hz

### Control Architecture
- **Input Device:** 8BitDo Lite 2 Controller
- **Control Mapping:**
  - Left Stick: Linear velocity (forward/backward)
  - Right Stick: Angular velocity (turning)
  - D-Pad: Head movement
  - Face Buttons: Arm control
  - Shoulder Buttons: Gripper control

### Communication
- **ROS2 Topics:**
  - `/cmd_vel`: Twist messages for movement
  - `/servo_positions`: Custom messages for servo control
  - `/joy`: Joystick input data
- **Update Rates:**
  - Joy Node: 100Hz
  - Motor Control: 50Hz
  - Servo Control: 100Hz

### Technical Reference

#### Servo Assignments and Ranges
| Servo ID | Function          | Min° | Max° | Default° | Speed°/s | Notes                    |
|----------|------------------|------|------|----------|----------|--------------------------|
| 1        | Head Tilt        | 0    | 180  | 90       | 60       | Mechanical limit 30-150° |
| 2        | Head Pan         | 0    | 180  | 90       | 60       | Full range available     |
| 3        | Left Arm         | 0    | 180  | 45       | 45       | Resting at 45°          |
| 4        | Right Arm        | 0    | 180  | 135      | 45       | Mirrored, resting 135°  |
| 5        | Left Gripper     | 0    | 180  | 90       | 90       | 0=closed, 180=open      |
| 6        | Right Gripper    | 0    | 180  | 90       | 90       | 0=closed, 180=open      |
| 7        | Left Eye         | 0    | 180  | 90       | 120      | 45-135° typical range   |
| 8        | Right Eye        | 0    | 180  | 90       | 120      | 45-135° typical range   |
| 9        | Neck Tilt        | 0    | 180  | 90       | 30       | Mechanical limit 60-120° |
| 10       | Head Left Raise  | 0    | 180  | 90       | 45       | Mechanical limit 45-135° |
| 11       | Head Right Raise | 0    | 180  | 90       | 45       | Mechanical limit 45-135° |
| 12       | Bay Door         | 0    | 180  | 0        | 30       | 0=closed, 180=open      |

#### Motor Specifications
| Parameter          | Value      | Notes                                    |
|-------------------|------------|------------------------------------------|
| Motor Driver      | Cytron MD13S | PWM control, 10-30V DC                |
| Operating Voltage | 12V DC     | Operating range 11-13V                   |
| Max Current       | 13A        | Per motor, continuous                    |
| Peak Current      | 30A        | Per motor, <1s                          |
| PWM Frequency     | 20kHz      | Internal frequency                      |
| Control Mode      | Sign-Magnitude PWM | DIR + PWM control               |
| Encoder PPR       | 360        | Pulses per revolution                    |
| Gear Ratio        | 20:1       | Planetary gearbox                        |
| No Load Speed     | 200 RPM    | At 12V                                   |
| Stall Torque      | 2.5 Nm     | At 12V                                  |
| Wheel Diameter    | 100mm      | Rubber tire                             |
| Control Frequency | 50Hz       | PID control loop                        |

#### Controller Button Mappings
| Button/Axis    | Function              | Value Range | Default |
|---------------|----------------------|-------------|---------|
| Left Stick Y  | Forward/Back Speed   | -1.0 to 1.0 | 0.0     |
| Right Stick X | Turn Rate            | -1.0 to 1.0 | 0.0     |
| D-Pad Up      | Head Tilt Up         | N/A         | N/A     |
| D-Pad Down    | Head Tilt Down       | N/A         | N/A     |
| D-Pad Left    | Head Turn Left       | N/A         | N/A     |
| D-Pad Right   | Head Turn Right      | N/A         | N/A     |
| A Button      | Arms Down            | N/A         | N/A     |
| B Button      | Arms Up              | N/A         | N/A     |
| X Button      | Arms Forward         | N/A         | N/A     |
| Y Button      | Arms Back            | N/A         | N/A     |
| L1 Button     | Close Grippers       | N/A         | N/A     |
| R1 Button     | Open Grippers        | N/A         | N/A     |
| L2 Button     | Eyes Left            | N/A         | N/A     |
| R2 Button     | Eyes Right           | N/A         | N/A     |

#### LED Status Indicators
| LED | Color  | Pattern              | Meaning                    |
|-----|--------|---------------------|----------------------------|
| 0   | Green  | Solid               | Power on, system ready     |
| 0   | Red    | Solid               | Error state                |
| 1   | Blue   | Blink               | Serial communication active|
| 2   | Yellow | Pulse               | Servo movement in progress |
| 3   | White  | Solid               | Bootloader mode active     |

#### CM4 Connected Peripherals
| Device          | Connection | Details                                    |
|-----------------|------------|-------------------------------------------|
| Laser           | GPIO23     | PWM controlled laser module               |
| Microphone      | I2S        | Adafruit SPH0645 I2S MEMS (24-bit PCM)  |
| Speaker         | I2S        | Audio output, controlled via ALSA         |
| Battery Display | I2C (0x3C) | ZJY-IPS130-V2.0 1.3" IPS LCD (240x240)  |

System: Ubuntu 24.04.x

#### Installation

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

### config.txt changes

```File: /boot/firmware/config.txt```

```
dtparam=audio=on
dtparam=i2c_arm=on
dtparam=spi=on

enable_uart=1
```

### cmdline.txt changes

```File: /boot/firmware/cmdline.txt```

remove:
```
console=serial0,115200
```

### Bluetooth

1. **Enable Bluetooth and Pair the Controller:**

    * **Install Bluetooth Packages:**

        ```bash
        sudo apt update
        sudo apt install bluetooth bluez blueman
        ```

    * **Start the Bluetooth Service:**

        ```bash
        sudo systemctl start bluetooth
        ```

    * **Put the Controller in Pairing Mode:**

        * Press and hold the **Start** button on your 8BitDo Lite 2 until the LEDs flash rapidly.
    * **Pair Using Bluetoothctl:**

        ```bash
        bluetoothctl
        ```

        * In the `bluetoothctl` prompt, enter:

            ```csharp
            agent on
            default-agent
            scan on
            ```

        * Wait for the controller to appear (e.g., `8BitDo Lite 2`), then note its MAC address.
        * Pair and connect:

            ```ruby
            pair XX:XX:XX:XX:XX:XX
            trust XX:XX:XX:XX:XX:XX
            connect XX:XX:XX:XX:XX:XX
            ```

        * Exit `bluetoothctl`:

            ```
            quit
            ```

2. **Verify the Controller is Detected:**

    * **Check for Joystick Devices:**

        ```bash
        ls /dev/input/js*
        ```

        * You should see `/dev/input/js0`.
    * **Install Joystick Utilities:**

        ```bash
        sudo apt install joystick
        ```

    * **Test the Controller:**

        ```bash
        jstest /dev/input/js0
        ```

        * Move the sticks and press buttons to ensure inputs are registered.


### Serial testing

8. **Test Access to the Serial Port:**

    * Try accessing the serial port directly:

        ```bash
        sudo apt install minicom
        minicom -D /dev/ttyAMA0
        ```

    * If the port opens without errors, it's accessible.
