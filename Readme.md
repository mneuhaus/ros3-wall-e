# Wall-E Robot Project

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
