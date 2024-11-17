
System: Ubuntu 24.04.x

#### Installation

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

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