import os
import subprocess
import sys
import time

def main():
    firmware_file = os.path.join(os.path.dirname(__file__), '../firmware/main.py')

    try:
        # Wait for the device to be in BOOTSEL mode
        print("Please put your Servo 2040 into BOOTSEL mode (hold BOOTSEL while pressing reset)...")
        time.sleep(2)

        # Upload the firmware using picotool
        print(f"Uploading {firmware_file}...")
        subprocess.run(
            ['picotool', 'load', '-x', firmware_file],
            check=True
        )
        print("Firmware uploaded successfully!")

        # Reboot the device
        print("Rebooting the device...")
        subprocess.run(['picotool', 'reboot'], check=True)
        print("Device rebooted successfully!")
    except subprocess.CalledProcessError as e:
        print(f"Error uploading firmware: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
