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

        # Upload the firmware using rshell
        print(f"Uploading {firmware_file}...")
        subprocess.run(
            ['rshell', 'cp', firmware_file, '/pyboard/main.py'],
            check=True
        )
        subprocess.run(
            ['rshell', 'repl ~ import machine ~ machine.soft_reset() ~'],
            check=True
        )
        print("Firmware uploaded successfully!")
        print("Please reset your board to start running the new firmware.")
    except subprocess.CalledProcessError as e:
        print(f"Error uploading firmware: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
