import os
import subprocess
import sys

def main():
    # Update these variables as needed
    serial_port = '/dev/ttyACM0'
    firmware_file = os.path.join(os.path.dirname(__file__), '../firmware/main.py')

    # Check if the serial port exists
    if not os.path.exists(serial_port):
        print(f"Error: Serial port {serial_port} not found.")
        sys.exit(1)

    # Upload the firmware using ampy
    try:
        print(f"Uploading {firmware_file} to {serial_port}...")
        subprocess.run(
            ['ampy', '--port', serial_port, 'put', firmware_file],
            check=True
        )
        print("Firmware uploaded successfully!")

        # Restart the device
        print("Restarting the device...")
        subprocess.run(['ampy', '--port', serial_port, 'reset'], check=True)
        print("Device restarted successfully!")
    except subprocess.CalledProcessError as e:
        print(f"Error uploading firmware: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
