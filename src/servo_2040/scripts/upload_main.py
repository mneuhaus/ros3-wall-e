import os
import subprocess
import sys
import time
import serial
from pathlib import Path
import sys
import os

# Add the parent directory to the Python path to find the protocol module
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from servo_2040.protocol import Protocol, CommandType

def send_terminate_command(port='/dev/ttyAMA2', baudrate=115200):
    """Send termination command to the device."""
    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print("Sending termination command...")
            ser.write(Protocol.encode_terminate())
            time.sleep(0.5)  # Give device time to process
            response = ser.readline()
            if response:
                print(f"Device response: {response.decode('utf-8').strip()}")
            return True
    except serial.SerialException as e:
        print(f"Failed to send termination command: {e}")
        return False

def upload_main_py(main_py_path, port='/dev/ttyAMA2'):
    """Upload the main.py file to the Servo 2040 using rshell."""
    max_attempts = 3
    for attempt in range(max_attempts):
        try:
            print(f"Attempting to upload {main_py_path} (attempt {attempt + 1}/{max_attempts})...")
            subprocess.run(['rshell', '-p', port, 'cp', main_py_path, '/pyboard/main.py'], check=True)
            subprocess.run(['rshell', '-p', port, 'repl ~ import machine ~ machine.soft_reset() ~'], check=True)
            print("main.py uploaded successfully!")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Attempt {attempt + 1} failed: {e}")
            if attempt < max_attempts - 1:
                print("Retrying in 2 seconds...")
                time.sleep(2)
            else:
                print("Failed to upload main.py after multiple attempts")
                return False

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    main_py_path = os.path.join(script_dir, '../firmware/main.py')

    try:
        # First try to terminate gracefully
        if not send_terminate_command():
            print("Warning: Could not send termination command. Proceeding with upload anyway...")
            time.sleep(1)  # Brief pause before proceeding
        
        print("\nUploading main.py to device...")
        if not upload_main_py(main_py_path):
            sys.exit(1)

        print("\nAll done! Your code has been uploaded.")
        print("The board will automatically restart and run the new code.")

    except Exception as e:
        print(f"Error during upload: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
