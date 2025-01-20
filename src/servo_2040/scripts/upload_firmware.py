import os
import subprocess
import sys
import time
import urllib.request
import shutil

FIRMWARE_URL = "https://github.com/pimoroni/pimoroni-pico/releases/download/v1.23.0-1/pico-v1.23.0-1-pimoroni-micropython.uf2"

def download_firmware(firmware_path):
    """Download the Pimoroni custom firmware if not present."""
    if not os.path.exists(firmware_path):
        print(f"Downloading Pimoroni firmware to {firmware_path}...")
        os.makedirs(os.path.dirname(firmware_path), exist_ok=True)
        urllib.request.urlretrieve(FIRMWARE_URL, firmware_path)
        print("Firmware downloaded successfully!")

def find_pico_mount():
    """Find the RPI Pico mount point."""
    possible_paths = [
        "/media/pi/RPI-RP2",
        "/media/RPI-RP2",
        "/run/media/pi/RPI-RP2",
        "/run/media/RPI-RP2",
        "/Volumes/RPI-RP2"  # For macOS
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            return path
    return None

def wait_for_bootsel():
    """Wait for the RPI Pico to appear in BOOTSEL mode."""
    print("\nTo enter BOOTSEL mode:")
    print("1. Hold the BOOTSEL button (small button near USB)")
    print("2. While holding BOOTSEL, press and release RESET (or unplug/plug USB)")
    print("3. Keep holding BOOTSEL for 1 more second, then release")
    print("\nWaiting for device...")
    
    while True:
        pico_path = find_pico_mount()
        if pico_path:
            print(f"Device detected in BOOTSEL mode at {pico_path}!")
            time.sleep(1)  # Give the system a moment to fully mount the drive
            return pico_path
        time.sleep(2)

def flash_firmware(firmware_path):
    """Copy the UF2 firmware to the Pico."""
    pico_path = find_pico_mount()
    if not pico_path:
        raise RuntimeError("Cannot find RPI-RP2 mount point. Is the device in BOOTSEL mode?")
    print(f"Flashing firmware {firmware_path}...")
    shutil.copy2(firmware_path, pico_path)
    print("Firmware flashed! Waiting for device to restart...")
    time.sleep(5)  # Give the device time to restart

def upload_main_py(main_py_path):
    """Upload the main.py file using rshell."""
    max_attempts = 3
    for attempt in range(max_attempts):
        try:
            print(f"Attempting to upload {main_py_path} (attempt {attempt + 1}/{max_attempts})...")
            subprocess.run(['rshell', 'cp', main_py_path, '/pyboard/main.py'], check=True)
            subprocess.run(['rshell', 'repl ~ import machine ~ machine.soft_reset() ~'], check=True)
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
    # Setup paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    firmware_path = os.path.join(script_dir, '../firmware/pimoroni-micropython.uf2')
    main_py_path = os.path.join(script_dir, '../firmware/main.py')

    try:
        # Step 1: Download firmware if needed
        download_firmware(firmware_path)

        # Step 2: Flash UF2 firmware
        print("\nStep 1: Flashing Pimoroni firmware")
        pico_path = wait_for_bootsel()
        flash_firmware(firmware_path)

        # Step 3: Upload main.py
        print("\nStep 2: Uploading main.py")
        if not upload_main_py(main_py_path):
            sys.exit(1)

        print("\nAll done! Your Servo 2040 is ready to use.")
        print("The board will automatically restart and run the new firmware.")

    except Exception as e:
        print(f"Error during upload process: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
