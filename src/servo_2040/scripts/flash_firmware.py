import os
import subprocess
import sys
import time
import urllib.request

FIRMWARE_URL = "https://github.com/pimoroni/pimoroni-pico/releases/download/v1.23.0-1/pico-v1.23.0-1-pimoroni-micropython.uf2"

def download_firmware(firmware_path):
    """Download the Pimoroni custom firmware if not present."""
    if not os.path.exists(firmware_path):
        print(f"Downloading Pimoroni firmware to {firmware_path}...")
        os.makedirs(os.path.dirname(firmware_path), exist_ok=True)
        urllib.request.urlretrieve(FIRMWARE_URL, firmware_path)
        print("Firmware downloaded successfully!")

def wait_for_bootsel():
    """Wait for the Pico to be detected in BOOTSEL mode."""
    print("\nTo enter BOOTSEL mode:")
    print("1. Hold the BOOTSEL button (small button near USB)")
    print("2. While holding BOOTSEL, press and release RESET (or unplug/plug USB)")
    print("3. Keep holding BOOTSEL for 1 more second, then release")
    print("\nWaiting for device...")
    
    while True:
        try:
            result = subprocess.run(['picotool', 'info'], 
                                 capture_output=True, 
                                 text=True)
            if result.returncode == 0:
                print("Device detected in BOOTSEL mode!")
                return
        except subprocess.CalledProcessError:
            pass
        time.sleep(2)

def flash_firmware(firmware_path):
    """Flash firmware using picotool."""
    print(f"Erasing flash...")
    subprocess.run(['picotool', 'erase', '-f'], check=True)
    
    print(f"Loading firmware {firmware_path}...")
    subprocess.run(['picotool', 'load', '-f', firmware_path], check=True)
    
    print("Rebooting device...")
    subprocess.run(['picotool', 'reboot', '-f'], check=True)
    print("Firmware flashed! Device will restart.")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    firmware_path = os.path.join(script_dir, '../firmware/pimoroni-micropython.uf2')

    try:
        download_firmware(firmware_path)
        wait_for_bootsel()
        flash_firmware(firmware_path)
        print("\nBase firmware flashed successfully!")
        print("You can now run 'make upload-code' to upload the main.py file.")

    except Exception as e:
        print(f"Error during firmware flash: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
