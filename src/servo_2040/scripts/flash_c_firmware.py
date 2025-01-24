#!/usr/bin/env python3
"""
Flash C firmware to Servo 2040 board.
Waits for BOOTSEL mode and copies UF2 file.
"""

import os
import sys
import time
import shutil
import glob

def find_pico_drive():
    """Find the mounted Pico drive."""
    # Common mount points for the Pico in bootloader mode
    possible_paths = [
        "/media/*/RPI-RP2",      # Linux
        "/Volumes/RPI-RP2",      # macOS
        "/run/media/*/RPI-RP2",  # Some Linux distros
    ]
    
    for pattern in possible_paths:
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return None

def wait_for_bootsel():
    """Wait for the Pico to appear in BOOTSEL mode."""
    print("\nTo enter BOOTSEL mode:")
    print("1. Hold the BOOTSEL button (small button near USB)")
    print("2. While holding BOOTSEL, press and release RESET (or unplug/plug USB)")
    print("3. Keep holding BOOTSEL for 1 more second, then release")
    print("\nWaiting for device...")
    
    while True:
        drive = find_pico_drive()
        if drive:
            print(f"Device detected at {drive}")
            return drive
        time.sleep(1)

def flash_firmware(firmware_path):
    """Flash the C firmware to the Pico."""
    if not os.path.exists(firmware_path):
        print(f"Error: Firmware not found at {firmware_path}")
        print("Did you build the C firmware first? Try 'cd src/servo_2040/firmware/c_version && mkdir build && cd build && cmake .. && make'")
        return False
        
    drive = wait_for_bootsel()
    if not drive:
        print("Error: Could not find Pico in BOOTSEL mode")
        return False
        
    print(f"Copying firmware to {drive}...")
    try:
        shutil.copy2(firmware_path, drive)
        print("Firmware flashed successfully!")
        print("Device will restart automatically.")
        return True
    except Exception as e:
        print(f"Error flashing firmware: {e}")
        return False

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    firmware_path = os.path.join(script_dir, '../firmware/c_version/build/servo_controller.uf2')

    if not flash_firmware(firmware_path):
        sys.exit(1)

if __name__ == '__main__':
    main()
