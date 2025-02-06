#!/usr/bin/env python3
"""
Flash firmware to Servo 2040 board using picotool.
"""

import os
import subprocess
import sys
import time

def try_remote_bootloader():
    """Try to remotely trigger bootloader mode."""
    try:
        # Check if device is currently running
        result = subprocess.run(['picotool', 'info'], 
                              capture_output=True, 
                              text=True)
        if result.returncode == 0:
            print("Device detected, attempting remote bootloader entry...")
            # Reboot into USB boot mode
            subprocess.run(['picotool', 'reboot', '-u', '-f'], check=True)
            time.sleep(2)  # Wait for device to restart
            return True
    except subprocess.CalledProcessError:
        pass
    return False

def wait_for_bootsel():
    """Wait for the Pico to be detected in BOOTSEL mode."""
    print("Waiting for device in bootloader mode...")
    
    # First try remote bootloader entry
    if try_remote_bootloader():
        # Verify device is in bootloader mode
        for _ in range(5):  # Try for 10 seconds
            try:
                result = subprocess.run(['picotool', 'info'], 
                                     capture_output=True, 
                                     text=True)
                if result.returncode == 0:
                    print("Device detected in bootloader mode!")
                    return True
            except subprocess.CalledProcessError:
                pass
            time.sleep(2)
    
    # If remote entry failed, ask for manual intervention
    print("\nAutomatic bootloader entry failed.")
    print("Please enter BOOTSEL mode manually:")
    print("1. Hold the BOOTSEL button (small button near USB)")
    print("2. While holding BOOTSEL, press and release RESET (or unplug/plug USB)")
    print("3. Keep holding BOOTSEL for 1 more second, then release")
    
    while True:
        try:
            result = subprocess.run(['picotool', 'info'], 
                                 capture_output=True, 
                                 text=True)
            if result.returncode == 0:
                print("Device detected in bootloader mode!")
                return True
        except subprocess.CalledProcessError:
            pass
        time.sleep(2)

def flash_firmware(firmware_path):
    """Flash firmware using picotool."""
    if not os.path.exists(firmware_path):
        print(f"Error: Firmware not found at {firmware_path}")
        print("Did you build the firmware first? Try:")
        print("cd src/servo_2040/firmware && mkdir build && cd build && cmake .. && make")
        return False
    
    print(f"Loading firmware {firmware_path}...")
    subprocess.run(['picotool', 'load', '-f', firmware_path], check=True)
    
    print("Rebooting device...")
    subprocess.run(['picotool', 'reboot', '-f'], check=True)
    return True

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    firmware_path = os.path.join(script_dir, '../firmware/build/servo_controller.uf2')

    try:
        if wait_for_bootsel():
            if flash_firmware(firmware_path):
                print("\nFirmware flashed successfully!")
            else:
                sys.exit(1)
    except Exception as e:
        print(f"Error during firmware flash: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
