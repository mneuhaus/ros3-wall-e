#!/usr/bin/env python3
"""
Script to flash Tracks firmware to the device.

Usage:
    python3 flash_firmware.py /dev/serial/by-id/usb-Raspberry_Pi_Pico_E6632891E3959D25-if00

This script uses picotool to load the UF2 firmware file onto the device.
Ensure that the UF2 firmware file is located at "firmware/build/tracks_firmware.uf2".
"""

import sys
import subprocess
import os


def flash_firmware(device_path: str) -> None:
    firmware_file = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "firmware", "build", "tracks_firmware.uf2")
    if not os.path.isfile(firmware_file):
        print(f"Firmware file not found: {firmware_file}")
        sys.exit(1)
    print(f"Flashing firmware from {firmware_file} to {device_path}...")

    # Determine iSerial: if device_path is not a /dev path, assume it's the iSerial directly.
    if device_path.startswith("/dev"):
        import re
        match = re.search(r'Pico_([A-Za-z0-9]+)-if', device_path)
        if not match:
            print("Could not determine iSerial from device path")
            sys.exit(1)
        iserial = match.group(1)
    else:
        iserial = device_path
    print(f"Using iSerial: {iserial}")
    
    # Locate the device file in /dev/serial/by-id that contains the iSerial.
    import glob
    serial_links = glob.glob("/dev/serial/by-id/*")
    device_found = None
    for link in serial_links:
        if iserial in link:
            device_found = link
            break
    if not device_found:
        print(f"Device with iSerial {iserial} not found in /dev/serial/by-id")
        sys.exit(1)
    
    # Resolve the real device and read its bus and device numbers from sysfs.
    real_device = os.path.realpath(device_found)
    tty_name = os.path.basename(real_device)
    sysfs_path = f"/sys/class/tty/{tty_name}/device"
    try:
        with open(os.path.join(sysfs_path, "busnum"), "r") as f:
            bus_num = f.read().strip()
        with open(os.path.join(sysfs_path, "devnum"), "r") as f:
            dev_num = f.read().strip()
    except Exception as e:
        print(f"Error reading sysfs for device {tty_name}: {e}")
        sys.exit(1)
    
    try:
        subprocess.run(["picotool", "reboot", '-u', '--bus', bus_num, '--address', dev_num, '-f'], check=True)
        subprocess.run(["picotool", "load", firmware_file, '-f', '--bus', bus_num, '--address', dev_num], check=True)
        subprocess.run(["picotool", "reboot", '-a', '--bus', bus_num, '--address', dev_num], check=True)
        print("Firmware flashed successfully!")
    except subprocess.CalledProcessError as e:
        print(f"Error flashing firmware: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 flash_firmware.py <device_path>")
        sys.exit(1)

    flash_firmware(sys.argv[1])
