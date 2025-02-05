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

def get_usb_info(device_path: str) -> str:
    try:
        output = subprocess.check_output(["udevadm", "info", "--query=property", "--name", device_path], encoding='utf-8')
        bus_num = None
        dev_num = None
        for line in output.splitlines():
            if line.startswith("ID_USB_BUS="):
                bus_num = line.split("=")[1]
            if line.startswith("ID_USB_DEVICE="):
                dev_num = line.split("=")[1]
        if bus_num and dev_num:
            return f"Bus {bus_num} Device {dev_num}"
        else:
            return "Unknown USB bus/device"
    except Exception as e:
        return f"Error determining USB bus/device: {e}"

def flash_firmware(device_id: str) -> None:
    firmware_file = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "firmware", "build", "tracks_firmware.uf2")
    if not os.path.isfile(firmware_file):
        print(f"Firmware file not found: {firmware_file}")
        sys.exit(1)
    print(f"Flashing firmware from {firmware_file} to {device_id}...")
    usb_info = get_usb_info(device_id)
    print(f"Detected USB device: {usb_info}")
    try:
        #subprocess.run(["picotool", "reboot", '-f', '-u', '--ser', device_id], check=True)
        subprocess.run(["picotool", "load", firmware_file, '-f', '--bus', "001", '--address', "017"], check=True)
        subprocess.run(["picotool", "reboot", '-a', '--bus', "001", '--address', "017"], check=True)
        print("Firmware flashed successfully!")
    except subprocess.CalledProcessError as e:
        print(f"Error flashing firmware: {e}")
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 flash_firmware.py <device_path>")
        sys.exit(1)
    flash_firmware(sys.argv[1])
