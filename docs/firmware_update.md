# Firmware Update Guide

## Overview

The Wall-E robot uses two RP2040-based microcontrollers:
1. Pimoroni Servo 2040 - Servo and motor control
2. Custom Eyes Board - Display control

Both use the same update procedure but different firmware files.

## Prerequisites

1. Install tools:
```bash
sudo apt install picotool cmake build-essential
```

2. Set up udev rules:
```bash
sudo tee /etc/udev/rules.d/99-pico.rules << EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="0666"
EOF
```

3. Reload rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Building Firmware

### Servo 2040 Firmware

1. Build:
```bash
cd ~/ros2_ws
make servo2040/build
```

2. Flash:
```bash
make servo2040/flash
```

### Eyes Firmware

1. Build:
```bash
cd ~/ros2_ws
make eyes/build
```

2. Flash:
```bash
make eyes/flash
```

## Manual Update Procedure

If automatic flashing fails:

1. Enter BOOTSEL mode:
   - Hold BOOTSEL button
   - Press RESET (or reconnect USB)
   - Release BOOTSEL after 1 second

2. Device appears as USB drive

3. Copy appropriate .uf2 file:
   - Servo: `build/servo_controller.uf2`
   - Eyes: `build/eyes.uf2`

4. Device automatically reboots

## Verification

### Servo 2040

1. Check serial connection:
```bash
ls -l /dev/ttyACM0
```

2. Test communication:
```bash
ros2 run servo_2040 servo_2040
```

3. Verify response:
```
[INFO] [servo_2040]: Connected to /dev/ttyACM0
```

### Eyes Display

1. Check serial connection:
```bash
ls -l /dev/ttyACM1
```

2. Test display:
```bash
ros2 run eyes eyes_node
```

3. Verify response:
```
[INFO] [eyes_node]: Display initialized
```

## Troubleshooting

### Common Issues

1. Device not detected:
   - Check USB connection
   - Try different USB port
   - Verify udev rules

2. Build fails:
   - Update SDK: `cd $PICO_SDK_PATH && git pull`
   - Clean build: `rm -rf build`
   - Rebuild toolchain

3. Flash fails:
   - Use manual BOOTSEL method
   - Check USB drive permissions
   - Verify .uf2 file exists

### Recovery

If device becomes unresponsive:

1. Double-tap RESET while holding BOOTSEL
2. Device appears as USB drive
3. Flash known working firmware
4. If still fails, use picotool:
```bash
picotool info
picotool reboot -f -u
```

## Version History

Document firmware versions and changes:

```
servo_2040_v1.0.0:
- Initial release
- Basic servo control
- PWM motor support

eyes_v1.0.0:
- Initial release
- GC9A01A display support
- Basic animation
```

## Safety Notes

1. Never disconnect during update
2. Keep known working firmware backup
3. Test in isolation before full deployment
4. Document all changes and versions
