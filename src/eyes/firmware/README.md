# Eyes Display Firmware

This firmware drives the GC9A01A round LCD display for Wall-E's eyes.

## Features

- 240x240 round LCD display support
- Image display from flash memory
- Serial command interface
- DMA-based display updates

## Building

Make sure you have the Pico SDK and build tools installed. Then:

```bash
make eyes/build
```

Or manually:

```bash
cd src/eyes/firmware
mkdir build
cd build
cmake ..
make
```

## Flashing

The easiest way is to use the make target:

```bash
make eyes/flash
```

This will:
1. Try to automatically put the device in bootloader mode
2. If that fails, guide you through manual bootloader entry
3. Flash the firmware
4. Reboot the device

### Manual Flashing

1. Hold the BOOTSEL button
2. Plug in the USB cable
3. Release BOOTSEL after 1 second
4. The device should appear as a USB mass storage device
5. Copy build/eyes.uf2 to the device
6. It will automatically reboot with the new firmware

## Serial Protocol

The firmware accepts commands over USB serial at 115200 baud:

- `PING` - Check if device is responding
- `INIT_GPIO PIN=n MODE=m` - Initialize GPIO pin
- `DRAW_IMAGE X=x Y=y WIDTH=w HEIGHT=h` - Draw image at position
- `RESET_FIRMWARE` - Enter bootloader mode

Responses are in format: `COMMAND: STATUS MESSAGE`

## Development

See CONVENTIONS.md for coding standards and development practices.
