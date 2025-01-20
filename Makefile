# Makefile for ROS 2 Workspace Management

# Define your package name
PACKAGE_NAME=motor_controller

# Define the path to your ROS 2 workspace
ROS2_WS=$(shell pwd)

# Define the setup script
SETUP_SCRIPT=$(ROS2_WS)/install/setup.bash

# Default target
all: build

# Build the workspace
build:
	@echo "Building the workspace..."
	colcon build

# Source the workspace (not typically needed in Makefile)
#source:
#	@echo "Workspace sourced. Note: Sourcing in a Makefile may not affect your shell."

# Run the node
run:
	@echo "Running the node..."
	ros2 launch wall_e_launch all_nodes.launch.py

# Rebuild the workspace (clean and build)
rebuild: clean build

# Clean the workspace
clean:
	@echo "Cleaning the workspace..."
	rm -rf build/ install/ log/

# Variables
FIRMWARE_DIR = src/servo_2040/firmware
PYTHON_FILE = $(FIRMWARE_DIR)/main.py
UF2_FILE = $(FIRMWARE_DIR)/main.uf2
MICROPYTHON_URL = https://datasheets.raspberrypi.com/soft/micropython-firmware-pico-w-130623.uf2
MICROPYTHON_FILE = $(FIRMWARE_DIR)/micropython.uf2

# Flash command using picotool and rshell
flash:
	rshell cp $(PYTHON_FILE) /pyboard/main.py
	rshell "repl ~ import machine ~ machine.soft_reset() ~"
	@echo "File copied successfully. Please reset your board."

# Help message
# Download and flash MicroPython firmware (requires board in BOOTSEL mode)
flash-firmware:
	@echo "Downloading latest MicroPython firmware..."
	wget -O $(MICROPYTHON_FILE) $(MICROPYTHON_URL)
	@echo "Flashing MicroPython firmware..."
	cp $(MICROPYTHON_FILE) /media/$(USER)/RPI-RP2
	@echo "Firmware flashed successfully. Please reset your board."

help:
	@echo "Makefile Commands:"
	@echo "  make build          - Build the workspace"
	@echo "  make run            - Run the node"
	@echo "  make rebuild        - Clean and rebuild the workspace"
	@echo "  make clean          - Clean the workspace"
	@echo "  make flash-firmware - Flash fresh MicroPython firmware (BOOTSEL mode)"
	@echo "  make help           - Show this help message"

# Phony targets (not associated with files)
.PHONY: all build run rebuild clean help flash-firmware
