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

# Flash command using Python upload script
flash:
	@echo "Uploading code using Python script..."
	python3 -c "from servo_2040.scripts.upload_firmware import upload_main_py; upload_main_py('$(PYTHON_FILE)', '/dev/ttyAMA2')"
	@echo "File copied and board reset successfully."

# Help message
# Flash base firmware to Servo 2040
flash-servo2040:
	@echo "Flashing Servo 2040 base firmware..."
	python3 src/servo_2040/scripts/flash_firmware.py

# Upload code to Servo 2040
upload-code:
	@echo "Uploading code to Servo 2040..."
	python3 src/servo_2040/scripts/upload_main.py

help:
	@echo "Makefile Commands:"
	@echo "  make build          - Build the workspace"
	@echo "  make run            - Run the node"
	@echo "  make rebuild        - Clean and rebuild the workspace"
	@echo "  make clean          - Clean the workspace"
	@echo "  make flash-firmware - Flash fresh MicroPython firmware (BOOTSEL mode)"
	@echo "  make flash-servo2040 - Flash Servo 2040 base firmware (BOOTSEL mode)"
	@echo "  make upload-code    - Upload main.py code to Servo 2040"
	@echo "  make help           - Show this help message"

# Phony targets (not associated with files)
.PHONY: all build run rebuild clean help flash-firmware
