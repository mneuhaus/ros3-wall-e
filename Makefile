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
	@echo "=== Starting ROS2 workspace build ==="
	@echo "Building packages..."
	VERBOSE=1 colcon build --event-handlers console_direct+
	@echo "=== Build complete ==="

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

# Flash firmware to Servo 2040
flash-firmware:
	@echo "=== Starting firmware flash process ==="
	@echo "Running flash script..."
	PYTHONUNBUFFERED=1 python3 -u src/servo_2040/scripts/flash_firmware.py
	@echo "=== Flash process complete ==="

help:
	@echo "Makefile Commands:"
	@echo "  make build          - Build the workspace"
	@echo "  make run            - Run the node"
	@echo "  make rebuild        - Clean and rebuild the workspace"
	@echo "  make clean          - Clean the workspace"
	@echo "  make flash-firmware - Flash firmware to Servo 2040 (BOOTSEL mode)"
	@echo "  make help           - Show this help message"

# Phony targets (not associated with files)
.PHONY: all build run rebuild clean help flash-firmware flash-c-firmware
