# Makefile for ROS 2 Workspace Management

# Define your package name
PACKAGE_NAME=motor_controller

# Define the path to your ROS 2 workspace
ROS2_WS=$(shell pwd)

# Define the setup script
SETUP_SCRIPT=$(ROS2_WS)/install/setup.bash

# Default target
all: build

# Build the ROS2 workspace
ros2/build:
	@echo "Building the ROS2 workspace..."
	colcon build

# Run the ROS2 node
ros2/run:
	@echo "Running the ROS2 node..."
	ros2 launch wall_e_launch all_nodes.launch.py

# Rebuild the ROS2 workspace
ros2/rebuild: ros2/clean ros2/build

# Clean the ROS2 workspace
ros2/clean:
	@echo "Cleaning the ROS2 workspace..."
	rm -rf build/ install/ log/

# Variables
FIRMWARE_DIR = src/servo_2040/firmware
EYES_FIRMWARE_DIR = src/eyes/firmware

# Build firmware for Servo 2040
servo2040/build:
	@echo "Building Servo 2040 firmware..."
	mkdir -p $(FIRMWARE_DIR)/build
	cd $(FIRMWARE_DIR)/build && cmake .. && make

# Flash firmware to Servo 2040
servo2040/flash:
	@echo "Flashing Servo 2040 firmware..."
	python3 src/servo_2040/scripts/flash_firmware.py

# Build firmware for Eyes
eyes/build:
	@echo "Building Eyes firmware..."
	mkdir -p $(EYES_FIRMWARE_DIR)/build
	cd $(EYES_FIRMWARE_DIR)/build && cmake .. && make

# Flash firmware to Eyes
eyes/flash:
	@echo "Flashing Eyes firmware..."
	python3 src/eyes/scripts/flash_firmware.py

help:
	@echo "Makefile Commands:"
	@echo "  make ros2/build    - Build the ROS2 workspace"
	@echo "  make ros2/run      - Run the ROS2 node"
	@echo "  make ros2/rebuild  - Clean and rebuild the ROS2 workspace"
	@echo "  make ros2/clean    - Clean the ROS2 workspace"
	@echo "  make servo2040/build - Build the Servo 2040 firmware"
	@echo "  make servo2040/flash - Flash firmware to Servo 2040 (BOOTSEL mode)"
	@echo "  make eyes/build     - Build the Eyes firmware"
	@echo "  make eyes/flash     - Flash firmware to Eyes (BOOTSEL mode)"
	@echo "  make help           - Show this help message"

# Phony targets (not associated with files)
.PHONY: all ros2/build ros2/run ros2/rebuild ros2/clean help servo2040/flash servo2040/build eyes/build eyes/flash
