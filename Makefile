# Makefile for ROS 2 Workspace Management

# Define your package name
PACKAGE_NAME=motor_controller

# Define the path to your ROS 2 workspace
ROS2_WS=$(shell pwd)

# Define the setup script
SETUP_SCRIPT=$(ROS2_WS)/install/setup.bash

# Pico SDK setup
PICO_SDK_PATH ?= $(HOME)/pico-sdk
ARM_GCC_PATH ?= /usr/local

# Default target
all: build

# Setup ARM toolchain
pico/toolchain:
	@if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then \
		echo "Installing ARM toolchain..."; \
		brew install --cask gcc-arm-embedded; \
	else \
		echo "ARM toolchain already installed"; \
	fi

# Setup Pico SDK
pico/setup:
	@if [ ! -d "$(PICO_SDK_PATH)" ]; then \
		echo "Cloning Pico SDK..."; \
		git clone https://github.com/raspberrypi/pico-sdk.git $(PICO_SDK_PATH); \
		cd $(PICO_SDK_PATH) && git submodule update --init; \
	else \
		echo "Updating Pico SDK..."; \
		cd $(PICO_SDK_PATH) && git pull && git submodule update --init; \
	fi
	@echo "export PICO_SDK_PATH=$(PICO_SDK_PATH)" >> ~/.zshrc

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
TRACKS_FIRMWARE_DIR = src/tracks/firmware

# Build firmware for Servo 2040
servo2040/build:
	@echo "Building Servo 2040 firmware..."
	mkdir -p $(FIRMWARE_DIR)/build
	cd $(FIRMWARE_DIR)/build && cmake .. && make

# Flash firmware to Servo 2040
servo2040/flash:
	@echo "Flashing Servo 2040 firmware..."
	python3 src/servo_2040/scripts/flash_firmware.py

# Build and flash Servo 2040 firmware
servo2040/update: servo2040/build servo2040/flash
	@echo "Servo 2040 firmware updated"

# Build firmware for Eyes
eyes/build: pico/toolchain
	@echo "Building Eyes firmware..."
	mkdir -p $(EYES_FIRMWARE_DIR)/build
	cd $(EYES_FIRMWARE_DIR)/build && cmake .. && make

# Flash firmware to Eyes
eyes/flash:
	@echo "Flashing Eyes firmware..."
	python3 src/eyes/scripts/flash_firmware.py

# Build and flash Eyes firmware
eyes/update: eyes/build eyes/flash
	@echo "Eyes firmware updated"

# Build firmware for Tracks
tracks/build:
	@echo "Building Tracks firmware..."
	mkdir -p $(TRACKS_FIRMWARE_DIR)/build
	cd $(TRACKS_FIRMWARE_DIR)/build && cmake .. && make

# Flash firmware to Tracks
tracks/flash:
	@echo "Flashing Tracks firmware..."
	python3 src/tracks/scripts/flash_firmware.py /dev/serial/by-id/usb-Raspberry_Pi_Pico_E6612483CB1A9621-if00

# Build and flash Tracks firmware
tracks/update: tracks/build tracks/flash
	@echo "Tracks firmware updated"

tracks/monitor:
	sudo docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:jazzy serial --dev /dev/serial/by-id/usb-Raspberry_Pi_Pico_E6612483CB1A9621-if00 -b 115200

help:
	@echo "Makefile Commands:"
	@echo "  make ros2/build    - Build the ROS2 workspace"
	@echo "  make ros2/run      - Run the ROS2 node"
	@echo "  make ros2/rebuild  - Clean and rebuild the ROS2 workspace"
	@echo "  make ros2/clean    - Clean the ROS2 workspace"
	@echo "  make pico/setup    - Setup/update Pico SDK"
	@echo "  make servo2040/build - Build the Servo 2040 firmware"
	@echo "  make servo2040/flash - Flash firmware to Servo 2040 (BOOTSEL mode)"
	@echo "  make eyes/build     - Build the Eyes firmware"
	@echo "  make eyes/flash     - Flash firmware to Eyes (BOOTSEL mode)"
	@echo "  make help           - Show this help message"

# Phony targets (not associated with files)
.PHONY: all ros2/build ros2/run ros2/rebuild ros2/clean help pico/setup servo2040/flash servo2040/build eyes/build eyes/flash
