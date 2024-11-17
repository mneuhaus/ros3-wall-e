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

# Help message
help:
	@echo "Makefile Commands:"
	@echo "  make build     - Build the workspace"
	@echo "  make run       - Run the node"
	@echo "  make rebuild   - Clean and rebuild the workspace"
	@echo "  make clean     - Clean the workspace"
	@echo "  make help      - Show this help message"

# Phony targets (not associated with files)
.PHONY: all build run rebuild clean help
