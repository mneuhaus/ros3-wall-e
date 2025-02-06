# Installation Guide

## Prerequisites

- Ubuntu 24.04 or newer
- ROS2 Jazzy
- Python 3.10+
- Git

## ROS2 Installation

1. Set up ROS2 repositories:
```bash
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
```

2. Add ROS2 GPG key:
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

3. Add repository:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

4. Install ROS2:
```bash
sudo apt update
sudo apt install ros-jazzy-desktop
```

## Project Setup

1. Create workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone repository:
```bash
git clone https://github.com/yourusername/wall-e.git .
```

3. Install dependencies:
```bash
cd ~/ros2_ws
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

4. Build project:
```bash
source /opt/ros/jazzy/setup.bash
colcon build
```

5. Source workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Additional Dependencies

1. Install Bluetooth packages:
```bash
sudo apt install bluetooth bluez blueman
```

2. Install audio dependencies:
```bash
sudo apt install python3-pygame
```

3. Install development tools:
```bash
sudo apt install cmake build-essential picotool
```

## Configuration

1. Add user to required groups:
```bash
sudo usermod -a -G dialout,gpio,i2c,spi $USER
```

2. Set up udev rules for USB devices:
```bash
sudo tee /etc/udev/rules.d/99-pico.rules << EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0003", MODE="0666"
EOF
```

3. Reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Testing Installation

1. Launch all nodes:
```bash
ros2 launch wall_e_launch all_nodes.launch.py
```

2. Test joystick:
```bash
ros2 run joy joy_node
```

3. Test web interface:
```bash
ros2 launch wall_e_web all_launch.py
```

## Troubleshooting

### Common Issues

1. Serial port permission denied:
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```

2. Joystick not detected:
   ```bash
   ls -l /dev/input/js0
   sudo chmod 666 /dev/input/js0
   ```

3. Audio issues:
   ```bash
   pulseaudio -k && pulseaudio --start
   ```

### Getting Help

- File issues on GitHub
- Check ROS2 forums
- Review system logs: `journalctl -xe`
