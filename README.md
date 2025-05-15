# MoonDawg ROS

## Overview

`moondawg-ros` is a ROS2 package for controlling SIU's Lunabotics mining robot. This package provides a comprehensive framework for robot control via gamepad input, hardware communication, and a web-based operator interface.

## Installation

### Prerequisites

- Ubuntu 24.04 or compatible Linux distribution

### Quick Setup

1. **Set up ROS2 sources**:

   ```bash
   sudo bash scripts/add_ros_sources.sh
   ```

2. **Install ROS2 Jazzy**:

   ```bash
   sudo apt install ros-jazzy-ros-base
   ```

3. **Install dependencies**:

   ```bash
   sudo bash scripts/install_dependencies.sh
   ```

4. **Build the package**:

   ```bash
   git clone https://github.com/SIU-Robotics/moondawg-ros.git
   cd moondawg-ros
   colcon build --symlink-install
   source install/setup.bash
   ```

5. **Configure autostart** (optional, script is made for Jazzy):
   ```bash
   sudo bash scripts/install_autostart.sh
   ```

## Usage

### Launch the Robot System

```bash
ros2 launch moondawg moondawg.launch.py
```

### Launch Options

```bash
# Disable camera
ros2 launch moondawg moondawg.launch.py enable_camera:=false

# Change I2C bus
ros2 launch moondawg moondawg.launch.py i2c_bus:=2

# Enable debug logging
ros2 launch moondawg moondawg.launch.py debug:=true

# Disable RealSense cameras
ros2 launch moondawg moondawg.launch.py enable_realsense:=false

# Specify RealSense camera serial numbers (if you have multiple)
ros2 launch moondawg moondawg.launch.py realsense1_serial:=123456789012 realsense2_serial:=987654321098

# QR code testing:
- Terminal 1: ros2 run v4l2_camera_node --ros-args -p video_device:=/dev/video0
- Terminal 2: ros2 run moondawg_qr_reader qr_code_reader

```

### Web Interface

Access the control interface at: `http://[ROBOT_IP]:9090`

## System Architecture

### Nodes

- **controller_parser** - Processes gamepad inputs and sends robot commands
- **i2c_node** - Manages communication with motor controllers via I2C
- **serial_node** - Optional interface for Arduino-based peripherals
- **cam2image** - Standard USB camera interface
- **realsense2_camera** - Interfaces with Intel RealSense depth cameras (D435/D455)

## Troubleshooting

### Common Issues

- **No controller detected**: Test with an online gamepad tester. If it appears, it is likely the index of the controller is wrong in **website/script.js**.
- **I2C communication errors**: Run `sudo i2cdetect -y 1` (or other i2c interface) to ensure the ESP32s are visible.
- **RealSense cameras not detected**: Run `rs-enumerate-devices` to list connected RealSense devices and verify their serial numbers.
- **RealSense permission issues**: Make sure the udev rules are correctly installed by running `sudo bash scripts/install_dependencies.sh`.

## License

Apache License 2.0
