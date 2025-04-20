# MoonDawg ROS

## Overview

`moondawg-ros` is a ROS2 package for controlling SIU's Lunabotics mining robot. This package provides a comprehensive framework for robot control via gamepad input, hardware communication, and a web-based operator interface.

## Key Features

-   **Gamepad Control** - Xbox controller interface for all robot functions
-   **Web Interface** - Browser-based control panel with camera feed and diagnostics
-   **I2C Communication** - Interface with motor controllers and sensors
-   **Multiple Drive Modes** - Support for different steering configurations
-   **ROS2 Architecture** - Built on ROS2 Jazzy for modular, maintainable design

## Installation

### Prerequisites

-   Ubuntu 24.04 or compatible Linux distribution

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
```

### Web Interface

Access the control interface at: `http://[ROBOT_IP]:9090`

## System Architecture

### Nodes

-   **controller_parser** - Processes gamepad inputs and sends robot commands
-   **i2c_node** - Manages communication with motor controllers via I2C
-   **serial_node** - Optional interface for Arduino-based peripherals

## Troubleshooting

### Common Issues

-   **No controller detected**: Test with an online gamepad tester. If it appears, it is likely the index of the controller is wrong in **website/script.js**.
-   **I2C communication errors**: Run `sudo i2cdetect -y 1` (or other i2c interface) to ensure the ESP32s are visible.

## License

Apache License 2.0
