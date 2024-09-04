# moondawg-ros

## Overview

`moondawg-ros` is a ROS2 package designed to interface with SIU's Lunabotics robot, providing functionalities for Xbox controller input, serial communication, and diagnostics. The package includes several nodes and scripts to facilitate these operations.

## Prerequisites
- ROS2 Humble
This package is made for ROS2 humble, ensure it is installed and sourced on Ubuntu 22.04. (see https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- ROS2 packages
```bash
sudo apt install ros-humble-cv-bridge ros-humble-rosbridge-suite ros-humble-image-tools
```
- Python modules
```bash
sudo apt install python3-opencv python3-serial
```

## Running the package
1. Clone the repository:
```bash
git clone https://github.com/SIU-Robotics/moondawg-ros.git
cd moondawg-ros
```

2. Build the package:
```bash
colcon build
```

3. Source the workspace:
```bash
source install/setup.bash
```

4. Run:
```bash
ros2 launch moondawg moondawg.launch.py
```

## Nodes
### xbox_translator

Translates Xbox controller inputs to Arduino-parsable messages.

### serial_bridge
Sends strings via serial to configured port (default: /dev/ttyACM0).

### diagnostics
Provides diagnostic information about the package's status.

## Configuration

### Parameters

- **rate**: Baud rate for serial communication (default: 9600).
- **port**: Serial port (default: `/dev/ttyACM0`).
- **belt_speed_index**: The index into the belt speed array for which belt speed to use (default: 0).
- **wheel_full_speed**: The fastest forward speed of the wheel motors (default: 130).
- **wheel_full_stopped**: The value for a stopped wheel motor (default: 90).

## Web Interface

A simple web interface is provided to display Xbox controller inputs and diagnostics.
