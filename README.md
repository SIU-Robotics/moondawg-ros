# moondawg-ros

## Overview

`moondawg-ros` is a ROS2 package designed to interface with SIU's Lunabotics robot, providing functionalities for Xbox controller input, serial communication, and diagnostics. The package includes several nodes and scripts to facilitate these operations.


## Prerequisites
### ROS2 Humble
This package is made for ROS2 humble, ensure it is installed and sourced on Ubuntu 22.04.
### Misc packages
```bash
sudo apt install python3-opencv ros-humble-rosbridge-suite 
```

## Installation
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

## Nodes

### xbox_translator

Translates Xbox controller inputs to Arduino-parsable messages.

### serial_bridge

Bridges serial communication between the Raspberry Pi and the Arduino.

### diagnostics

Provides diagnostic information about the package's status.

## Launch Files

### moondawg.launch.py

Launches all the necessary nodes for the `moondawg-ros` package.


## Configuration

### Parameters

- **rate**: Baud rate for serial communication (default: 9600).
- **port**: Serial port (default: `/dev/ttyACM0`).
- **belt_speed_index**: The index into the belt speed array for which belt speed to use (default: 0).
- **wheel_full_speed**: The fastest forward speed of the wheel motors (default: 130).
- **wheel_full_stopped**: The value for a stopped wheel motor (default: 90).

## Web Interface

A simple web interface is provided to display Xbox controller inputs and diagnostics.
