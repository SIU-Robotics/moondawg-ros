# moondawg-ros

## Overview

`moondawg-ros` is a ROS2 package designed to interface with SIU's Lunabotics robot, providing functionalities for Xbox controller input, serial communication, and diagnostics. The package includes several nodes and scripts to facilitate these operations.

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/moondawg-ros.git
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

Translates Xbox controller inputs to ROS messages.

### serial_bridge

Bridges serial communication between the Raspberry Pi and connected devices.

### diagnostics

Provides diagnostic information about the system's status.

## Launch Files

### moondawg.launch.py

Launches all the necessary nodes for the `moondawg-ros` package.


## Configuration

### Parameters

- **rate**: Baud rate for serial communication (default: 9600).
- **port**: Serial port (default: `/dev/ttyACM0`).

## Web Interface

A simple web interface is provided to display Xbox controller inputs and diagnostics.
