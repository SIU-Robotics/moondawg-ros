# moondawg-ros

## Overview

`moondawg-ros` is a ROS2 package for controlling SIU's Lunabotics robot. This package provides a robust framework for managing robot controls via Xbox controller input, handling I2C and serial communication with onboard hardware, and providing a web interface for remote operation.

## Features

- **Controller Interface**: Process Xbox controller inputs for robot steering and function control
- **Four-Wheel Steering**: Support for different steering modes (crab and rotate modes)
- **I2C Communication**: Communicate with motor controllers and sensors via I2C
- **Serial Communication**: Optional serial interface for Arduino-based peripherals
- **Web Interface**: Browser-based control panel with video feed
- **Autonomous Operation**: Support for autonomous digging sequences
- **Diagnostics**: Comprehensive diagnostic information for debugging

## Installation

### Prerequisites

- ROS2 (Humble or later recommended)
- Ubuntu 22.04 or compatible Linux distribution

### Required ROS2 Packages

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-rosbridge-suite ros-$ROS_DISTRO-image-tools
```

### Required Python Packages

```bash
sudo apt install python3-opencv python3-serial
pip install smbus2
```

### Building the Package

```bash
# Clone the repository
git clone https://github.com/SIU-Robotics/moondawg-ros.git
cd moondawg-ros

# Build the package
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Usage

### Starting the Robot Control System

Launch the complete system with default settings:

```bash
ros2 launch moondawg moondawg.launch.py
```

### Launch Options

The launch file supports various parameters to customize operation:

```bash
# Disable the camera node
ros2 launch moondawg moondawg.launch.py enable_camera:=false

# Change the serial port
ros2 launch moondawg moondawg.launch.py serial_port:=/dev/ttyACM1

# Change the I2C bus ID
ros2 launch moondawg moondawg.launch.py i2c_bus:=2

# Enable debug mode for more detailed logging
ros2 launch moondawg moondawg.launch.py debug:=true
```

### Web Interface

The web interface is accessible at http://[ROBOT_IP]:9090 when the rosbridge server is running.

## Node Configuration

Configuration parameters can be adjusted in the YAML file at `config/moondawg_params.yaml`.

### Controller Parser Node

The controller parser node translates Xbox controller inputs into robot commands:

- **Driving Modes**:

  - **Rotate Mode**: Enables point turns with differential wheel speed
  - **Crab Mode**: All wheels point in the same direction for lateral movement

- **Controller Mappings**:
  - **Left Joystick**: Main movement control
  - **Right Trigger**: Forward movement
  - **Left Trigger**: Backward movement
  - **Left Bumper**: Belt control
  - **Right Bumper**: Toggle driving mode
  - **B Button**: Reverse belt
  - **Y Button**: Cycle belt speeds
  - **X Button**: Control deposit auger
  - **D-Pad Up/Down**: Control belt position

### I2C Node

The I2C node manages communication with motor controllers and other I2C devices:

```bash
# View I2C statistics
ros2 topic echo /i2c_node/diag
```

### Serial Node

The Serial node provides communication with Arduino-based peripherals (if used):

```bash
# Send a serial command
ros2 topic pub /serial_node/serial std_msgs/String "data: 'm,90,90'"
```

## Development

### Project Structure

```
moondawg-ros/
├── config/
│   └── moondawg_params.yaml    # Configuration parameters
├── launch/
│   └── moondawg.launch.py      # Main launch file
├── moondawg/
│   ├── controller_parser.py    # Xbox controller processing node
│   ├── i2c_node.py             # I2C communication node
│   ├── serial_node.py          # Serial communication node
│   └── string_gen.py           # Command string generator utilities
├── website/
│   ├── index.html              # Web interface
│   └── script.js               # Web interface JavaScript
└── scripts/
    └── install_deps.sh         # Dependency installation script
```

### Adding New Features

To add new robot functionality:

1. Update the appropriate node (controller_parser.py, i2c_node.py, or serial_node.py)
2. Add any new parameters to config/moondawg_params.yaml
3. Update the web interface if needed

## Troubleshooting

### Connection Issues

- **ROSBridge Connection Failures**: Check network connectivity and firewall settings
- **I2C Device Not Responding**: Verify address and bus settings in config/moondawg_params.yaml
- **Serial Connection Failures**: Check if the correct port is specified and verify permissions

### Diagnostic Information

The package provides extensive diagnostic information on various topics:

```bash
# View controller diagnostic info
ros2 topic echo /controller_parser/diag

# View I2C diagnostic info
ros2 topic echo /i2c_node/diag

# View serial diagnostic info
ros2 topic echo /serial_node/diag
```

## License

This project is licensed under the Apache License 2.0.
