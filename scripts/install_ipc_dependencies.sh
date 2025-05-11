#!/bin/bash
# Install dependencies for intra-process communication in ROS2

echo "Installing dependencies for ROS2 intra-process communication..."

# Install required ROS2 packages
sudo apt-get update
sudo apt-get install -y \
    ros-humble-rclcpp-components \
    ros-humble-composition

echo "Dependencies installed successfully!"
echo "Building the workspace..."

# Return to workspace and build
cd ..
colcon build --symlink-install

echo "Done! You can now run the launch file with intra-process communication enabled:"
echo "ros2 launch moondawg moondawg.launch.py"
