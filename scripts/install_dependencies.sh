#!/bin/bash
set -e

# Run as root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root. Please run with 'sudo bash $0' or as root user."
    exit 1
fi

apt update
apt install ros-jazzy-cv-bridge ros-jazzy-rosbridge-suite ros-jazzy-image-tools -y
apt install ros-jazzy-realsense2* -y
apt install ros-jazzy-librealsense2* -y
apt install python3-opencv python3-smbus2 -y

git clone https://github.com/IntelRealSense/librealsense
cd librealsense
bash scripts/setup_udev_rules.sh
cd ../
rm -rf librealsense

# Reload udev rules
udevadm control --reload-rules && udevadm trigger