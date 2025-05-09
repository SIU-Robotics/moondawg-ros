#!/bin/bash

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root. Please run with 'sudo bash $0' or as root user."
    exit 1
fi

# Run commands without sudo
apt update && apt install ros-jazzy-cv-bridge ros-jazzy-rosbridge-suite ros-jazzy-image-tools -y
apt update && apt install ros-jazzy-realsense2-camera ros-jazzy-realsense2-description -y
apt update && apt install python3-opencv python3-smbus2 -y

# Install RealSense SDK dependencies (if needed)
apt update && apt install -y \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev

# Create udev rules for RealSense cameras to allow access without root privileges
cat > /etc/udev/rules.d/99-realsense-libusb.rules << EOF
# Intel RealSense devices
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b07", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b3a", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0afb", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad3", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0acb", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0af2", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad6", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0afe", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0afd", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad5", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad4", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad2", MODE="0666"
EOF

# Reload udev rules
udevadm control --reload-rules && udevadm trigger