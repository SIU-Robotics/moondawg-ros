#!/bin/bash

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root. Please run with 'sudo bash $0' or as root user."
    exit 1
fi

# Run commands without sudo
apt update && apt install ros-jazzy-cv-bridge ros-jazzy-rosbridge-suite ros-jazzy-image-tools -y
apt update && apt install python3-opencv python3-smbus2 -y