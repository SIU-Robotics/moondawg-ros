#!/bin/bash
set -e

# Run as root
if [ "$EUID" -ne 0 ]; then
    echo "This script must be run as root. Please run with 'sudo bash $0' or as root user."
    exit 1
fi

touch /usr/local/bin/run_moondawg.sh
echo """#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/robotics/moondawg-ros/install/setup.bash
ros2 launch moondawg moondawg.launch.py""" > /usr/local/bin/run_moondawg.sh
chmod +x /usr/local/bin/run_moondawg.sh

touch /etc/systemd/system/moondawg.service
echo """[Unit]
Description=Starting ROS on startup
After=network.target

[Service]
ExecStart=/usr/local/bin/run_moondawg.sh
User=robotics
Restart=on-failure

[Install]
WantedBy=multi-user.target""" > /etc/systemd/system/moondawg.service

systemctl daemon-reload
systemctl enable moondawg.service
systemctl start moondawg.service
