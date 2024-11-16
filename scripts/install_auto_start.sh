touch ~/run_moondawg.sh
echo """#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/nasa/moondawg-ros/install/setup.bash
ros2 launch moondawg moondawg.launch.py""" > ~/run_moondawg.sh
sudo chmod +x ~/run_moondawg.sh
sudo touch /etc/systemd/system/moondawg.service
sudo echo """[Unit]
Description=Starting ROS on startup
After=network.target

[Service]
ExecStart=/home/nasa/run_moondawg.sh
User=nasa
WorkingDirectory=/home/nasa/
Restart=on-failure

[Install]
WantedBy=multi-user.target""" > /etc/systemd/system/moondawg.service
sudo systemctl daemon-reload
sudo systemctl enable moondawg.service
sudo systemctl start moondawg.service