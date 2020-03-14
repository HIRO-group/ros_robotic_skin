#!/bin/bash
sudo apt remove --purge ros-*
sudo apt purge --auto-remove ros-melodic-desktop-full
sudo rm -rf /etc/ros
echo "I guess ROS is now fully removed, check if any files exist in /opt/ros and then dont forget to restart"
