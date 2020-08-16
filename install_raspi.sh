#!/bin/bash

# HIRO Group installation script for ros_robotic_skin
# and the corresponding ROS packages.

# TODO: Need to check if we are currently in catkin_ws/src
cd .. && git clone git@github.com:HIRO-group/hiro_ros_arm_controller.git
cd .. && catkin build

PROJECT_DIR="~/Documents/project"
if [ -L "$PROJECT_DIR"]; then
    echo "Dir exists"
else
    echo "Not a valid catkin workspace!"
    exit 1
fi

cd "$PROJECT_DIR" && git clone git@github.com:HIRO-group/roboskin.git
cd roboskin
python setup.py install --raspi
