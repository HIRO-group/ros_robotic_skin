#!/bin/bash

# HIRO Group installation script for ros_robotic_skin
# and the corresponding ROS packages.

# TODO: Need to check if we are currently in catkin_ws/src
cd .. && git clone git@github.com:HIRO-group/hiro_ros_arm_controller.git
git clone https://github.com/HIRO-group/imu_calib
cd hiro_ros_arm_controller && ./install.sh
cd ../.. && catkin build
