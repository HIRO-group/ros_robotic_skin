#!/bin/bash
# shellcheck shell=bash
CATKIN_WORKSPACE=$(pwd)
PROJECT_DEVEL_SETUP=$CATKIN_WORKSPACE/devel/setup.bash
source ~/.bashrc
source /opt/ros/melodic/setup.bash
source "$PROJECT_DEVEL_SETUP"
source /usr/share/gazebo/setup.sh
roslaunch ros_robotic_skin simulation.launch
