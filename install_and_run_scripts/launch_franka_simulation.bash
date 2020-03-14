#!/bin/bash
# The above line is required for shellcheck which is a great linter, to parse
# the script and interpret it as a bash not zsh, it's a hack and it works
# I hope zsh will provide a good linter, I despertely need one, it's an awesome
# shell, but God gave just shellcheck for now, so gotta djust homie!
# CATKIN_WORKSPACE will be used as path where the path script resides
CATKIN_WORKSPACE=$(pwd)
PROJECT_DEVEL_SETUP="$CATKIN_WORKSPACE"/devel/setup.bash
gnome-terminal -- ./launch_gazebo.bash
# sleep 50
# gnome-terminal -- ./imu_listener.zsh
# gnome-terminal -- ./launch_activation_matrix.zsh
# gnome-terminal -- ./joint_movement.zsh
