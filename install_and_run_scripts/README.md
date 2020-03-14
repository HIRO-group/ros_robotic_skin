# ROS_launch_scripts

This repository contains all launch scripts required to run the Franka Gazebo Simulation.

### Guidelines

- All the paths should be with respect to a base folder
- I know some paths need to be hard coded, they are to be declared on top
- All scripts should be in zsh. Don't forget to install shell linter. 

The repository should be cloned just inside the catkin work space, within same level there should be src, build, devel, install

### Usage:

- ./launch_franka_simulations.zsh (For running all your simulations)
- ./build_all_packages.zsh (For building everything and catkin_make from scratch)

Good Luck!