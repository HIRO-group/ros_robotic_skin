![Github Actions:Build](https://github.com/HIRO-group/ros_robotic_skin/workflows/ROS%20Robotic%20Skin%20CI/badge.svg)
![Documentation Generation](https://github.com/HIRO-group/ros_robotic_skin/workflows/Documentation%20Generation/badge.svg)


# ROS Robotic Skin
- **Current Release**: `0.0.1` as of 2020/8/2
- **Suporting Version**: `ROS Melodic`
- **Documentation**: https://hiro-group.ronc.one/ros_robotic_skin/

# Quick Installation
```
# Assume catkin_ws is already made
cd catkin_ws/src

# Install hiro_ros_arm_controller
git clone git@github.com:HIRO-group/hiro_ros_arm_controller.git
cd hiro_ros_arm_controller
./install.sh
cd ..

# Install this repo
git clone https://github.com/HIRO-group/ros_robotic_skin/
cd ..
catkin build   # or catkin make
```

# Docker Installation
We also have a [`Dockerfile`](https://github.com/HIRO-group/ros_robotic_skin/blob/master/Dockerfile). See how you install and run it in [our Wiki page](https://github.com/HIRO-group/ros_robotic_skin/wiki/Running-on-Docker)


# Launch Files
## Prerequisite
In order to run the Panda Gazebo simulation, make sure that you have built your workspace, then run (from `catkin_ws`, or the root of your catkin workspace)

This tutorial will **assume** that you have added this line to your `~/.bashrc` so you don't have to `source` the `setup.bash` file everytime you open
a terminal.

```sh
source <path to your workspace>/devel/setup.bash
# If you followed the ROS tutorials, it should be
# source ~/catkin_ws/devel/setup.bash
```

## Running Panda in simulation
You can add as many IMUs as you want in `config/imu_poses/imu_poses.txt`.
The format per line is as follows:

```sh
x,y,z,roll,pitch,yaw, num_link_connected_to
```

Run

```sh
python scripts/imu_spawners/spawn_real_imus.py
```

to update the xacro file with the IMU changes. Now, to run the simulation:

```sh

roslaunch ros_robotic_skin simulation.launch
```

![](images/panda_example.png)


## Running Real Panda
Change the `robot_ip` accordingly.
```sh
roslaunch ros_robotic_skin panda.launch robot_ip:=172.16.0.172
```

![](images/real_panda.jpg)

## Running Sawyer in simulation
```sh
roslaunch ros_robotic_skin sawyer_world.launch
```

![](images/sawyer_example.png)

# For DEVELOPERS
## Test
We use 2 different tests.
1. `Flake8` <br>
to enforce style consistency across Python projects.
2. `ROS Test` <br>
to ensure our codes run without any bugs.

### RUN TESTS BEFORE COMMITTING
#### `Flake8`
```sh
flake8 . --max-complexity=10 --max-line-length=140
```
within this repository (after cloning and changing directories to `ros_robotic_skin`).

#### `ROS Test`
Make sure that you have sourced the catkin workspace containing this package.
```sh
rostest ros_robotic_skin test.test
```
This will run our simulation without the GUI and perform the tests.

Guides to add tests are described in [our Wiki page](https://github.com/HIRO-group/ros_robotic_skin/wiki/How-to-add-test)


## Parameters
Parameters are all saved in `config/params.yaml`. <br>
In every launch file, it should load the yaml file. <br>
Whenever you use it, load it like `example_param = ropsy.get_param("/example_param")`

Refer to
- https://roboticsbackend.com/ros-param-yaml-format/
- https://roboticsbackend.com/get-set-ros-params-rospy-roscpp/

# Miscellaneous
Here are some extra resources for this package.

## Ding Paper Explanation Resources
Below are links to an explanation of the Ding control paper and the Overleaf document that has the equations included in the explanation.

- Ding Explanation: [here](https://docs.google.com/presentation/d/1LrW7mna1wRgHsIzw3wXOrvIg3xlkNpIfmVRfGyxG_v0/edit?usp=sharing)
- Overleaf File: [here](https://www.overleaf.com/read/hwndqxxqtvds)
