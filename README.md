![Github Actions:Build](https://github.com/HIRO-group/ros_robotic_skin/workflows/ROS%20Robotic%20Skin%20CI/badge.svg)
![Documentation Generation](https://github.com/HIRO-group/ros_robotic_skin/workflows/Documentation%20Generation/badge.svg)


# ROS Robotic Skin
- **Current Release**: `0.0.1` as of 2020/3/25
- **Suporting Version**: `ROS Melodic`
- **Documentation**: https://hiro-group.ronc.one/ros_robotic_skin/

# Installation

We have an `install.sh` script that will install the following packages:

- Installs our Python `robotic_skin` package [here](https://github.com/HIRO-group/robotic_skin)

- The Franka Panda Gazebo Simulator package [here](https://github.com/HIRO-group/panda_simulation)

- The Sawyer Gazebo simulator

## To run `install.sh`
Make sure that you have cloned this repository from the `src` folder of a catkin workspace (eg: from `catkin_ws/src`). If you haven't, the script will give an error.
Additionally, you should clone this repository in an **empty** catkin workspace. For example, if you have the `franka_ros` package in the workspace, it will be deleted in favor of our forked version that works with this package.

### Obstacle Avoidance Simulation Install

1. In a ros catkin workspace on your computer in the `src` directory run the following to clone our github repo: `git clone https://github.com/HIRO-group/ros_robotic_skin.git` 

2. Change to the newly cloned directory: `cd ros_robotic_skin`

3. Change to the QPControl branch: `git checkout QPControl`

3. Run the install script with the following options: `./install.sh --git-option https --franka-build apt` this script should install all dependencies and build(compile) the competed workspace for you. Install can take up to 20 minutes on older machines.

4. Source your catkin workspace to ensure that the build changes are reflected in your terminal. From the head directory of your catkin workspace (the directory above where you installed ros_robotic_skin) run: `source devel/setup.bash`

5. To start the simulation use flacco launch file: `roslaunch ros_robotic_skin flacco.launch`

6. To move the arm from in simulation run: `rosrun ros_robotic_skin CartesianPositionController` in another terminal while step 5 is running. This will execute the code in the file: `CartesianPositionController.cpp` and should move in a circle for about 20 seconds. If you would like to run the controller with obstacle avoidance, include the `HIRO` flag at the end of the terminal command: `rosrun ros_robotic_skin CartesianPositionController HIRO`

7. If you want to alter the `CartesianPositionController.cpp` with your own movement start by looking at the bottom of the file where our example is located. The function `moveInCircle` should give you an overview of what you need to do in order to move the arm. 

8. If you change any C++ files in this project you must build the workspace again. Run the command: `catkin build ros_robotic_skin` after you have made and saved your changes. 

### Obstacle Avoidance Example

1. We have a launch file that will automatically start multiple files for the simulation and avoid an obstacle. File location `ros_robotic_skin/launch/avoidance_test.launch`
2. (Note) You might need to change the line `<arg name="node_start_delay" default="5.0" />`  so that the simulation starts before the controller, visualizer, and obstacle publisher. The number 5 indicates the number of seconds that nodes with the line `launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@'` wait before they run. 
3. Run the file with HIRO avoidance: `roslaunch ros_robotic_skin avoidance_test.launch avoidance_type:=HIRO`
4. To run without an avoidance algorithm run: `roslaunch ros_robotic_skin avoidance_test.launch`
5. The avoidance methods QP and Flacco are still being developed and will not function properly.



# End of Avoidance install information

### Usage:
```sh
./install.sh --git-option https|ssh --franka-build apt|source
```

### Examples
1. Simply Run (Default options= `ssh` && `apt`)
```sh
./install.sh
```

2. Use options
```sh
./install.sh --git-option ssh --franka-build source
```
This command will build `libfranka` from source and use ssh for git.

### Options
- `--git-option` <br>
specifies if we clone the `HIRO` repos via https or ssh.

- `franka_build` <br>
specifies whether we want to build `libfranka` from source or install it via `apt`.

### Docker
We also have a [`Dockerfile`](https://github.com/HIRO-group/ros_robotic_skin/blob/master/Dockerfile). See how you install and run it in [our Wiki page](https://github.com/HIRO-group/ros_robotic_skin/wiki/Running-on-Docker)

# Launch Files
## Prerequisite
In order to run the Panda Gazebo simulation, make sure that you have built your workspace, then run (from `catkin_ws`, or the root of your catkin workspace)

This tutorial will **assume** that you have added this line to your `~/.bashrc` so you don't have to `source` the `setup.bash` file everytime you open
a terminal.

```sh
source <path to your workspace>/devel/setup.bash
```

## Running Panda in simulation
You can add as many IMUs as you want in `config/imu_poses.txt`.
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
to ensure our codes run without any bugs

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

Guides to add tests are descibed in [our Wiki page](https://github.com/HIRO-group/ros_robotic_skin/wiki/How-to-add-test)


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

- Ding Explanation: [here...](https://docs.google.com/presentation/d/1LrW7mna1wRgHsIzw3wXOrvIg3xlkNpIfmVRfGyxG_v0/edit?usp=sharing)
- Overleaf File: [here...](https://www.overleaf.com/read/hwndqxxqtvds)
