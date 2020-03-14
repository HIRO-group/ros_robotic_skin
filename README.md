![ROS_Robotic_Skin CI](https://github.com/HIRO-group/ros_robotic_skin/workflows/ROS_Robotic_Skin%20CI/badge.svg)

# General
## Current Release
- `0.0.1` as of 2020/3/13

## Supporting version
`ROS Melodic`

# Installation
## `robotic_skin` python package
```sh
# Using HTTPS:
pip install --upgrade git+https://github.com/HIRO-group/robotic_skin.git
# Using SSH:
pip install --upgrade git+ssh://git@github.com/HIRO-group/robotic_skin.git
```

## `ros_robotic_skin`
```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:HIRO-group/ros_robotic_skin.git
cd ..
catkin_make
```

## Franka Panda Gazebo Simulator
Please refer to the Franka Install Guide [here](https://hiro-group.ronc.one/franka_installation_tutorial.html) for dependencies. <br>
Be careful to install `melodic` dependencies.

For the panda simulator, install `panda_simulation` as
```sh
cd ~/catkin_ws/src
git clone https://github.com/HIRO-group/panda_simulation
git clone https://github.com/erdalpekel/panda_moveit_config
git clone --branch simulation https://github.com/HIRO-group/franka_ros
cd ..
sudo apt-get install libboost-filesystem-dev
```

You can install `libfranka` via `apt-get` or from source.

Via `apt-get`:

```sh
sudo apt-get install ros-melodic-libfranka
```

From source:

```sh
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev

git clone --recursive https://github.com/frankaemika/libfranka
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka
cd libfranka

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

Making the workspace
```sh
cd ~/catkin_ws/src
# append the -DFranka_DIR part if you built libfranka from source.
catkin_make -DFranka_DIR:PATH=/path/to/libfranka/build
```

## (Optional but Recommended) Install Sawyer Gazebo Simulator
```sh
cd ~/catkin_ws/src
wstool init
wstool merge https://gist.githubusercontent.com/jarvisschultz/f65d36e3f99d94a6c3d9900fa01ee72e/raw/sawyer_packages.rosinstall
wstool update
cd ..
catkin_make
```

## Running Simulation

In order to run the Panda Gazebo simulation, make sure that you have built your workspace, then run (from `catkin_ws`, or the root of your catkin workspace)
```sh
source devel/setup.bash
roslaunch ros_robotic_skin simulation.launch
```


## Running Real Panda

Perform similar steps as what you would do for running the Panda Gazebo simulation, and then run:
```sh
source devel/setup.bash
roslaunch ros_robotic_skin panda.launch robot_ip:=172.16.0.172
```

## Capture Poses
For this section to work, you must first be running 1 of 3 tasks:
- The Franka Panda simulation
- The **real** Franka Panda running ROS
- The Sawyer Robot simulation

Otherwise, this code will **not** work.

If you want to be able to capture certain poses of the robot, run
```sh
roslaunch ros_robotic_skin collect_poses.launch is_sim:=<true|false> save_file:=<filename.txt> robot_type:=<sawyer|panda>
```

`is_sim` specifies if the robot is being run in simulation, `save_file` specifies where to save the file of collected poses, and `robot_type` specifies the type of robot (there's only two choices - Sawyer and Panda). This will save a file of the poses in the `data` folder, which should serve helpful later on.

## Activity Matrix Generation
The activity matrix generation currently needs to be done for three separate files, but we have a roslaunch file to handle this. To do so, from the root of your catkin workspace, run:

```sh

source devel/setup.bash
roslaunch activity_matrix.launch is_sim:=<sim_bool> filename:=<txt_filename>

```

Where `sim_bool` should be `true` is you are running the Panda in simulation, and `sim_bool` should be `false` if you are running the Panda in real life. These series of commands will run the Panda through a variety of poses and generate the necessary activity matrix. The activity matrix will be saved in the `data` folder of this package, under `txt_filename`. By default,
`txt_filename` is `positions.txt`.

## Miscellaneous

It is **highly** recommended to add the line 
```sh
source <path to your workspace>/devel/setup.bash
```
to your `.bashrc` file.

# Documentation Generation

We use [`rosdoc_lite`](http://wiki.ros.org/rosdoc_lite) for documentation generation. You can check it out
[here](https://hiro-group.ronc.one/ros_robotic_skin). When you are making a pull request to this repository, and you get your PR merged, you can update the documentation with the following steps (assuming you are on the master branch):

```sh

git pull origin master
git checkout gh-pages
git merge master
./docs_generate.sh

```

These commands and scripts will ensure that the `gh-pages` branch is up to date with your changes. From here, you can commit and push to the remote branch, and https://hiro-group.ronc.one/ros_robotic_skin should be updated fairly soon.

# Flake8 Testing

In order to have the Github Actions build pass, we use `flake8` for style enforcement. To test this, simply run

```sh
flake8 . --max-complexity=10 --max-line-length=140
``` 
within this repository (after cloning and changing directories to `ros_robotic_skin`).

# Setting
Set these environment variables

On Raspberry Pi
```
ROS_MASTER_URI=http://IP_ADDRESS_OF_RASPI:11311
ROS_IP=IP_ADDRESS_OF_THIS_MACHINE
```
In our case, `IP_ADDRESS_OF_RASPI` was `172.16.0.143` and
`IP_ADDRESS_OF_THIS_MACHINE` was `172.16.0.143`

On your pc
```
ROS_MASTER_URI=http://IP_ADDRESS_OF_RASPI:11311
ROS_IP=IP_ADDRESS_OF_THIS_MACHINE
```
In our case, `IP_ADDRESS_OF_RASPI` was `172.16.0.143` and
`IP_ADDRESS_OF_THIS_MACHINE` was `172.16.0.238`

# Troubleshooting
Although you can see a list of topics on your pc by `rostopic list`, but cannot see any messages `rostopic echo SOME_MESSAGE`, then check your network setting
https://github.com/ros/meta-ros/issues/134

You need to export `ROS_ROOT` and `CMAKE_PREFIX_PATH`, and also don't forget to add lists to your `/etc/hosts`

On Raspberry Pi:
```
ROS_ROOT=/usr
CMAKE_PREFIX_PATH=/usr
```

```
127.0.1.1 raspberrypi
IP_ADDRESS_OF_RASPI raspberrypi.localdomain raspberrypi
```
In our case, `IP_ADDRESS_OF_RASPI`  was `172.16.0.143`

Good article explaining everything in more detail: https://krishnachaitanya9.github.io/posts/ros_publish_subscribe/


# For DEVELOPERS
## Test
### How to run test
```
cd ~/catkin_ws
cakin_make run_tests
```

### How to add test
1. Add unittest under `test/` directory.
2. Add these lines in `launch/test.test`
```xml
<launch>
    <!-- type specifices which test script to run -->
    <test test-name='test' pkg='ros_robotic_skin' type='template.py'/>
</launch>
```
3. (If not done) Add these lines below to `CMakeLists.txt`
```cmake
find_package(rostest REQUIRED)
add_rostest(launch/test.test)
```

# Parameters
Parameters are all saved in `config/params.yaml`. <br>
In every launch file, it should load the yaml file. <br>
Whenever you use it, load it like `example_param = ropsy.get_param("/example_param")`

Refer to
- https://roboticsbackend.com/ros-param-yaml-format/
- https://roboticsbackend.com/get-set-ros-params-rospy-roscpp/
