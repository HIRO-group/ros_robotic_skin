#!/bin/bash

# HIRO Group installation script for ros_robotic_skin
# and the corresponding ROS packages.

# checks to see if a valid catkin workspace has been created
TWO_DOTS=$(cd ../../ && pwd)
THREE_DOTS=$(cd ../../.. && pwd)
if [[ -d ../../src && $TWO_DOTS != $THREE_DOTS ]]; then
    echo "Dir exists"
else
    echo "Not a valid catkin workspace!"
    echo "Terminating installation..."
    # exit the whole thing
    exit 1
fi

# goes through the arguments that were specified
while [[ $# -gt 0 ]]
do
key="$1"
case $key in
    --git-option)
    if [[ "$2" == "ssh" || "$2" == "https" ]]
    then
        GIT_OPTION="$2"
    else
        echo "ssh or https not selected. Resorting to ssh git option."
    fi
    shift # past argument
    shift # past value
    ;;

    --franka-build)
    if [[ "$2" == "source" || "$2" == "apt" ]]
    then
        FRANKA_BUILD="$2"
    else
        echo "source or apt not selected. Resorting to apt option."
    fi
    shift # past argument
    shift # past value
    ;;
    *)    # unknown option
    shift # past argument
    ;;
esac
done

# if the user didn't specifies git method and franka build method,
# resort to defaults.
if [[ -z "$GIT_OPTION" ]]
then
    echo "--git-option was not set. Setting it to ssh by default."
    GIT_OPTION="ssh"
fi

if [[ -z "$FRANKA_BUILD" ]]
then
    echo "--franka-build was not set. Setting it to apt by default."
    FRANKA_BUILD="apt"
fi

# needed for ros
source /opt/ros/melodic/setup.bash
sudo apt update
sudo apt upgrade
sudo apt install python3-pip python-catkin-tools


if [[ $GIT_OPTION = "ssh" ]]
then
  pip3 install --upgrade --user git+ssh://git@github.com/HIRO-group/robotic_skin.git
else
  pip3 install --upgrade --user git+https://github.com/HIRO-group/robotic_skin.git
fi
cd ..
# clone repositories for simulation

if [ -d "panda_simulation" ]
then
    echo "panda_simulation package exists, deleting to make sure our forked version is cloned."
    rm -rf panda_simulation
fi

git clone https://github.com/HIRO-group/panda_simulation

if [ -d "panda_moveit_config" ]
then
    echo "panda_moveit_config package exists, deleting to make sure erdal's forked version is cloned."
    rm -rf panda_moveit_config
fi

git clone https://github.com/ros-planning/panda_moveit_config


if [ -d "franka_ros" ]
then
    echo "franka_ros package exists, deleting to make sure our forked version is cloned."
    rm -rf franka_ros
fi

git clone --branch simulation https://github.com/HIRO-group/franka_ros

cd ..
sudo apt install libboost-filesystem-dev
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka --skip-keys ros_robotic_skin --skip-keys libgazebo7-dev
sudo apt install ros-melodic-imu-filter-madgwick ros-melodic-ros-control ros-melodic-ros-controllers
cd src
git clone git@github.com:HIRO-group/Custom_IMU_Madgwick_Filter.git

# install dependencies
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev libcgal-dev

if [[ $FRANKA_BUILD = "source" ]]
then
  git clone --recursive https://github.com/frankaemika/libfranka
  cd libfranka
  mkdir build
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  cmake --build .
  cd ../../..

else
  sudo apt install ros-melodic-libfranka
  cd ..
fi

# setup sawyer simulation
cd src

if [ -d "sawyer_robot" ]
then
    echo "wstool initialized already, skipping"

else
    wstool init

fi

wstool merge https://gist.githubusercontent.com/jarvisschultz/f65d36e3f99d94a6c3d9900fa01ee72e/raw/sawyer_packages.rosinstall
wstool update
cd ..
rosdep install --from-paths src --ignore-src -y -r --skip-keys libgazebo7-dev
sudo apt install ros-melodic-moveit-visual-tools




# fix error from ld command
sed -i '48i\target_link_libraries(${PROJECT_NAME} yaml-cpp)' src/sawyer_simulator/sawyer_sim_controllers/CMakeLists.txt
# clean things up before the show!
rm -rf devel
rm -rf build
if [[ $FRANKA_BUILD = "source" ]]
then
  catkin build -DFranka_DIR:PATH=$(pwd)/src/libfranka/build
else
  catkin build
fi
