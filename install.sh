#!/bin/bash

# options for:
# git ssh vs https
# desired name of workspace name
# build libfranka from source or not
GIT_OPTION="ssh"
WORKSPACE_NAME="catkin_ws"  
FRANKA_BUILD="apt-get"

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
        GIT_OPTION="ssh"
    fi
    shift # past argument
    shift # past value
    ;;
    --ws-name)
    WORKSPACE_NAME="$2"
    shift # past argument
    shift # past value
    ;;
    --franka-build)
    if [[ "$2" == "source" || "$2" == "apt-get" ]]
    then
        FRANKA_BUILD="$2"
    else
        echo "source or apt-get not selected. Resorting to apt-get option."
        FRANKA_BUILD="apt-get"
    fi
    shift # past argument
    shift # past value
    ;;
    *)    # unknown option
    shift # past argument
    ;;
esac
done

source /opt/ros/melodic/setup.bash

if [[ $GIT_OPTION = "ssh" ]]
then
  pip3 install --upgrade git+https://github.com/HIRO-group/robotic_skin.git
else
  pip3 install --upgrade git+ssh://git@github.com/HIRO-group/robotic_skin.git
fi

mkdir -p $WORKSPACE_NAME/src
cd $WORKSPACE_NAME/src

if [[ $GIT_OPTION = "ssh" ]]
then
  git clone git@github.com:HIRO-group/ros_robotic_skin.git
else
  git clone https://github.com/HIRO-group/ros_robotic_skin
fi
  
git clone https://github.com/HIRO-group/panda_simulation
git clone https://github.com/erdalpekel/panda_moveit_config
git clone --branch simulation https://github.com/HIRO-group/franka_ros
cd ..
sudo apt-get install libboost-filesystem-dev
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka --skip-keys ros_robotic_skin
sudo apt-get install ros-melodic-imu-madgwick
cd src

if [[ $FRANKA_BUILD = "source" ]]
then
  sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
  git clone --recursive https://github.com/frankaemika/libfranka
  cd libfranka
  mkdir build
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  cmake --build .
  cd ../../..
  catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$(pwd)/src/libfranka/build

else
  sudo apt-get install ros-melodic-libfranka
  cd ..
  catkin_make -j4 -DCMAKE_BUILD_TYPE=Release
fi

cd src
wstool init
wstool merge https://gist.githubusercontent.com/jarvisschultz/f65d36e3f99d94a6c3d9900fa01ee72e/raw/sawyer_packages.rosinstall
wstool update
cd ..
sudo apt-get install -y ros-melodic-joystick-drivers
sudo apt-get install -y ros-melodic-image-proc

sed -i '48i\target_link_libraries(${PROJECT_NAME} yaml-cpp)' src/sawyer_simulator/sawyer_sim_controllers/CMakeLists.txt
if [[ $FRANKA_BUILD = "source" ]]
then
  catkin_make -DFranka_DIR:PATH=$(pwd)/src/libfranka/build
else
  catkin_make
fi
