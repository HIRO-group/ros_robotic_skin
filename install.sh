#!/bin/bash

source /opt/ros/melodic/setup.bash
pip3 install --upgrade git+https://github.com/HIRO-group/robotic_skin.git
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone https://github.com/HIRO-group/ros_robotic_skin
git clone https://github.com/HIRO-group/panda_simulation
git clone https://github.com/erdalpekel/panda_moveit_config
git clone --branch simulation https://github.com/HIRO-group/franka_ros
cd ..
sudo apt-get install libboost-filesystem-dev
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka
sudo apt-get install ros-melodic-imu-madgwick
cd src

if [[ $1 == 'source' ]]
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
catkin_make
