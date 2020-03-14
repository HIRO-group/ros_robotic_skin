#!/bin/bash
# The above line is required for shellcheck which is a great linter, to parse
# the script and interpret it as a bash not zsh, it's a hack and it works
# CATKIN_WORKSPACE will be used as path where the path script resides
CATKIN_WORKSPACE=$(pwd)
LIBFRANKA_BUILD_DIR=$CATKIN_WORKSPACE/src/libfranka/build
# Activate sources
source /opt/ros/melodic/setup.bash
# Install all the dependencies required
rosdep install --from-paths src --ignore-src -y --skip-keys libfranka
rosdep install --from-paths src --ignore-src -y --skip-keys ros_robotic_skin
# I don't know why but rosdep doesn't install the below package which is required
# Raise an issue for that maybe?
sudo apt install ros-melodic-imu-filter-madgwick
sudo apt install ros-melodic-effort-controllers
# End installing
if [ -d "src/libfranka/build" ]; then
    echo "libfranka build already exists. Good!"
else
  cd src/libfranka
  git submodule update --init
  mkdir build
  cd "$CATKIN_WORKSPACE"
fi
if ! cd src/libfranka/build; then
  echo "cd to libfranka not possible. Something went wrong, check if build directory was created in libfranka"
fi
echo "in libfranka build directory"
# Delete all files to make a clean install homie!
if ! rm -rf * ; then
    echo "Unable to delete files in libfranks build. Exiting..."
    exit 1
fi
if ! cmake -DCMAKE_BUILD_TYPE=Release .. ; then
    echo "Unable to generate make files for libfranka, maybe some dependency missing? Check logs above. Exiting..."
    exit 1
fi
if ! make ; then
    echo "Unable to build libfranka, maybe some missing dependency? Exiting..."
    exit 1
fi
# All good, lets run catkin_make
if ! cmake .. ; then
    echo "Unable to generate make files for libfranka, maybe some dependency missing? Check logs above. Exiting..."
    exit 1
fi
if ! cd $CATKIN_WORKSPACE ; then
    echo "cd to catkin workspace failed, you might be running the script at wrong place"
    exit 1
fi
if ! catkin_make -j4 -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=$LIBFRANKA_BUILD_DIR ; then
    echo "cd to catkin workspace failed, you might be running the script at wrong place"
    exit 1
fi
if [ -d "$CATKIN_WORKSPACE/build" ]; then
  rm -rf "$CATKIN_WORKSPACE/build"
  echo "Deleted build"
fi

if [ -d "$CATKIN_WORKSPACE/devel" ]; then
  rm -rf "$CATKIN_WORKSPACE/devel"
  echo "Deleted devel"
fi

if [ -d "$CATKIN_WORKSPACE/install" ]; then
  rm -rf "$CATKIN_WORKSPACE/install"
  echo "Deleted install"
fi
# Finally do catk make install
catkin_make install
