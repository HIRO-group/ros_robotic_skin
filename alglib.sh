#!/bin/bash

wget https://www.alglib.net/translator/re/alglib-3.16.0.cpp.gpl.tgz
mkdir src/alglib
tar -xzf alglib-3.16.0.cpp.gpl.tgz -C  src/alglib
rm alglib-3.16.0.cpp.gpl.tgz
mv src/alglib/cpp/src src/alglib
rm -rf src/alglib/cpp

catkin build ros_robotic_skin


