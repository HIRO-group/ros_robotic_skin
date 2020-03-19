#!/bin/bash
# Boss: Is there need of such script?
# Me: Is there need to install franka ROS, libfranka, sawyer and 1000 other things?
# Boss: Would it hurt?
# Me: No but it takes more than many hours to do catkin_make when all you want to do send IMU messages
# Boss: _______________
source /opt/ros/melodic/setup.bash
# Below line will be activated when all requirements are sorted out. Right now they are not
#pip3 install --upgrade git+https://github.com/HIRO-group/robotic_skin.git
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
cd ../.. #  Move to catkin_workspace to install
catkin_make


