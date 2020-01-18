# Installation of `robotic_skin` python package
```
pip install --upgrade git+https://github.com/HIRO-group/robotic_skin.git
```

# Installation of `ros_robotic_skin`
```
cd ~/catkin_ws/src
git clone git@github.com:HIRO-group/ros_robotic_skin.git
cd ~/catkin_ws
cakin_make
```

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
