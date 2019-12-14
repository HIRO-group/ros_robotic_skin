
# setting 
set these environment variables

On Raspberry Pi
```
ROS_MASTER_URI=http://IP_ADDRESS_OF_RASPI:11311
ROS_IP=IP_ADDRESS_OF_THIS_MACINE
```

On your pc
```
ROS_MASTER_URI=http://IP_ADDRESS_OF_RASPI:11311
ROS_IP=IP_ADDRESS_OF_THIS_MACIHNE
```

# Troubleshooting
Although you can see a list of topics on your pc by `rostopic list`, but cannot see any messages `rostopic echo SOME_MESSAGE`, then check your network setting
https://github.com/ros/meta-ros/issues/134

You need to export `ROS_ROOT` and `CMAKE_PREFIX_PATH`, and also don't forget to add lists to your `/etc/hosts`
```
on qemux86-64:
ROS_ROOT=/usr
ROS_MASTER_URI=http://qemux86-64:11311/
CMAKE_PREFIX_PATH=/usr

/etc/hosts:
192.168.7.2 qemux86-64.localdomain qemux86-64

on my other machine:
ROS_MASTER_URI=http://192.168.7.2:11311/

etc/hosts:
192.168.7.2 qemux86-64
```
