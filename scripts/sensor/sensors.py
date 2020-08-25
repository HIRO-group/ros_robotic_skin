import os

import rospkg
from roboskin.sensor.lsm6ds3 import LSM6DS3_IMU
ROS_ROBOSKIN_DIR = rospkg.RosPack().get_path('ros_robotic_skin')
CONFIG_DIR = os.path.join(ROS_ROBOSKIN_DIR, 'config')


def set_environment_variables(ros_core_ip):
    # Check for ros_core_ip
    if ros_core_ip is not None:
        raise ValueError('ros_core_ip should not be None')

    # Use Default port value
    ros_core_port = '11311'

    os.environ["ROS_MASTER_URI"] = 'http://%s:%d' % (ros_core_ip, ros_core_port)


class IMU(LSM6DS3_IMU):
    def __init__(self, ros_core_ip=None, **kwargs):
        super().__init__(self, **kwargs)
        if ros_core_ip is not None:
            set_environment_variables(ros_core_ip)
