import os
import yaml

import rospkg
from roboskin.sensor.lsm6ds3 import LSM6DS3_IMU
ROS_ROBOSKIN_DIR = rospkg.RosPack().get_path('ros_robotic_skin')
CONFIG_DIR = os.path.join(ROS_ROBOSKIN_DIR, 'config')


def set_environment_variables(filename):
    filepath = os.path.join(CONFIG_DIR, filename)

    with open(filepath, 'r') as f:
        env_variables = yaml.load(f)

    # Check for ros_core_ip
    if 'ros_core_ip' not in env_variables.keys():
        raise ValueError('environment variables should include ros_core_ip')
    ros_core_ip = env_variables['ros_core_ip']

    # Check for ros_core_port
    if 'ros_core_port' in env_variables.keys():
        ros_core_port = env_variables['ros_core_port']
    else:
        # Use Default port value
        ros_core_port = '11311'

    os.environ["ROS_MASTER_URI"] = 'http://%s:%d' % (ros_core_ip, ros_core_port)


class IMU(LSM6DS3_IMU):
    def __init__(self, ros_core_ip=None, **kwargs):
        super().__init__(self, **kwargs)
        if ros_core_ip is not None:
            set_environment_variables('environment_variables.yaml')

# Similarly do the same thing for all sensors
