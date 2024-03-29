#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

import numpy as np


class ImuListener():

    def __init__(self, num_imus):
        """
        ImuListener class for registering when IMUs are
        activated; that is, the IMU is moved enough.

        Arguments
        ----------
        `num_imus`: `int`
            Amount of IMUs on the robot

        `acc_thresh`: `float`
            The default threshold used to determine the
            minimal difference between two consecutive
            IMU readings to constitute an 'activated' IMU.

        Returns
        ----------
        returns: None
        """
        # create subscribers for num_imus topics
        self.norms = np.zeros(num_imus)
        for i in range(1, num_imus):
            rospy.Subscriber('imu_data{}'.format(i), Imu, self.imu_callback)

    def imu_callback(self, data):
        """
        A ROS callback for checking IMU data and
        publishing a message if a certain IMU is
        activated.

        Arguments
        ----------
        `data`: `Imu`
            `Imu` message from the IMU

        Returns
        ----------
        returns: None
        """
        A = data.linear_acceleration
        norm = np.linalg.norm([A.x, A.y, A.z])
        # rospy.loginfo('{},  {}'.format(data.header.frame_id, norm))

        i = int(data.header.frame_id[-1])
        self.norms[i] = norm


if __name__ == '__main__':
    # create the IMU listener, and spin.
    rospy.init_node('skin_calibration', anonymous=True)

    num_imus = 7
    imu_listener = ImuListener(num_imus)
    publishers = []
    for i in range(num_imus):
        publishers.append(rospy.Publisher('imu_norm{}'.format(i), Float64, queue_size=10))
    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        for i in range(num_imus):
            publishers[i].publish(imu_listener.norms[i])
        r.sleep()
