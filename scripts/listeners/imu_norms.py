#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

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
        self.norms = []
        rospy.init_node('skin_calibration', anonymous=True)
        # create subscribers for num_imus topics
        for i in range(1, num_imus):
            rospy.Subscriber('imu_data{}'.format(i), Imu, self.imu_callback)

        rospy.spin()

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
        self.norms.append(norm)
        rospy.loginfo('{},  {},  {}'.format(data.header.frame_id, norm, np.mean(self.norms)))


if __name__ == '__main__':
    # create the IMU listener, and spin.
    imu_listener = ImuListener(num_imus=7)
