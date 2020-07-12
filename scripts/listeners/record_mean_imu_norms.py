#!/usr/bin/env python

import sys
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

import numpy as np


class ImuListener():

    def __init__(self, num_imus, axis):
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
        self.num_imus = num_imus
        self.axis = axis
        self.values = []
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

        if self.axis == 'x':
            value = A.x
        elif self.axis == 'y':
            value = A.y
        elif self.axis == 'z':
            value = A.z
        else:
            raise ValueError('No such key as ' + self.axis)
        
        self.values.append(value)
        rospy.loginfo('{},  {}'.format(data.header.frame_id, np.mean(self.values)))


if __name__ == '__main__':
    # create the IMU listener, and spin.
    rospy.init_node('record_norm', anonymous=True)
    axis = sys.argv[1]

    num_imus = 7
    imu_listener = ImuListener(num_imus, axis)
    rospy.spin()