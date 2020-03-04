#!/usr/bin/env python

import rospy 
from std_msgs.msg import String, Int16, Bool
from sensor_msgs.msg import Imu

import numpy as np

class ImuListener():

    def __init__(self, num_imus, acc_thresh=0.3):
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

        self.prev_imu_matrix = np.full((num_imus, 3), np.nan)
        self.acc_thresh = acc_thresh
        rospy.init_node('skin_calibration', anonymous=True)
        self.imu_mvmt_pub = rospy.Publisher('/imu_activated', Int16, queue_size=1)
        self.imu_num_msg = Int16()
        # create subscribers for num_imus topics
        for i in range(num_imus):
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
        imu_num = int(data.header.frame_id[-1])
        is_nan = np.isnan(self.prev_imu_matrix[imu_num])
        # if the measurements are unitialized, initialize them
        if np.all(is_nan):
            self.prev_imu_matrix[imu_num,0] = data.linear_acceleration.x
            self.prev_imu_matrix[imu_num,1] = data.linear_acceleration.y
            self.prev_imu_matrix[imu_num,2] = data.linear_acceleration.z
        else:
            # data from previous callback was already initialized
            prev_imu_acc = self.prev_imu_matrix[imu_num] 
            x_diff = np.abs(prev_imu_acc[0] - data.linear_acceleration.x)
            y_diff = np.abs(prev_imu_acc[1] - data.linear_acceleration.y)
            z_diff = np.abs(prev_imu_acc[2] - data.linear_acceleration.z)
            total_diff = np.linalg.norm(np.array([x_diff, y_diff, z_diff]))
            if total_diff > self.acc_thresh:
                print('*************DIFF***************')
                print(data.header.frame_id)
                print('NEW DATA')
                print('x: {}'.format(data.linear_acceleration.x))
                print('y: {}'.format(data.linear_acceleration.y))
                print('z: {}'.format(data.linear_acceleration.z))
                print('OLD DATA')
                print('x: {}'.format(self.prev_imu_matrix[imu_num,0]))
                print('y: {}'.format(self.prev_imu_matrix[imu_num,1]))
                print('z: {}'.format(self.prev_imu_matrix[imu_num,2]))
                print('')
                # publish the imu message if the acceleration is above a certain threshold.
                self.imu_mvmt_pub.publish(Int16(imu_num))
       
            self.prev_imu_matrix[imu_num,0] = data.linear_acceleration.x
            self.prev_imu_matrix[imu_num,1] = data.linear_acceleration.y
            self.prev_imu_matrix[imu_num,2] = data.linear_acceleration.z

if __name__ == '__main__':
    # create the IMU listener, and spin.
    imu_listener = ImuListener(num_imus=7)
