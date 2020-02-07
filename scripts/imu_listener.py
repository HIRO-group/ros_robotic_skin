#!/usr/bin/env python

import rospy 
from std_msgs.msg import String
from sensor_msgs.msg import Imu

import numpy as np

prev_imu_link2 = None
prev_imu_link3 = None
acc_thresh = 1.0

class ImuListener():

    def __init__(self, num_imus):

        self.prev_imu_matrix = np.full((num_imus, 3), np.nan)

        rospy.init_node('skin_calibration', Imu, self.imu_callback)
        self.imu_mvmt_pub = rospy.Publisher('/imu_activated', Int16)
        self.imu_mvmt_pub = rospy.Subscriber('/imu_activated', Int16, imu_mvmt) 
        # create subscribers for num_imus topics
        for i in range(num_imus):
            rospy.Subscriber('imu_data{}'.format(i))
        rospy.spin()

    def imu_callback(self, data):
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
            if x_diff > acc_thresh or y_diff > acc_thresh or z_diff > acc_thresh:
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

            self.prev_imu_matrix[imu_num,0] = data.linear_acceleration.x
            self.prev_imu_matrix[imu_num,1] = data.linear_acceleration.y
            self.prev_imu_matrix[imu_num,2] = data.linear_acceleration.z

def callback(data):
    global prev_imu_link2
    global prev_imu_link3
    
    # If we don't have any data on the prev state set the previous measurments
    if data.header.frame_id == 'imu_link3' and prev_imu_link3 == None:
        prev_imu_link3 = data.linear_acceleration
    elif data.header.frame_id == 'imu_link2' and prev_imu_link2 == None:
        prev_imu_link2 = data.linear_acceleration


    # Compare the current and prev and see if we have a change that meets the threshold
    elif data.header.frame_id == 'imu_link3':
        x_diff = abs(prev_imu_link3.x - data.linear_acceleration.x)
        y_diff = abs(prev_imu_link3.y - data.linear_acceleration.y)
        z_diff = abs(prev_imu_link3.z - data.linear_acceleration.z)
        if x_diff > acc_thresh or y_diff > acc_thresh or z_diff > acc_thresh:
            # the threshold is large enough to publish an imu activated message
            print('*************DIFFF***************')
            print(data.header.frame_id)
            print('NEW DATA')
            print(data.linear_acceleration)
            print('OLD DATA')
            print(prev_imu_link3)
            print('')
        prev_imu_link3 = data.linear_acceleration
    
    #prev_imu_link2 = data.linear_acceleration
    
def listener():
    rospy.init_node('skin_calibration', anonymous=True)
    rospy.Subscriber('imu_data2', Imu, callback)
    rospy.Subscriber('imu_data3', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
