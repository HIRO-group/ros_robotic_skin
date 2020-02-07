#!/usr/bin/env python

import rospy 
from std_msgs.msg import String
from sensor_msgs.msg import Imu

prev_imu_link2 = None
prev_imu_link3 = None
acc_thresh = 1.0

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
