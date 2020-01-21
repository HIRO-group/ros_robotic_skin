#!/usr/bin/env python

import rospy 
import numpy as np
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu

current_dof = None
num_dofs = 7
num_skinunits = 7
activation_matrix = np.zeros([num_skinunits, num_dofs])

def callback(data):
    global current_dof
    global activation_matrix
    current_dof = data

    print('****** INT16 ******')
    print(data)
    print('')

def listener():
    rospy.init_node('activation_matrix', anonymous=True)
    rospy.Subscriber('/joint_mvmt_dof', Int16, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()