#!/usr/bin/env python

import rospy 
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu

current_dof = None
num_dofs = 7
num_skinunits = 7
activation_matrix = np.zeros([num_skinunits, num_dofs])

def setDoF(data):
    # TODO: Is there a better way to share this data?
    global current_dof
    current_dof = data.data

def imu_mvmt(data):
    global activation_matrix
    # If no DoF has been set then we are just moving the robot to the starting pose
    if not current_dof == None:
        # Indexing though row, col
        print('DOF', current_dof, 'IMU', type(data))
        activation_matrix[data.data, current_dof] = 1

def calibration_complete(data):
    if data.data == True:
        print("****** ACTIVATION MATRIX ******")
        print(activation_matrix)

def listener():
    rospy.init_node('activation_matrix', anonymous=True)
    rospy.Subscriber('/joint_mvmt_dof', Int16, setDoF)
    rospy.Subscriber('/imu_activated', Int16, imu_mvmt)
    rospy.Subscriber('/calibration_complete', Bool, calibration_complete)

    # TODO: Subscribe to an imu Int16. publish the imu that was activated
    # TODO: Subscribe to bool msg that tells us if the activation matrix is complete
    rospy.spin()

if __name__ == '__main__':
    listener()