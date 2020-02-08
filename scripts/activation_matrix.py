#!/usr/bin/env python

import rospy 
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu

import numpy as np


class ActivationMatrix():
    def __init__(self, num_dofs=7, num_skinunits=7):
        self.num_dofs = num_dofs
        self.num_skinunits = num_skinunits
        self.activation_matrix = np.zeros([self.num_skinunits, self.num_dofs])
        self.current_dof = None

    def spin(self):
        rospy.init_node('activation_matrix', anonymous=True)
        rospy.Subscriber('/joint_mvmt_dof', Int16, self.set_dof)
        rospy.Subscriber('/imu_activated', Int16, self.imu_mvmt)
        rospy.Subscriber('/calibration_complete', Bool, self.calibration_complete)
        rospy.spin()

    def set_dof(self, data):
        self.current_dof = data.data

    def imu_mvmt(self, data):
        if self.current_dof is not None:
            self.activation_matrix[data.data, self.current_dof] = 1
            print(self.activation_matrix)


    def calibration_complete(self, data):
        if data.data == True:
            print("****** ACTIVATION MATRIX ******")
            print(self.activation_matrix)
            np.savetxt('matrix.txt', self.activation_matrix)

# current_dof = None
# num_dofs = 7
# num_skinunits = 7
# activation_matrix = np.zeros([num_skinunits, num_dofs])

# def setDoF(data):
#     # TODO: Is there a better way to share this data?
#     global current_dof
#     current_dof = data.data

# def imu_mvmt(data):
#     global activation_matrix
#     # If no DoF has been set then we are just moving the robot to the starting pose
#     if current_dof is not None:
#         # Indexing though row, col
#         # print('DOF', current_dof, 'IMU', type(data))
#         # print(data.data)
#         activation_matrix[data.data, current_dof]+=1
#         print(activation_matrix)

# def calibration_complete(data):
#     if data.data == True:
#         print("****** ACTIVATION MATRIX ******")
#         print(activation_matrix)
#         np.savetxt('matrix.txt', activation_matrix)

# def listener():
#     rospy.init_node('activation_matrix', anonymous=True)
#     rospy.Subscriber('/joint_mvmt_dof', Int16, setDoF)
#     rospy.Subscriber('/imu_activated', Int16, imu_mvmt)
#     rospy.Subscriber('/calibration_complete', Bool, calibration_complete)

#     # TODO: Subscribe to an imu Int16. publish the imu that was activated
#     # TODO: Subscribe to bool msg that tells us if the activation matrix is complete
#     rospy.spin()

if __name__ == '__main__':
    mat = ActivationMatrix()
    mat.spin()