#!/usr/bin/env python

import rospy 
import rospkg

import numpy as np
from std_msgs.msg import Bool, Int16 
from sensor_msgs.msg import Imu
import numpy as np

from robotic_skin.algorithm.convert_to_lowertriangular_matrix import ConvertToLT

import os
import errno
import sys


class ActivityMatrix():
    def __init__(self, ros_robotic_skin_path, num_dofs=7, num_skinunits=7):
        self.num_dofs = num_dofs
        self.num_skinunits = num_skinunits
        self.save_dir = os.path.join(ros_robotic_skin_path, 'data')
        try: 
            os.makedirs(self.save_dir)
        except OSError as e:
            if e.errno == errno.EEXIST:
                print('Data folder already exists!')

        self.activity_matrix = np.zeros([self.num_skinunits, self.num_dofs])
        self.current_dof = None

    def spin(self):
        rospy.init_node('activity_matrix', anonymous=True)
        rospy.Subscriber('/joint_mvmt_dof', Int16, self.set_dof)
        rospy.Subscriber('/imu_activated', Int16, self.imu_mvmt)
        rospy.Subscriber('/calibration_complete', Bool, self.calibration_complete)
        rospy.spin()

    def set_dof(self, data):
        # set current dof being actuated
        self.current_dof = data.data

    def imu_mvmt(self, data):
        if self.current_dof is not None and self.current_dof in range(7):
            # If no DoF has been set then we are just moving the robot to the starting pose
            print('DOF:', self.current_dof, 'IMU:', type(data))
            print(data.data)
            self.activity_matrix[data.data, self.current_dof] = 1
            print(self.activity_matrix)


    def calibration_complete(self, data):
        if data.data == True:
            print("****** ACTIVITY MATRIX ******")
            print(self.activity_matrix)
            _, final_matrix, _, _, _ = ConvertToLT(self.activity_matrix).get_lt_matrix_infos()
            # convert the activity matrix to upper triangular

            save_mat_path = os.path.join(self.save_dir, 'activity_matrix.txt')
            np.savetxt(save_mat_path, final_matrix)

if __name__ == '__main__':
    # get the path to the ros_robotic_skin package
    rospack = rospkg.RosPack()
    ros_robotic_skin_path = rospack.get_path('ros_robotic_skin')
    try:
        activity_matrix = ActivityMatrix(ros_robotic_skin_path)
        activity_matrix.spin()
    except rospy.ROSInterruptException:
        print('Exiting Panda control process...')
    