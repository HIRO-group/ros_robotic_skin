#!/usr/bin/env python

import rospy
import rospkg

import numpy as np
from std_msgs.msg import Bool, Int16

from robotic_skin.algorithm.convert_to_lowertriangular_matrix \
                import ConvertToLT

import os
import errno


class ActivityMatrix():
    def __init__(self, ros_robotic_skin_path, num_dofs=7, num_skinunits=7):
        """
        Activity Matrix class for a generalized robot arm

        Arguments
        ----------
        `ros_robotic_skin_path`: `str`
            The absolute filepath to this ros package.

        `num_dofs`: `int`
            The amount of degrees of freedom in the robot.

        `num_skinunuts`: `int`
            The amount of skin units on the robot

        Returns
        ----------
        returns: nothing
        """
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
        """
        The spin function for the ActivityMatrix class. Initializes
        the ROS node and all of the subscribers.

        Arguments
        ----------
        None.

        Returns
        ----------
        returns: None
        """
        rospy.init_node('activity_matrix', anonymous=True)
        rospy.Subscriber('/joint_mvmt_dof', Int16, self.set_dof)
        rospy.Subscriber('/imu_activated', Int16, self.imu_mvmt)
        rospy.Subscriber('/calibration_complete', Bool,
                         self.calibration_complete)
        rospy.spin()

    def set_dof(self, data):
        """
        A ROS callback for setting the current DOF
        that is being actuated.

        Arguments
        ----------
        `data`: `Int16`
            `Int16` message of the actuated DOF

        Returns
        ----------
        returns: None
        """
        # set current dof being actuated
        self.current_dof = data.data

    def imu_mvmt(self, data):
        """
        A ROS callback for setting a value in the activity
        matrix based on if it was activated or not.

        Arguments
        ----------
        `data`: `Int16`
            `Int16` message of the activated IMU

        Returns
        ----------
        returns: None
        """
        if self.current_dof is not None and self.current_dof in range(7):
            # If no DoF has been set then we are just moving the
            # robot to the starting pose
            print('DOF:', self.current_dof, 'IMU:', type(data))
            print(data.data)
            self.activity_matrix[data.data, self.current_dof] = 1
            print(self.activity_matrix)

    def calibration_complete(self, data):
        """
        A ROS callback for saving the final
        activity matrix to a file when calibration
        is complete.

        Arguments
        ----------
        `data`: `Bool`
            `Bool` message that determines if calibration
            is complete.

        Returns
        ----------
        returns: None
        """
        if data.data is True:
            print("****** ACTIVITY MATRIX ******")
            print(self.activity_matrix)
            _, final_matrix, _, _, _ = ConvertToLT(  # noqa: F841
                    self.activity_matrix).get_lt_matrix_infos()
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
