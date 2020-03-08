#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Bool, Int16

import rospkg
import os
import numpy as np

from PandaController import PandaController
from SawyerController import SawyerController


class ActivityMatrixController():
    def __init__(self, controller, desired_positions_path, is_sim=True):
        """
        Panda Trajectory control class for sending
        JointTrajectory messages to the real Franka Panda,
        or the simulated Panda.


        Parameters
        --------------
        `is_sim`: `bool`: If we are working with the real or simulated Panda

        """

        # get the controller, either sawyer or panda
        self.controller = controller

        self.joint_dof_pub = rospy.Publisher(
                '/joint_mvmt_dof', Int16, queue_size=1)
        self.calibration_pub = rospy.Publisher(
                '/calibration_complete', Bool, queue_size=1)
        # rospy.init_node('calibration_joint_mvmt_node', anonymous=True)
        self.pos_mat = np.loadtxt(desired_positions_path)
        print(self.pos_mat)

    def spin(self):
        """
        Goes through all of the joints and positions.

        Arguments
        ---------
        None

        Returns
        ----------
        returns: None
        """
        joint_int = 0
        self.controller.send_once()
        while not rospy.is_shutdown():
            # Increment the Dof we are actuating here
            # Check if we have actuated every DoF, to end this script

            # go through every pose
            print(joint_int)
            if joint_int == 7:
                self.calibration_pub.publish(True)
                print('CALIBRATION COMPLETE')
                break
            # Publish this message so activation_matrix.py
            # knows which Dof what actuated
            self.joint_dof_pub.publish(joint_int)
            rospy.sleep(1)

            # publish message to actuate the dof
            self.controller.publish_positions(self.pos_mat[joint_int*2], 2)
            rospy.sleep(5)
            # publish -1 so nothing is put into activity matrix
            self.joint_dof_pub.publish(-1)
            # bring back to home position before next pose
            self.controller.publish_positions(self.pos_mat[(joint_int*2)+1], 2)

            joint_int += 1


if __name__ == '__main__':
    arg = sys.argv[1]
    filename = sys.argv[2]
    robot_type = sys.argv[3]
    is_sim = True if arg == 'true' else False

    if robot_type == 'sawyer' and is_sim is False:
        raise Exception('Real Sawyer support is currently not supported.')
    # controller was determined from roslaunch file.
    controller = PandaController(is_sim=is_sim) if robot_type == 'panda' \
        else SawyerController()
    rospack = rospkg.RosPack()
    ros_robotic_skin_path = rospack.get_path('ros_robotic_skin')
    desired_positions_path = os.path.join(
            ros_robotic_skin_path, 'data', filename)
    try:
        activity_matrix_control = \
            ActivityMatrixController(controller, desired_positions_path,
                                     is_sim=is_sim)
        activity_matrix_control.spin()

    except rospy.ROSInterruptException:
        print('Exiting Franka Panda control process...')
