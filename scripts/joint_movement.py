#!/usr/bin/env python
import argparse
import sys
import math
import rospy
from std_msgs.msg import Bool, Int16
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rospkg
import os
import numpy as np

from PandaController import PandaController

class ActivityPandaController():
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

        self.joint_dof_pub = rospy.Publisher('/joint_mvmt_dof', Int16, queue_size=1)
        self.calibration_pub = rospy.Publisher('/calibration_complete', Bool, queue_size=1)
        rospy.init_node('calibration_joint_mvmt_node', anonymous=True)
        self.pos_mat = np.loadtxt(desired_positions_path)


        # trajectory_msg = JointTrajectory()
        # trajectory_msg.header.stamp = rospy.Time.now()
        # trajectory_msg.header.frame_id = '/base_link'

        # trajectory_msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6',
        #            'panda_joint7']

        # point = JointTrajectoryPoint()
        # point.positions = [0, 0, 0, 0, 0, 0, 0]
        # point.time_from_start.secs = 1
        # trajectory_msg.points = [point]
        # self.trajectory_msg = trajectory_msg
        # self.point = point
        # self.is_sim = is_sim
        # self.get_trajectory_publisher()
        # sending initial msg

    # def get_desired_positions(self):
    #     # gets the desired positions to move the panda.
    #     np.loadtxt()
    # def get_trajectory_publisher(self):
    #     topic_string = '/panda_arm_controller/command' if self.is_sim else '/joint_trajectory_controller/command'
    #     self.trajectory_pub = rospy.Publisher(topic_string, 
    #                                             JointTrajectory, queue_size=1)

    # def send_once(self):
    #     # TODO: Look up do we need to have one message to init the robot?
    #     # If I only send one message then the franka does not move.
    #     # TODO: This might not be the best starting postion for the robot to be in.
    #     # Think about if we are losing some inforamtion by using a completely vertical postion
    #     # for the start.
    #     # Move arm to starting position
    #     self.trajectory_pub.publish(self.trajectory_msg)
    #     rospy.sleep(1)
    #     self.trajectory_pub.publish(self.trajectory_msg)
    #     rospy.sleep(1)

    def spin(self):
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
            # Publish this message so activation_matrix.py knows which Dof what actuated
            self.joint_dof_pub.publish(joint_int)
            rospy.sleep(1)

            # publish message to actuate the dof
            self.controller.publish_positions(self.pos_mat[joint_int*2])
            rospy.sleep(5)

            # bring back to home position before next pose
            self.controller.publish_positions(self.pos_mat[(joint_int*2)+1])

            joint_int+=1

        

if __name__ == '__main__':
    arg = sys.argv[1]
    filename = sys.argv[2]
    is_simulation = True if arg == 'true' else False
    rospack = rospkg.RosPack()
    ros_robotic_skin_path = rospack.get_path('ros_robotic_skin')
    desired_positions_path = os.path.join(ros_robotic_skin_path, 'data', filename)
    try:
        panda_controller = PandaController(is_sim=is_simulation)
        activity_panda_control = ActivityPandaController(panda_controller, desired_positions_path,
                                is_sim=is_simulation)
        activity_panda_control.spin()
        # panda_control = PandaTrajectoryControl(desired_positions_path, is_simulation)
        # panda_control.send_once()
        # panda_control.spin()
    except rospy.ROSInterruptException:
        print('Exiting Franka Panda control process...')