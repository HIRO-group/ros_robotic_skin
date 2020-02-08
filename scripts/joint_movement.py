#!/usr/bin/env python

import argparse
import sys
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class PandaTrajectoryControl():
    def __init__(self, is_sim=True):
        """
        Panda Trajectory control class for sending 
        JointTrajectory messages to the real Franka Panda,
        or the simulated Panda.


        Parameters
        --------------
        `is_sim`: `bool`: If we are working with the real or simulated Panda
             
        """
        self.joint_dof_pub = rospy.Publisher('/joint_mvmt_dof', Int16, queue_size=1)
        self.calibration_pub = rospy.Publisher('/calibration_complete', Bool, queue_size=1)
        rospy.init_node('calibration_joint_mvmt_node', anonymous=True)
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.header.frame_id = '/base_link'

        trajectory_msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6',
                   'panda_joint7']

        point = JointTrajectoryPoint()
        point.positions = [0, 0, 0, 0, 0, 0, 0]
        point.time_from_start.secs = 1
        trajectory_msg.points = [point]
        self.trajectory_msg = trajectory_msg
        self.point = point
        self.is_sim = is_sim
        self.get_trajectory_publisher()
        # sending initial msg

    def get_trajectory_publisher(self):
        topic_string = '/panda_arm_controller/command' if self.is_sim else '/joint_trajectory_controller/command'
        self.trajectory_pub = rospy.Publisher(topic_string, 
                                                JointTrajectory, queue_size=1)


    def send_once(self):
        # TODO: Look up do we need to have one message to init the robot?
        # If I only send one message then the franka does not move.
        # TODO: This might not be the best starting postion for the robot to be in.
        # Think about if we are losing some inforamtion by using a completely vertical postion
        # for the start.
        # Move arm to starting position
        self.trajectory_pub.publish(self.trajectory_msg)
        rospy.sleep(1)
        self.trajectory_pub.publish(self.trajectory_msg)
        rospy.sleep(1)


    def spin(self):
        joint_int = 0
        while not rospy.is_shutdown():
            # Increment the Dof we are actuating here
            # Check if we have actuated every DoF, to end this script
            print(joint_int)
            if joint_int == 7:
                self.calibration_pub.publish(True)
                print('CALIBRATION COMPLETE')
                break

            self.point.positions[joint_int] = -1
            self.trajectory_msg.points = [self.point]

            # Publish this message so activation_matrix.py knows which Dof what actuated
            self.joint_dof_pub.publish(joint_int)
            rospy.sleep(1)

            # publish message to actuate the dof
            self.trajectory_pub.publish(self.trajectory_msg)
            rospy.sleep(5)

            joint_int+=1

        

if __name__ == '__main__':
    arg = sys.argv[1]
    is_simulation = True if arg == 'true' else False
    try:
        panda_control = PandaTrajectoryControl(is_simulation)
        panda_control.send_once()
        panda_control.spin()
    except rospy.ROSInterruptException:
        print('Exciting Franka Panda control process...')