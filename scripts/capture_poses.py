#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import os

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import JointState

# a class to capture key poses of the panda 

class CapturePose():

    def __init__(self, save_path, poses=11, joints=7):

        # publisher for when the user wants the pose to be captured
        rospy.init_node('capture_poses', anonymous=True)

        topic_string = "/franka_state_controller/joint_states"
        self.joint_pos_sub = rospy.Subscriber(topic_string, JointState, self.capture_pose_callback)
        self.is_in_captured_pose = False
        self.poses = poses
        self.captured_positions = np.zeros((poses, joints))
        self.save_path = os.path.join(save_path, 'data', 'positions.txt')
        rospy.spin()

    def capture_pose_callback(self, data):
        if self.is_in_captured_pose:
            # get the current pose from the panda
            joint_positions = np.array(data.position)
            # save to txt file 
            self.captured_positions[self.pose_num] = joint_positions

if __name__ == "__main__":

    rospack = rospkg.RosPack()
    ros_robotic_skin_path = rospack.get_path('ros_robotic_skin')

    cp = CapturePose(ros_robotic_skin_path)
    cp.capture_poses()