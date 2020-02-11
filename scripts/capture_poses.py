#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import JointState

# a class to capture key poses of the panda 

class CapturePose():

    def __init__(self, poses=7, joints=7):

        # publisher for when the user wants the pose to be captured
        topic_string = "/franka_state_controller/joint_states"
        self.joint_pos_sub = rospy.Subscriber(topic_string, JointState, self.capture_pose_callback)
        self.is_in_captured_pose = False
        self.poses = poses
        self.captured_positions = np.zeros((poses, joints))
        self.pose_num = 0

    def capture_poses(self):
        for i in range(self.poses):
            self.pose_num = i
            self.is_in_captured_pose = False
            print("Please move the robot to the desired pose.")
            raw_input("Press enter to capture the pose: ")
            self.is_in_captured_pose = True
            print("==========================")
            print("Please wait 2 seconds in this pose...")
            # sleep for 5 seconds
            rospy.sleep(2)
        np.savetxt(self.captured_positions)


    def capture_pose_callback(self, data):
        if self.is_in_captured_pose:
            # get the current pose from the panda
            joint_positions = np.array(data.data.position)
            # save to txt file 
            self.captured_positions[self.pose_num] = joint_positions

if __name__ == "__main__":
    cp = CapturePose()
    cp.capture_poses()