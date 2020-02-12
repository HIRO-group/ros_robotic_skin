#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import os

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import JointState

# a class to capture key poses of the panda 

class WaitPose():

    def __init__(self, save_path, poses=11, joints=7):

        # publisher for when the user wants the pose to be captured
        rospy.init_node('capture_poses', anonymous=True)
        topic_string = "/franka_state_controller/joint_states"
        
        self.is_in_captured_pose_sub = rospy.Publisher("is_in_captured_pose", Bool)
        self.poses = poses
        self.pose_num = 0
        rospy.spin()

    def capture_poses(self):
        for i in range(self.poses):
            self.pose_num = i
            self.is_in_captured_pose_sub.publish(Bool(False))
            print("Please move the robot to the desired pose.")
            raw_input("Press enter to capture the pose: ")
            self.is_in_captured_pose_sub.publish(Bool(True))
            print("==========================")
            print("Please wait 2 seconds in this pose...")
            # sleep for 5 seconds
            rospy.sleep(2)
        np.savetxt(self.save_path, self.captured_positions)


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