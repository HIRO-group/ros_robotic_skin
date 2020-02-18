#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import os
import sys
import errno


from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import JointState

# a class to capture key poses of the panda 

class CapturePose():

    def __init__(self, save_path, joints=7, is_sim=False):

        # publisher for when the user wants the pose to be captured

        # get max number of poses from ros params
        self.is_sim = is_sim
        rospy.init_node("capture_poses", anonymous=True)
        rospy.Subscriber("/is_in_captured_pose", Bool, self.get_is_captured_callback)
        rospy.Subscriber("/zero_g_pose_num", Int16, self.get_pose_num_callback)
        
        topic_string = "/joint_states" if is_sim else "/franka_state_controller/joint_states" 
        rospy.Subscriber(topic_string, JointState, self.capture_pose_callback)
        self.total_num_poses = rospy.get_param("/zero_g_poses", default=11)

        self.captured_positions = np.zeros((self.total_num_poses, joints))
        self.is_in_captured_pose = False
        self.save_dir = os.path.join(save_path, 'data')

        try: 
            os.makedirs(self.save_dir)
        except OSError as e:
            if e.errno == errno.EEXIST:
                print('Data folder already exists!')

        self.save_path = os.path.join(self.save_dir, 'positions.txt')
        self.pose_num = 0
        # get the total number of poses
        rospy.spin()

    def get_pose_num_callback(self, data):
        self.pose_num = data.data

    def get_is_captured_callback(self, data):
        self.is_in_captured_pose = data.data

    def capture_pose_callback(self, data):
        if self.is_in_captured_pose:
            # get the current pose from the panda
            joint_positions = np.array(data.position)
            # save to txt file 
            arr_pos = joint_positions[2:] if self.is_sim else joint_positions

            self.captured_positions[self.pose_num] = arr_pos
            # if last pose is met, we're done
            if self.pose_num == self.total_num_poses - 1:
                np.savetxt(self.save_path, self.captured_positions)

if __name__ == "__main__":

    rospack = rospkg.RosPack()
    ros_robotic_skin_path = rospack.get_path('ros_robotic_skin')

    arg = sys.argv[1]
    is_sim = True if arg == 'true' else False

    cp = CapturePose(ros_robotic_skin_path, is_sim=is_sim)