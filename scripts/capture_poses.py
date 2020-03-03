#!/usr/bin/env python

import rospy
import rospkg
import numpy as np
import os
import sys
import errno


from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import JointState

import intera_interface

# a class to capture key poses of the panda 

class CapturePose():

    def __init__(self, save_path, joints=7, is_sim=False, robot_type="panda", 
                    filename="positions.txt"):
        """
        Creates a CapturePose object. 

        Arguments
        ----------
        `save_path`: `str`
            The path of where the `ros_robotic_skin` package skin is.

        `joints`: `int`
            The amount of joints

        `is_sim`: `bool`
            If we are in simulation or real life

        `robot_type`: `str`
            The robot type, either `"panda"` or `"sawyer"`

        `filename`: `str`
            The filename to save the captured poses to

        Returns
        ----------
        returns: None
        """

        # publisher for when the user wants the pose to be captured

        # get max number of poses from ros params
        self.is_sim = is_sim
        rospy.init_node("capture_poses", anonymous=True)
        rospy.Subscriber("/is_in_captured_pose", Bool, self.get_is_captured_callback)
        rospy.Subscriber("/zero_g_pose_num", Int16, self.get_pose_num_callback)
        # topic string is based on simulation and panda type
        if robot_type == 'sawyer':
            # work with intera interface, no topic string here
            self.robot_type = 'sawyer'
            self._limb = intera_interface.Limb("right")
        else:
            self.robot_type = 'panda'
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

        self.save_path = os.path.join(self.save_dir, filename)
        self.pose_num = 0
        # get the total number of poses

    def get_pose_num_callback(self, data):
        """
        Pose number callback from ROS Subscriber 

        Arguments
        ----------
        `data`: `Int16`
            The data of the joint number

        Returns
        ----------
        returns: None
        """
        self.pose_num = data.data

    def get_is_captured_callback(self, data):
        """
        Sees if the robot is in a pose that needs to be
        captured and eventually sent out to a file.

        Arguments
        ----------
        `data`: `Bool`
            If the robot is in a captured pose.

        Returns
        ----------
        returns: None
        """
        self.is_in_captured_pose = data.data
        # if is sawyer, make a query to the sawyer api
        if self.robot_type == 'sawyer' and self.is_in_captured_pose:
            # if we are dealing with sawyer, we don't need to rely on callbacks
            joint_states_dict = self._limb.joint_angles()
            for joint_name in joint_states_dict:
                # get the joint number
                joint_num = int(joint_name[-1])
                self.captured_positions[self.pose_num, joint_num] = joint_states_dict[joint_name] 
                if self.pose_num == self.total_num_poses - 1:
                    np.savetxt(self.save_path, self.captured_positions)


    def capture_pose_callback(self, data):
        """
        A ROS callback for getting all of the joint states

        Arguments
        ----------
        `data`: `JointState`
            joint states of the robot

        Returns
        ----------
        returns: None
        """
        
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

    # get the path of ros_robotic_skin
    rospack = rospkg.RosPack()
    ros_robotic_skin_path = rospack.get_path('ros_robotic_skin')

    arg = sys.argv[1]
    is_sim = True if arg == 'true' else False

    robot_type = sys.argv[2]
    filename = sys.argv[3]

    if robot_type == 'sawyer' and is_sim == False:
        raise Exception('Real Sawyer support is currently not supported.')

    try:
        # keep capturing the poses.
        cp = CapturePose(ros_robotic_skin_path, is_sim=is_sim, 
                            robot_type=robot_type, filename=filename)
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Exiting Sawyer control process...')