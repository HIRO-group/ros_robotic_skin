#!/usr/bin/env python
"""
This is a sawyer pose library, this is comprised of three main types of classes
1) General Utilities: defs utilized by all classes
2) Static Defs: All defs starting with static. Used only for static data collection
3) dynamic Defs: All defs starting with dynamic. Used for dynamic data collection only
Each Pose should be defined in this way
poses_list = [
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1'],
        [[-0.5, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, -0.5, 0, 0, 0, 0], 'Pose_2']
    ]
Which is basically:
poses_list = [
        [[Position List_1], [Velocity_list_1], 'Pose_name_1'],
        [[Position List_2], [Velocity_list_2], 'Pose_name_2']
    ]
"""
import datetime
import rospy
import math
from math import pi
import numpy as np

import intera_interface

class SawyerController(object):
    """
    This is the main SawyerPose class. The main reason this class needs to be overridden is that you can rospy.init_node
    once. Hence. You need to call this super method.
    """

    def __init__(self, limb="right"):
        # Create Publishers and Init Node
        rospy.init_node('sawyer_controller', anonymous=True)
        # Set Initial position of the robot
        self._limb = intera_interface.Limb(limb)
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()

        self.positions = {name: 0.0 for name in self._limb.joint_names()}
        self.velocities = {name: 0.0 for name in self._limb.joint_names()}

        try:
            self._limb.set_joint_positions(self.positions)
        except rospy.ROSInterruptException:
            print('Set Joint Velocities Failed')

        self.sleep_time_static = rospy.get_param('/static_sleep_time')
        self.r = rospy.Rate(rospy.get_param('/dynamic_frequency'))
        self.pose_name = ''

    def publish_positions(self, positions, sleep):
        """
        Set joint positions of the sawyer 

        Arguments
        ----------
        positions: list
            Set the joint positions according to the list that you get
        sleep: float
            Set how long it should wait after commanding a command

        Returns
        ----------
        return: None
        """
        if len(positions) != 7:
            raise Exception("The length of input list should be 7, as sawyer has 7 joints.")
        
        for joint_name, position in zip(self._limb.joint_names(), positions):
            self.positions[joint_name] = position

        if not rospy.is_shutdown():
            try:
                self._limb.move_to_joint_positions(self.positions, timeout=sleep)
            except rospy.ROSInterruptException:
                print('Set Joint Positions Failed')

            if sleep:
                rospy.sleep(self.sleep_time_static)
            else:
                self.r.sleep()

    def publish_velocities(self, velocities, sleep=None):
        """
        Set joint velocities of the sawyer 

        Arguments
        ----------
        positions: list
            Set the joint velocities according to the list that you get
        sleep: float
            Set how long it should wait after commanding a command
        
        Returns
        ----------
        return: None
        """
        if len(velocities) != 7:
            raise Exception("The length of input list should be 7, as sawyer has 7 arms")
        
        for joint_name, velocity in zip(self._limb.joint_names(), velocities):
            self.velocities[joint_name] = velocity
        
        if not rospy.is_shutdown():
            try:
                self._limb.set_joint_velocities(self.velocities)
            except rospy.ROSInterruptException:
                print('Set Joint Velocities Failed')
            
            if sleep:
                rospy.sleep(self.sleep_time_static)
            else:
                self.r.sleep()

    def publish_trajectory(self, positions, velocities, accelerations, sleep):
        """
        Set joint trajectory (positions and velocities) of the sawyer 

        Arguments
        ----------
        positions: list
            Set the joint positions according to the list that you get
        velocities: list
            Set the joint velocities according to the list that you get
        sleep: float
            Set how long it should wait after commanding a command
        
        Returns
        ----------
        return: None
        """
        if len(positions) != 7:
            raise Exception("The length of input list should be 7, as sawyer has 7 arms")
        if len(velocities) != 7:
            raise Exception("The length of input list should be 7, as sawyer has 7 arms")
        
        for joint_name, position, velocity in zip(self._limb.joint_names(), positions, velocities):
            self.positions[joint_name] = position
            self.velocities[joint_name] = velocity

        if not rospy.is_shutdown():
            try:
                self._limb.set_joint_trajectory(self._limb.joint_names(), positions, velocities, accelerations)
            except rospy.ROSInternalException:
                print('Set Joint Trajectory Failed')

            if sleep:
                rospy.sleep(self.sleep_time_static)
            else:
                self.r.sleep()

    # End General Utilities
    def set_positions_list(self, poses, sleep):
        """
        Set joint poses (positions)
        
        Arguments
        ----------
        poses:  Special Data Structure
            Loop through all the positions and velocities in the poses
            poses = [pose, pose, ..., pose]
            pose = [positions, velocities, string]
        sleep: float
            Set how long it should wait after commanding a command

        Returns
        --------
        return: None
        """
        for pose in poses:
            positions, _, pose_name = pose[0], pose[1], pose[2]
            self.pose_name = pose_name
            self.publish_positions(positions, sleep)

    def set_velocities_list(self, poses, sleep=None):
        """
        Set poses (velocities)
        
        Arguments
        ----------
        poses:  Special Data Structure
            Loop through all the positions and velocities in the poses
            poses = [pose, pose, ..., pose]
            pose = [positions, velocities, string]
        sleep: float
            Set how long it should wait after commanding a command

        Returns
        --------
        return: None
        """
        for each_pose in poses:
            _, velocities, pose_name = each_pose[0], each_pose[1], each_pose[2]
            self.pose_name = pose_name
            self.publish_velocities(velocities, sleep)

    def set_trajectory_list(self, poses, sleep=None):
        """
        Set poses (positions and velocities)
        
        Arguments
        ----------
        poses:  Special Data Structure
            Loop through all the positions and velocities in the poses
            poses = [pose, pose, ..., pose]
            pose = [positions, velocities, string]
        sleep: float
            Set how long it should wait after commanding a command

        Returns
        --------
        return: None
        """
        # TODO: add accelerations
        for each_pose in poses:
            positions, velocities, pose_name = each_pose[0], each_pose[1], each_pose[2]
            accelerations = np.zeros(7)
            self.pose_name = pose_name
            self.publish_trajectory(positions, velocities, accelerations, sleep)


if __name__ == "__main__":
    poses_list = [
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1'],
        [[-0.5, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, -0.5, 0, 0, 0, 0], 'Pose_2']
    ]
    controller = SawyerController()
    while True:
        controller.set_trajectory_list(poses_list, sleep=1)
