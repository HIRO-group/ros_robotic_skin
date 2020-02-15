#!/usr/bin/env python
"""

"""
import rospy
import math
import intera_interface
from math import pi
import datetime

class SawyerPose(object):
    """
    This is the main SawyerPose class. The main reason this class needs to be overridden is that you can rospy.init_node
    once. Hence. You need to call this super method.
    """

    def __init__(self, limb="right"):
        # Create Publishers and Init Node
        rospy.init_node('sawyer_pose', anonymous=True)
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
        self.pose_string = ''

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
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        
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

    def publish_velocities(self, velocities, sleep):
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
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        
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

    def publish_trajectory(self, positions, velocities, sleep):
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
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        if len(velocities) != 7:
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        
        for joint_name, position, velocity in zip(self._limb.joint_names(), positions, velocities):
            self.positions[joint_name] = position
            self.velocities[joint_name] = velocity

        if not rospy.is_shutdown():
            try:
                self._limb.set_joint_trajectory(self._limb.joint_names(), positions, velocities)
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
            positions, _, pose_string = pose[0], pose[1], pose[2]
            self.pose_string = pose_string
            self.publish_positions(positions, sleep)

    def set_velocities_list(self, poses, sleep):
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
            _, velocities, pose_string = each_pose[0], each_pose[1], each_pose[2]
            self.pose_string = pose_string
            self.publish_velocities(velocities, sleep)

    def set_trajectory_list(self, poses, sleep):
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
        for each_pose in poses:
            positions, velocities, pose_string = each_pose[0], each_pose[1], each_pose[2]
            self.pose_string = pose_string
            self.publish_trajectory(positions, velocities, sleep)


if __name__ == "__main__":
    poses_list = [
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1'],
        [[-0.5, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, -0.5, 0, 0, 0, 0], 'Pose_2']
    ]
    sp = SawyerPose()
    while True:
        sp.set_trajectory_list(poses_list, sleep=1)
