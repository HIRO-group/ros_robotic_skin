#!/usr/bin/env python
"""
This is a panda pose library, this is comprised of three main types of classes
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
import rospy
import math
from std_msgs.msg import Int16, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from math import pi
import datetime


class PandaController(object):
    """
    This is the main PandaPose class. The main reason this class needs to be overridden is that you can rospy.init_node
    once. Hence. You need to call this super method.
    """

    def __init__(self, is_sim=True):
        # Create Publishers and Init Node
        self.is_sim = is_sim
        self.trajectory_pub = self.get_trajectory_publisher(is_sim)
        rospy.init_node('panda_pose', anonymous=True)

        # Prepare msg to send 
        self.msg = JointTrajectory()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = '/base_link'
        self.msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                                'panda_joint6', 'panda_joint7']

        # Set Initial position of the robot
        self.point = JointTrajectoryPoint()
        self.point.positions = [0, 0, 0, 0, 0, 0, 0]
        self.point.velocities = [0, 0, 0, 0, 0, 0, 0]
        self.point.time_from_start.secs = 1
        self.msg.points = [self.point]
        # TODO: Look up do we need to have one message to init the robot?
        # If I only send one message then the franka bottom most joint does not move in simulation.
        # Need to check if same thing happens in real robot too. @peasant98 can you please confirm this?
        # TODO: This might not be the best starting position for the robot to be in.
        # Think about if we are losing some information by using a completely vertical position
        # for the start.
        # Move arm to starting position
        self.trajectory_pub.publish(self.msg)
        rospy.sleep(1)
        self.trajectory_pub.publish(self.msg)
        rospy.sleep(1)

        self.sleep_time_static = rospy.get_param('/static_sleep_time')
        self.r = rospy.Rate(rospy.get_param('/dynamic_frequency'))
        self.pose_string = ''

    # General Utilities
    def get_trajectory_publisher(self, is_sim):
        topic_string = '/panda_arm_controller/command' if is_sim else '/joint_trajectory_controller/command'
        return rospy.Publisher(topic_string, JointTrajectory, queue_size=1)

    def publish_positions(self, positions, sleep):
        """
        Set joint positions of the panda 

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
        for index, _ in enumerate(self.point.positions):
            self.point.positions[index] = positions[index]
        
        self._publish_all_values(sleep)        

    def publish_velocities(self, velocities, sleep):
        """
        Set joint velocities of the panda 

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
        for index, _ in enumerate(velocities):
            self.point.velocities[index] = velocities[index]
        
        self._publish_all_values(sleep)        

    def publish_trajectory(self, positions, velocities, accelerations, sleep):
        """
        Set joint trajectory (positions and velocities) of the panda 

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

        for index, _ in enumerate(positions):
            self.point.positions[index] = positions[index]
            self.point.velocities[index] = velocities[index]
            self.point.accelerations[index] = accelerations[index]

        self._publish_all_values(sleep)        

    # End General Utilities
    def _publish_all_values(self, sleep):
        """
        This will just set all the values and send the trajectory msg to ros master. This should be used for static
        data collection
        
        Arguments
        ------------
        sleep: float
            Set how long it should wait after commanding a command

        Returns
        --------
        return: None
        """
        self.msg.points = [self.point]
        if not rospy.is_shutdown():
            # publish message to actuate the dof
            self.trajectory_pub.publish(self.msg)

            if sleep:
                rospy.sleep(self.sleep_time_static)
            else:
                self.r.sleep()

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
        for each_pose in poses:
            positions, _, pose_string = each_pose[0], each_pose[1], each_pose[2]
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
        # TODO: add accelerations
        for each_pose in poses:
            positions, velocities, pose_string = each_pose[0], each_pose[1], each_pose[2]
            accelerations = [0.0]*7
            self.pose_string = pose_string
            self.publish_trajectory(positions, velocities, accelerations, sleep)

if __name__ == "__main__":
    poses_list = [
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1'],
        [[-0.5, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, -0.5, 0, 0, 0, 0], 'Pose_2']
    ]
    controller = PandaController()
    while True:
        controller.move_like_sine_dynamic()
