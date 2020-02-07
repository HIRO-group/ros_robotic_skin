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


class PandaPose(object):
    """
    This is the main PandaPose class. The main reason this class needs to be overridden is that you can rospy.init_node
    once. Hence. You need to call this super method.
    """

    def __init__(self):
        # Create Publishers and Init Node
        self.trajectory_pub = rospy.Publisher('/panda_arm_controller/command', JointTrajectory, queue_size=1)
        self.pub_int = rospy.Publisher('/joint_mvmt_dof', Int16, queue_size=1)
        self.pub_bool = rospy.Publisher('/calibration_complete', Bool, queue_size=1)
        rospy.init_node('calibration_joint_mvmt_node', anonymous=True)
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

    def _set_pose(self, pose, pose_string):
        """
        Set pose of the panda
        :param pose: list
            Set the pose according to the list that you get
        :param pose_string: str
            Set Pose type string like Pose_0
        :return:
        """
        if len(pose) != 7:
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        for index, _ in enumerate(self.point.positions):
            self.point.positions[index] = pose[index]
        self.pose_string = pose_string

    def _set_velocity(self, velocity):
        """
        Set velocity of the joint
        :param velocity:
            Set the joint velocity according to the list that you get
        :return: None
        """
        if len(velocity) != 7:
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        for index, _ in enumerate(velocity):
            self.point.velocities[index] = velocity[index]

    # End General Utilities

    # Start Dynamic Changes
    def _set_all_values_dynamic(self):
        """
        This will just set all the values and send the trajectory msg to ros master. This should be used for dynamic
        data collection
        :return:
        """
        self.msg.points = [self.point]
        if not rospy.is_shutdown():
            # publish message to actuate the dof
            self.trajectory_pub.publish(self.msg)
            self.r.sleep()

    def set_poses_position_dynamic(self, poses):
        """
        Set Poses
        :param poses: Special Data Structure
            Loop through all the poses and velocity in the poses
        :return: None
        """
        for each_pose in poses:
            pose_configuration, velocity_configuration, pose_string = each_pose[0], each_pose[1], each_pose[2]
            self._set_pose(pose_configuration, pose_string)
            self.pose_string = pose_string
            self._set_all_values_dynamic()

    def set_poses_velocity_dynamic(self, poses):
        """
        Set Velocities
        :param poses: Special Data Structure
            Loop through all the poses and velocity in the poses
        :return: None
        """
        for each_pose in poses:
            pose_configuration, velocity_configuration, pose_string = each_pose[0], each_pose[1], each_pose[2]
            self._set_velocity(velocity_configuration)
            self.pose_string = pose_string
            self._set_all_values_dynamic()

    def set_poses_position_velocity_dynamic(self, poses):
        """
        Set Velocities and Poses
        :param poses: Special Data Structure
            Loop through all the poses and velocity in the poses
        :return: None
        """
        for each_pose in poses:
            pose_configuration, velocity_configuration, pose_string = each_pose[0], each_pose[1], each_pose[2]
            self._set_pose(pose_configuration, pose_string)
            self._set_velocity(velocity_configuration)
            self.pose_string = pose_string
            self._set_all_values_dynamic()

    def move_like_sine_dynamic(self):
        """
        This will move the joint of the Panda ARM like a sine wave
        # TODO: add minutes delay
        :return:
        """
        d1 = datetime.datetime.now() + datetime.timedelta(minutes=1)
        while True:
            for each_degree in range(0, 360):
                self._set_velocity([0, 0, 5 * math.sin(math.radians(each_degree)), 0, 0, 0, 0])
                self._set_all_values_dynamic()
            if d1 < datetime.datetime.now():
                break

    # Start Static Change
    def _set_all_values_static(self):
        """
        This will just set all the values and send the trajectory msg to ros master. This should be used for static
        data collection
        :return:
        """
        self.msg.points = [self.point]
        if not rospy.is_shutdown():
            # publish message to actuate the dof
            self.trajectory_pub.publish(self.msg)
            rospy.sleep(self.sleep_time_static)

    def set_poses_position_static(self, poses):
        """
        Set poses
        :param poses:  Special Data Structure
            Loop through all the poses and velocity in the poses
        :return: None
        """
        for each_pose in poses:
            pose_configuration, velocity_configuration, pose_string = each_pose[0], each_pose[1], each_pose[2]
            self._set_pose(pose_configuration, pose_string)
            self.pose_string = pose_string
            self._set_all_values_static()

    def set_poses_velocity_static(self, poses):
        """
        Set Velocities
        :param poses:  Special Data Structure
            Loop through all the poses and velocity in the poses
        :return: None
        """
        for each_pose in poses:
            pose_configuration, velocity_configuration, pose_string = each_pose[0], each_pose[1], each_pose[2]
            self._set_velocity(velocity_configuration)
            self.pose_string = pose_string
            self._set_all_values_static()

    def set_poses_position_velocity_static(self, poses):
        """
        Set poses and velocities
        :param poses:  Special Data Structure
            Loop through all the poses and velocity in the poses
        :return: None
        """
        for each_pose in poses:
            pose_configuration, velocity_configuration, pose_string = each_pose[0], each_pose[1], each_pose[2]
            self._set_pose(pose_configuration, pose_string)
            self._set_velocity(velocity_configuration)
            self.pose_string = pose_string
            self._set_all_values_static()


if __name__ == "__main__":
    poses_list = [
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1'],
        [[-0.5, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, -0.5, 0, 0, 0, 0], 'Pose_2']
    ]
    pp = PandaPose()
    while True:
        pp.move_like_sine_dynamic()
