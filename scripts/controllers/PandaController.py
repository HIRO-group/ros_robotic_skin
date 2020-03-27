#!/usr/bin/env python
"""
This is a panda pose library, this is comprised of three main types of classes
1) General Utilities: defs utilized by all classes
2) Static Defs: All defs starting with static.
    Used only for static data collection
3) dynamic Defs: All defs starting with dynamic.
    Used for dynamic data collection only
Each Pose should be defined in this way
poses_list = [
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4],
        [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1'],
        [[-0.5, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4],
        [0, 0, -0.5, 0, 0, 0, 0], 'Pose_2']
    ]
Which is basically:
poses_list = [
        [[Position List_1], [Velocity_list_1], 'Pose_name_1'],
        [[Position List_2], [Velocity_list_2], 'Pose_name_2']
    ]
"""
import rospy
from math import pi
import numpy as np

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PandaControllerManager import ControllerType, PandaControllerManager  # noqa: E402


class PandaController(object):
    """
    This is the main PandaController class.
    The main reason this class needs to be overridden is
    that you can rospy.init_node
    once. Hence. You need to call this super method.
    """

    def __init__(self, is_sim=True):
        # Create Publishers and Init Node
        """
        Panda Controller class for controlling the panda in
        both simulation and in real life

        Arguments
        ---------
        `is_sim`: `bool`
            Whether the controlled robot is from simulation or not.
        """
        self.is_sim = is_sim
        rospy.init_node('panda_pose', anonymous=True)
        self.trajectory_pub = self.get_trajectory_publisher(is_sim)
        self.position_pubs = self.get_position_publishers()
        self.velocity_pubs = self.get_velocity_publishers()

        self.pcm = PandaControllerManager()
        # Prepare msg to send
        self.msg = JointTrajectory()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = '/base_link'
        self.msg.joint_names = ['panda_joint1', 'panda_joint2',
                                'panda_joint3', 'panda_joint4', 'panda_joint5',
                                'panda_joint6', 'panda_joint7']

        # Set Initial position of the robot
        self.point = JointTrajectoryPoint()
        self.point.positions = [0, 0, 0, 0, 0, 0, 0]
        self.point.velocities = [0, 0, 0, 0, 0, 0, 0]
        self.point.accelerations = [0, 0, 0, 0, 0, 0, 0]
        self.point.time_from_start.secs = 1
        self.msg.points = [self.point]
        # TODO: Look up do we need to have one message to init the robot?
        # If I only send one message,
        #  then the franka bottom-most joint does not move in simulation.
        # Need to check if same thing happens in real robot too.
        # @peasant98 can you please confirm this?
        # TODO: This might not be the best starting position
        # for the robot to be in.
        # Think about if we are losing some information by
        # using a completely vertical position
        # for the start.
        # Move arm to starting position
        # self.trajectory_pub.publish(self.msg)
        rospy.sleep(1)
        # self.trajectory_pub.publish(self.msg)
        rospy.sleep(1)

        self.sleep_time_static = rospy.get_param('/static_sleep_time')
        self.r = rospy.Rate(rospy.get_param('/dynamic_frequency'))
        self.pose_name = ''

        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.joint_states = JointState()

    @property
    def joint_names(self):
        """
        Gets the joint names of the Panda.
        """
        return self.msg.joint_names

    @property
    def joint_angles(self):
        """
        Gets the current joint angles of the Panda.
        """
        index = [self.joint_states.name.index(joint_name) for joint_name in self.joint_names]
        return [self.joint_states.position[idx] for idx in index]

    @property
    def joint_velocities(self):
        """
        Gets the current joint velocities of the Panda.
        """
        index = [self.joint_states.name.index(joint_name) for joint_name in self.joint_names]
        return [self.joint_states.velocity[idx] for idx in index]

    def joint_state_callback(self, joint_states):
        """
        Joint state callback for the Panda.

        Arguments
        ---------
        `joint_states`: `JointState`
            The joint state message received from the Subscriber.

        """
        self.joint_states = joint_states

    def joint_angle(self, joint_name):
        """
        Gets the joint angle for the Panda's
        corresponding joint name.

        Arguments
        ---------
        `joint_name`: `str`
            The name of the joint on the Panda.

        """
        idx = self.joint_states.name.index(joint_name)
        return self.joint_states.position[idx]

    def joint_velocity(self, joint_name):
        """
        Gets the joint velocity for the Panda's
        corresponding joint name.

        Arguments
        ---------
        `joint_name`: `str`
            The name of the joint on the Panda.

        """
        idx = self.joint_states.name.index(joint_name)
        return self.joint_states.velocity[idx]

    def set_joint_position_speed(self, speed=1.0):
        """
        Sets the joint position speed for the Panda.
        However, this is not implemented yet.

        Arguments
        ---------
        `speed`: `float`
            The speed in rad/s of the joints.

        """
        rospy.logerr('Set Joint Position Speed Not Implemented for Panda')

    # General Utilities
    def get_trajectory_publisher(self, is_sim):
        """
        Sets the correct topic to publish a `JointTrajectory`
        message to. This is dependent on if we are in simulation or
        not.

        Arguments
        ---------
        `is_sim`: `bool`
            If Panda is in simulation or not.

        """
        topic_string = '/panda_joint_trajectory_controller/command' if is_sim else '/joint_trajectory_controller/command'
        return rospy.Publisher(topic_string, JointTrajectory, queue_size=1)

    def get_velocity_publishers(self):
        # create a list of velocity publishers.
        self.joint_velocity_controller_names = ['panda_joint%s_velocity_controller' % (i) for i in range(1, 8)]
        joint_velocity_pubs = []
        for name in self.joint_velocity_controller_names:
            pub = rospy.Publisher('/%s/command' % (name), Float64, queue_size=10)
            joint_velocity_pubs.append(pub)
        return joint_velocity_pubs

    def get_position_publishers(self):
        self.joint_position_controller_names = ['panda_joint%s_position_controller' % (i) for i in range(1, 8)]
        joint_position_pubs = []
        for name in self.joint_position_controller_names:
            pub = rospy.Publisher('/%s/command' % (name), Float64, queue_size=10)
            joint_position_pubs.append(pub)
        return joint_position_pubs

    def send_once(self):
        """
        Sends one `JointTrajectory` message.
        """
        self.pcm.switch_mode(ControllerType.TRAJECTORY)
        self.trajectory_pub.publish(self.msg)
        rospy.sleep(1)
        self.trajectory_pub.publish(self.msg)
        rospy.sleep(1)

    def publish_positions(self, positions, sleep):
        """
        Set joint positions of the panda with the
        Panda Joint Position Controller

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
        self.pcm.switch_mode(ControllerType.POSITION)
        if len(positions) != 7:
            raise Exception("The length of input list should be 7, as panda has 7 joints")
        for index, pos in enumerate(positions):
            self.position_pubs[index].publish(Float64(pos))
        rospy.sleep(sleep)

    def publish_velocities(self, velocities, sleep):
        """
        Set joint velocities of the panda with
        the JointVelocityController.
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

        self.pcm.switch_mode(ControllerType.VELOCITY)
        if len(velocities) != 7:
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        for index, vel in enumerate(velocities):
            self.velocity_pubs[index].publish(Float64(vel))
        # sleep a bit of time
        rospy.sleep(sleep)
        for pub in self.velocity_pubs:
            pub.publish(Float64(0.0))

        # self._publish_all_values(sleep)

    def publish_accelerations(self, accelerations, sleep):
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
        self.pcm.switch_mode(ControllerType.TRAJECTORY)

        if len(accelerations) != 7:
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        for index, _ in enumerate(accelerations):
            self.point.accelerations[index] = accelerations[index]

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
        self.pcm.switch_mode(ControllerType.TRAJECTORY)

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
        This will just set all the values and send the trajectory msg
        to ros master. This should be used for static data collection

        Arguments
        ------------
        sleep: float
            Set how long it should wait after commanding a command

        Returns
        --------
        return: None
        """
        self.pcm.switch_mode(ControllerType.TRAJECTORY)

        self.msg.points = [self.point]
        self.msg.header.stamp = rospy.Time.now()

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
        self.pcm.switch_mode(ControllerType.POSITION)

        for each_pose in poses:
            positions, _, pose_name = each_pose[0], each_pose[1], each_pose[2]  # noqa: F841
            self.pose_name = pose_name
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
        self.pcm.switch_mode(ControllerType.VELOCITY)

        for each_pose in poses:
            _, velocities, pose_name = each_pose[0], each_pose[1], each_pose[2]  # noqa: F841
            self.pose_name = pose_name
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
        self.pcm.switch_mode(ControllerType.TRAJECTORY)

        # TODO: add accelerations
        for each_pose in poses:
            positions, velocities, pose_name = each_pose[0], each_pose[1], each_pose[2]
            accelerations = np.zeros(7)
            self.pose_name = pose_name
            self.publish_trajectory(positions, velocities, accelerations, sleep)


if __name__ == "__main__":
    # just 2 poses for now.
    poses_list = [
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1'],
        [[-0.5, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, -0.5, 0, 0, 0, 0], 'Pose_2']
    ]
    # uncomment the below code for things in action!
    controller = PandaController()
    # controller.publish_velocities([0, 0.5, 0.5, 0, 0, 0, 0], 1)
    # controller.publish_positions([0,0,0,0,0,0,0],5)
    while True:
        controller.set_trajectory_list(poses_list, sleep=1)