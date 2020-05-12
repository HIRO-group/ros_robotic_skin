#!/usr/bin/env python
"""
Code.
"""
import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from RobotControllerManager import RobotControllerManager, ControllerType


class RobotArm(object):
    """
    Generic robot arm implementation in ROS. Works for n joints.
    """

    def __init__(self, num_joints,
                 is_sim=True, joint_states_topic='/joint_states',
                 ignore_joints_arr=['panda_finger_joint1', 'panda_finger_joint2'],
                 controller_names=None):
        """
        intializiation of the robot arm class.

        `controller_names` should have:

        `controller_names[0]` - position controllers

        `controller_names[1]` - velocity controllers

        `controllers_names[2]` - ONE trajectory controller.
        """


        self.num_joints = num_joints
        if controller_names is None:
            controller_names = []
            default_pos_names = ['panda_joint{}_position_controller'.format(i) for i in range(1, num_joints+1)]
            default_vel_names = ['panda_joint{}_velocity_controller'.format(i) for i in range(1, num_joints+1)]
            default_traj_name = ['panda_joint_trajectory_controller' if is_sim else 'joint_trajectory_controller']
            controller_names.append(default_pos_names)
            controller_names.append(default_vel_names)
            controller_names.append(default_traj_name)

        # create the controller manager for switching controllers.
        self.controller_manager = RobotControllerManager(controller_names)

        self.ignore_joints_arr = ignore_joints_arr
        self.data_exists = False
        self.joint_data = None
        self.names = None
        self.positions = None

        # get list of position and velocity publishers ... and trajectory publisher.
        self.pos_pubs, self.vel_pubs, self.traj_pub = self.get_publishers(controller_names)

        rospy.Subscriber(joint_states_topic, JointState, self.joint_state_callback)
        # need a sleep, so message is sent from publisher, interestingly.
        rospy.sleep(2)

    def joint_names(self):
        """
        get the joint names of the robot
        """
        if self.data_exists:
            return self.names

    def joint_angles(self):
        """
        gets the joint angles of the robot arm
        """
        if self.data_exists:
            return np.array(self.joint_data.position)[self.valid_indices]

    def joint_velocities(self):
        """
        gets the joint velocities of the robot arm
        """
        if self.data_exists:
            return np.array(self.joint_data.velocity)[self.valid_indices]

    def joint_angle(self, joint_name):
        """
        gets the joint angle based on the provided joint name.
        """
        if self.data_exists:
            angles = self.joint_angles()
            return angles[self.mapping[joint_name]]

    def joint_velocity(self, joint_name):
        """
        gets the joint velocity based on the provided joint name.
        """
        if self.data_exists:
            velocities = self.joint_velocities()
            return velocities[self.mapping[joint_name]]

    def set_joint_position_speed(self, speed=1.0):
        rospy.logerr('Set Joint Position Speed Not Implemented for Robot Arm')

    def move_to_joint_positions(self, positions, sleep=1.0, timeout=15.0):
        """
        moves robot to specific joint positions.
        """
        if not rospy.is_shutdown():
            self.controller_manager.switch_mode(ControllerType.POSITION)
            if len(positions) != self.num_joints:
                raise ValueError("Wrong number of joints provided.")
            for index, pos in enumerate(positions):
                self.pos_pubs[index].publish(Float64(pos))
            rospy.sleep(sleep)

    def set_joint_velocities(self, velocities):
        """
        affects the set joint velocities
        """
        self.controller_manager.switch_mode(ControllerType.VELOCITY)

        if len(velocities) != self.num_joints:
            raise ValueError("Wrong number of joints provided.")

        for index, vel in enumerate(velocities):
            self.vel_pubs[index].publish(Float64(vel))

    def set_joint_trajectory(self, trajectories, time_per_step=1.0):
        """
        sends a joint trajectory command.
        """
        if not rospy.is_shutdown():
            self.controller_manager.switch_mode(ControllerType.TRAJECTORY)

            # 3d tensor of shape (3, n_joint, n_point)
            points_tensor = np.array(trajectories)
            assert points_tensor.shape[1] == 3, "3 row should be provided for pos, vel and acc."
            assert len(points_tensor.shape) == 3, "Trajectories should be of 3-d shape!"
            assert points_tensor.shape[2] == self.num_joints, "Trajectory should " \
                                                            "account for num joints."

            num_points = points_tensor.shape[0]
            time_from_start = time_per_step
            time_from_start_duration = rospy.Duration(time_from_start)
            msg = JointTrajectory()
            msg.points = []
            for i in range(num_points):
                point = JointTrajectoryPoint()
                point.positions = list(points_tensor[i, 0])
                point.velocities = list(points_tensor[i, 1])
                point.accelerations = list(points_tensor[i, 2])
                point.time_from_start = time_from_start_duration
                time_from_start += time_per_step
                time_from_start_duration = rospy.Duration(time_from_start)
                msg.points.append(point)
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = '/base_link'
            msg.joint_names = list(self.names)

            self.traj_pub.publish(msg)

    def joint_state_callback(self, data):
        """
        callback that takes care of the joint state data.
        """
        self.joint_data = data
        if self.names is None:
            names_arr = np.array(self.joint_data.name)
            arr = np.isin(names_arr, self.ignore_joints_arr)
            indices = np.squeeze(np.argwhere(arr == False))
            self.valid_indices = indices
            self.names = names_arr[self.valid_indices]
            self.mapping = {v: i for i, v in enumerate(self.names)}
            self.positions = {name: 0.0 for name in self.names}

            # construct mapping on joint names to indices
        self.data_exists = True

    def get_publishers(self, controller_names):
        """
        gets position, velocity, and trajectory publisher(s). Lengths may
        be different.
        """
        joint_position_pubs = []
        joint_velocity_pubs = []
        pos_controller_names = controller_names[0]
        vel_controller_names = controller_names[1]
        traj_controller_name = controller_names[2][0]
        traj_name = '/{}/command'.format(traj_controller_name)
        traj_pub = rospy.Publisher(traj_name, JointTrajectory, queue_size=1)

        for name in pos_controller_names:
            pub = rospy.Publisher('/{}/command'.format(name), Float64,
                                  queue_size=10)
            joint_position_pubs.append(pub)

        for name in vel_controller_names:
            pub = rospy.Publisher('/{}/command'.format(name), Float64,
                                  queue_size=10)
            joint_velocity_pubs.append(pub)

        return joint_position_pubs, joint_velocity_pubs, traj_pub
