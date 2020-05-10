#!/usr/bin/env python
"""
This code serves as the generic RobotController
class.
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from enum import Enum


class ControllerType(Enum):
    POSITION = 1
    VELOCITY = 2
    TRAJECTORY = 3


class RobotArm(object):
    """
    Generic robot arm implementation in ROS
    """
    def __init__(self, joint_states_topic='/joint_states',
                 ignore_joints_arr=['panda_finger_joint1', 'panda_finger_joint2']):
        rospy.init_node('robot_arm')
        self.ignore_joints_arr = ignore_joints_arr
        self.data_exists = False
        self.joint_data = None
        self.names = None
        self.positions = None
        rospy.Subscriber(joint_states_topic, JointState, self.joint_state_callback)

    def joint_names(self):
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

    def move_to_joint_positions(self, positions, timeout=15.0):
        """
        moves robot to specific joint positions.
        """
        pass

    def joint_state_callback(self, data):
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

class RobotController(object):
    def __init__(self, is_sim=True):
        """
        robot controller for n joints.
        """
        self.is_sim = is_sim
        rospy.init_node('robot_controller', anonymous=True)

        self.sleep_time_static = rospy.get_param('/static_sleep_time')
        self.r = rospy.Rate(rospy.get_param('/dynamic_frequency'))
        self.pose_name = ''

    @property
    def joint_names(self):
        pass

    @property
    def joint_velocities(self):
        pass

    def joint_state_callback(self, joint_states):
        """
        Joint state callback for the Panda.

        Arguments
        ---------
        `joint_states`: `JointState`
            The joint state message received from the Subscriber.

        """
        pass

    def joint_angle(self, joint_name):
        pass

    def joint_velocity(self, joint_name):
        pass

    def set_joint_position_speed(self, speed=1.0):
        pass

    def get_trajectory_publisher(self, is_sim):
        pass

    def get_velocity_publishers(self):
        pass

    def get_position_publishers(self):
        pass

    def send_once(self):
        pass

    def publish_positions(self, positions, sleep):
        pass

    def send_velocities(self, velocities):
        pass

    def publish_velocities(self, velocities):
        pass

    def publish_accelerations(self, accelerations, sleep):
        pass

    def publish_trajectory(self, positions, velocities, accelerations, sleep):
        pass

    def _publish_all_values(self, sleep):
        pass

    def set_positions_list(self, poses, sleep):
        pass

    def set_velocities_list(self, velocities, sleep):
        pass

    def set_trajectory_list(self, trajectories, sleep):
        pass


if __name__ == '__main__':
    arm = RobotArm()
    while(not rospy.is_shutdown()):
        # print("hoo")
        pass
        # print(arm.joint_velocity("panda_joint1"))


"""
if self.data_exists:
            for idx, val in self.names:
                self.positions[val] = positions[idx]

"""