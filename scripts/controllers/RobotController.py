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
        self.joint_data = None
        self.names = None
        rospy.Subscriber(joint_states_topic, JointState, self.joint_state_callback)

    def joint_names(self):
        if self.joint_data is not None:
            return self.names

    def joint_angles(self):
        """
        gets the joint angles of the robot arm
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
    print(arm.joint_names())
    while(not rospy.is_shutdown()):
        # print("hoo")
        pass
        print(arm.joint_names())
