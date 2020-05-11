#!/usr/bin/env python
"""
This code serves as the generic RobotController
class.
"""

import rospy
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from enum import Enum
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ControllerType(Enum):
    POSITION = 1
    VELOCITY = 2
    TRAJECTORY = 3


class RobotArm(object):
    """
    Generic robot arm implementation in ROS
    """
    def __init__(self, num_joints,
                 is_sim=True, joint_states_topic='/joint_states',
                 ignore_joints_arr=['panda_finger_joint1', 'panda_finger_joint2'],
                 controller_names=None):
        """
        controller_names should have -
        controller_names[0] - position controllers
        controller_names[1] - velocity controllers
        controllers_names[2] - ONE trajectory controller.
        """
        if controller_names is None:
            controller_names = []
            default_pos_names = ['panda_joint%s_position_controller' % (i) for i in range(1, num_joints+1)]
            default_vel_names = ['panda_joint%s_velocity_controller' % (i) for i in range(1, num_joints+1)]
            default_traj_name = ['/panda_joint_trajectory_controller/command' if is_sim else '/joint_trajectory_controller/command']
            controller_names.append(default_pos_names)
            controller_names.append(default_vel_names)
            controller_names.append(default_traj_name)

        rospy.init_node('robot_arm')
        self.ignore_joints_arr = ignore_joints_arr
        self.data_exists = False
        self.joint_data = None
        self.names = None
        self.positions = None
        self.pos_pubs, self.vel_pubs, self.traj_pub = self.get_publishers(controller_names)
        rospy.Subscriber(joint_states_topic, JointState, self.joint_state_callback)

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

    def move_to_joint_positions(self, positions, sleep=5.0, timeout=15.0):
        """
        moves robot to specific joint positions.
        """
        if len(positions) != self.num_joints:
            raise ValueError("Wrong number of joints provided.")
        for index, pos in enumerate(positions):
            self.pos_pubs[index].publish(Float64(pos))
        rospy.sleep(sleep)

    def set_joint_velocities(self, velocities):
        """
        affects the set joint velocities
        """
        if len(velocities) != self.num_joints:
            raise ValueError("Wrong number of joints provided.")

        for index, vel in enumerate(velocities):
            self.vel_pubs[index].publish(Float64(vel))

    def set_joint_trajectory(self, trajectories):
        """
        sends a joint trajectory command.
        """
        # 3d tensor of shape (3, n_joint, n_point)
        points_tensor = np.array(trajectories)
        assert points_tensor.shape[1] == 3, "3 row should be provided for pos, vel and acc."
        assert len(points_tensor.shape) == 3, "Trajectories should be of 3-d shape!"
        assert points_tensor.shape[2] == self.num_joints, "Trajectory should " \
                                                          "account for num joints."

        num_points = points_tensor.shape[0]
        time_from_start = 1.0
        msg = JointTrajectory()
        msg.points = []
        for i in range(num_points):
            point = JointTrajectoryPoint()
            point.positions = list(points_tensor[i, 0])
            point.velocities = list(points_tensor[i, 1])
            point.accelerations = list(points_tensor[i, 2])
            point.time_from_start = time_from_start
            msg.points.append(point)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = '/base_link'
        msg.joint_names = list(self.names)
        self.traj_pub.publish(msg)

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

    def get_publishers(self, controller_names):
        """
        gets velocity and position publishers. Lengths may
        be different.
        """
        joint_velocity_pubs = []
        joint_position_pubs = []
        pos_controller_names = controller_names[0]
        vel_controller_names = controller_names[1]
        traj_controller_name = controller_names[2][0]
        traj_pub = rospy.Publisher(traj_controller_name, JointTrajectory, queue_size=1)

        for name in vel_controller_names:
            pub = rospy.Publisher('/{}/command'.format(name), Float64,
                                  queue_size=10)
            joint_velocity_pubs.append(pub)

        for name in pos_controller_names:
            pub = rospy.Publisher('/{}/command'.format(name), Float64,
                                  queue_size=10)
            joint_position_pubs.append(pub)

        return joint_position_pubs, joint_velocity_pubs, traj_pub


class RobotController(object):
    def __init__(self, is_sim=True):
        """
        robot controller for n joints.
        """
        self.is_sim = is_sim
        rospy.init_node('robot_controller', anonymous=True)
        self.arm = RobotArm(num_joints=7)
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