#!/usr/bin/env python
import os
import sys
from collections import OrderedDict
import copy
from math import pi
import numpy as np
import pickle
import matplotlib.pyplot as plt

import rospy
import rospkg
from sensor_msgs.msg import Imu

sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts import utils  # noqa: E402
from scripts.exceptions import VariableNotInitializedException  # noqa:E402
from scripts.controllers.RobotController import PandaController, SawyerController  # noqa: E402

SIM_DT = 1.0 / rospy.get_param('/dynamic_frequency')
OSCILLATION_TIME = rospy.get_param('/oscillation_time')
FREQ = rospy.get_param('/oscillation_frequency')
AMPLITUDE = rospy.get_param('/oscillation_magnitude')


class StopWatch():
    def __init__(self):
        self.start_time = None

    def start(self):
        self.start_time = rospy.get_rostime().to_sec()

    def get_elapsed_time(self):
        if not self.is_started():
            raise VariableNotInitializedException("Varible 'start_time' is not initialized")

        return rospy.get_rostime().to_sec() - self.start_time

    def restart(self):
        self.start_time = rospy.get_rostime().to_sec()

    def stop(self):
        self.start_time = None

    def is_started(self):
        return self.start_time


class DynamicPoseData():
    """
    Class to store dynamic pose data into a nested dictionary.
    It is stored as in [20 poses][7 excited joints][7 IMUs][data].
    self.data[pose_name][joint_name][imu_name] = np.ndarray
    The data is defined as below.
    data = [max acceleration, theta at the point].
    """
    def __init__(self, pose_names, joint_names, imu_names, filepath):
        """
        Initialize DynamicPoseData class.

        Arguments
        ----------
        pose_names: list[str]
            Names of poses
        joint_names: list[str]
            Names of joints
        imu_names: list[str]
            Names of imus
        filepath: str
            Path to save the collected data
        """
        self.pose_names = pose_names
        self.joint_names = joint_names
        self.imu_names = imu_names

        self.filepath = filepath
        self.data = OrderedDict()
        self.data_shape = (0, 14)

        # Create nested dictionary to store data
        for pose_name in pose_names:
            self.data[pose_name] = OrderedDict()
            for joint_name in joint_names:
                self.data[pose_name][joint_name] = OrderedDict()
                for imu_name in self.imu_names:
                    self.data[pose_name][joint_name][imu_name] = np.empty(self.data_shape, float)

    def append(self, pose_name, joint_name, imu_name, data):
        """
        Append data to a dictionary whose keys are
        [pose_name][joint_name][imu_name]

        Arguments
        ----------
        pose_name: str
            Names of poses
        joint_name: str
            Names of joints
        imu_name: str
            Names of imus
        data: np.array
            Numpy array of size (1,4).
            Includes an accelerometer measurement and a joint angle.
        """
        if not isinstance(data, np.ndarray):
            raise ValueError('"data" must be a np.ndarray')

        self.data[pose_name][joint_name][imu_name] = \
            np.append(self.data[pose_name][joint_name][imu_name], np.array([data]), axis=0)

    def clean_data(self, verbose=False, time_range=(0.04, 0.16),
                   eliminate_outliers=True):
        """
        Cleans the data.

        Arguments
        ----------
        `verbose`: `bool`
            Determines if you want to see plots of the original
            data vs. the filtered data.

        `time_range`: `tuple`
            The time range from which the max acceleration is
            to be found. For example, (0.04, 0.16) means that
            the max acceleration value will be searched in between 0.04
            and 0.16 seconds.
        """
        # Create nested dictionary to store data
        data = copy.deepcopy(self.data)
        for pose_name in self.pose_names:
            for joint_name in self.joint_names:
                for imu_name in self.imu_names:
                    # on each pose, for each joint wiggle, get the
                    # maximum acceleration for each skin unit
                    imu_data = self.data[pose_name][joint_name][imu_name]
                    # filter imu acceleration, angular velocities,
                    # joint accelerations
                    imu_accs = imu_data[:, :3]

                    norms = np.linalg.norm(imu_accs, axis=1)
                    joint_accs = imu_data[:, 11]
                    if eliminate_outliers:
                        # use hampel filter for outlier detection
                        # it actually doesn't affect the end result much.
                        norms = utils.hampel_filter_forloop(norms, 10)[0]
                        joint_accs = utils.hampel_filter_forloop(joint_accs, 10)[0]

                    # filter
                    filtered_norms = utils.low_pass_filter(norms, 100.)
                    filtered_joint_accs = utils.low_pass_filter(joint_accs, 100.)

                    # array of imu data - both filtered and raw
                    imu_filtered_arr = []
                    imu_raw_arr = []

                    # max imu acceleration
                    imu_acc_max = 0
                    # max individual joint acceleration
                    joint_acc_max = 0
                    # idx of the max acceleration.
                    best_idx = 0

                    # go through filtered norms and accelerations
                    for idx, (norm, acc) in enumerate(zip(filtered_norms, filtered_joint_accs)):
                        cur_time = imu_data[idx, 10]
                        # add filtered and raw data to array
                        imu_filtered_arr.append(norm)
                        imu_raw_arr.append(norms[idx])
                        """
                        conditions for update of best idx:
                            - the filtered norm is greater than the current highest one.
                            - the time of this data lies within `time_range`
                            - the filtered joint acceleration is also greater than the current highest one.

                        """
                        if norm > imu_acc_max and cur_time < time_range[1] and cur_time > time_range[0] and acc > joint_acc_max:
                            best_idx = idx
                            imu_acc_max = norm
                            joint_acc_max = acc
                    # update data to the max acceleration point
                    best = self.data[pose_name][joint_name][imu_name][best_idx]

                    self.data[pose_name][joint_name][imu_name] = [best]

                    if verbose:
                        # plots the acceleration norms - filtered and raw
                        plt.plot(imu_raw_arr)
                        plt.plot(imu_filtered_arr)
                        plt.show()

                        rospy.loginfo(data[pose_name][joint_name][imu_name])

        return data

    def save(self, data):
        """
        Saves the data to a pickle file.

        Arguments
        ----------
        `data`: `OrderedDict`
            The data to be saved
        """
        ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
        filepath = os.path.join(ros_robotic_skin_path, self.filepath+'.pickle')

        with open(filepath, 'wb') as f:
            pickle.dump(data, f)


class DynamicPoseDataSaver():
    """
    Class for collecting dynamic pose data and save them as a pickle file
    """
    def __init__(self, controller, poses_list, filepath='data/dynamic_data'):
        """
        Initializes DynamicPoseDataSaver class.

        Arguments
        -----------
        controller:
            Wrapped controller to control either Panda or Sawyer robot.
        poses_list: list
            A list of poses. Each pose is a list.
            It includes 7 joint position, 7 joint velociites, and Pose name
        filepath: str
            File path to save the collected data
        """
        self.controller = controller
        self.poses_list = poses_list
        # constant
        self.pose_names = [pose[2] for pose in poses_list]

        # get imu names and topics through rostopic and xacro.
        self.imu_names, self.imu_topics = utils.get_imu_names_and_topics()

        self.curr_pose_name = self.pose_names[0]
        self.curr_joint_name = self.controller.joint_names[0]
        self.prev_angular_velocity = 0.0

        self.watch_dt = StopWatch()
        self.watch_motion = StopWatch()

        # data storage
        self.data_storage = DynamicPoseData(self.pose_names, self.controller.joint_names, self.imu_names, filepath)
        # Subscribe to IMUs
        for imu_topic in self.imu_topics:
            rospy.Subscriber(imu_topic, Imu, self.callback)

    def callback(self, data):
        """
        A callback function for IMU topics

        Arguments
        ----------
        data: sensor_msgs.msg.Imu
            IMU data. Please refer to the official documentation.
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
        """
        if self.watch_motion.is_started():
            acceleration = utils.Vector3_to_np(data.linear_acceleration)

            dt = self.watch_dt.get_elapsed_time()
            self.watch_dt.restart()
            if dt <= 0.9*SIM_DT:
                return

            curr_angular_velocity = self.controller.joint_velocity(self.curr_joint_name)
            angular_acceleration = (curr_angular_velocity - self.prev_angular_velocity) / dt
            self.prev_angular_velocity = curr_angular_velocity

            joint_angles = self.controller.joint_angles

            # time
            t = self.watch_motion.get_elapsed_time()
            self.data_storage.append(
                pose_name=self.curr_pose_name,          # for each defined initial pose
                joint_name=self.curr_joint_name,        # for each excited joint
                imu_name=data.header.frame_id,          # for each imu
                data=np.r_[
                    acceleration,
                    joint_angles,
                    t,
                    angular_acceleration,
                    AMPLITUDE,
                    curr_angular_velocity,
                ]
            )

    def goto_current_pose(self, pose):
        positions, _, pose_name = pose[0], pose[1], pose[2]  # noqa: F841
        self.curr_pose_name = pose_name
        # first, move to the position from <robot>_positions.txt
        self.controller.publish_positions(positions, sleep=2)
        print('At Position: ' + pose_name,
              map(int, utils.RAD2DEG * np.array(positions)))

    def prepare_recording(self, joint_name):
        # Set current joint
        self.curr_joint_name = joint_name
        # Set current joint velocity
        self.prev_angular_velocity = self.controller.joint_velocity(self.curr_joint_name)
        # Start Stop Watchs
        self.watch_dt.start()

    def move_like_sine_dynamic(self):
        """
        This will move the joint of the robot arm like a sine wave
        for all joints for all defined poses.
        """
        self.controller.set_joint_position_speed(speed=1.0)

        for pose in self.poses_list:
            for i, joint_name in enumerate(self.controller.joint_names):
                # Go to current setting position
                self.goto_current_pose(pose)

                # Initialize Variables
                self.prepare_recording(joint_name)
                # Prepare for publishing velocities
                velocities = np.zeros(len(self.controller.joint_names))

                # Start motion and recording
                self.watch_motion.start()
                while True:
                    # time within motion
                    t = self.watch_motion.get_elapsed_time()

                    # Oscillated Velocity pattern
                    velocities[i] = AMPLITUDE * np.sin(2 * pi * FREQ * t)
                    self.controller.send_velocities(velocities)

                    if t > OSCILLATION_TIME:
                        break

                    self.controller.r.sleep()
                self.watch_motion.stop()
                rospy.sleep(1)

    def save(self, save=True, verbose=False, clean=True):
        """
        Save data to a pickle file.


        Arguments
        ----------
        `save`: `bool`
            If the data will be saved

        `verbose`: `bool`
        """
        if clean:
            data = self.data_storage.clean_data(verbose)
        else:
            data = self.data_storage.data

        if save:
            rospy.loginfo('saving')
            self.data_storage.save(data)


if __name__ == "__main__":
    # [Pose, Joint, IMU, x, y, z]* number os samples according to hertz
    robot = sys.argv[1]

    if robot == 'panda':
        controller = PandaController()
        filename = 'panda_positions.txt'
    elif robot == 'sawyer':
        controller = SawyerController()
        filename = 'sawyer_positions.txt'
    else:
        raise ValueError("Must be either panda or sawyer")

    if len(sys.argv) > 2:
        filename = sys.argv[2]

    poses_list = utils.get_poses_list_file(filename)
    filepath = '_'.join(['data/dynamic_data', robot])

    dd = DynamicPoseDataSaver(controller, poses_list, filepath)
    dd.move_like_sine_dynamic()
    dd.save(save=True, verbose=False, clean=False)
