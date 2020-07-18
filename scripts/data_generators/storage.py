#!/usr/bin/env python
import sys
from collections import OrderedDict
import copy
import numpy as np
import pickle
import matplotlib.pyplot as plt
# ROS packages
import rospy
import rospkg

sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts import utils  # noqa: E402


def reject_outliers(data, m=1):
    """
    Rejects one outlier.

    Arguments
    ---------
    `data`: `np.array`
        The data

    `m`: `int`
        The number of standard deviations from the mean to
        reject data points.

    """
    is_in_std = np.all(np.absolute(data - np.mean(data, axis=0)) < m * np.std(data, axis=0), axis=1)
    return data[is_in_std, :]


def reject_outlier(data, m=1):
    """
    Rejects one outlier.

    Arguments
    ---------
    `data`: `np.array`
        The data

    `m`: `int`
        The number of standard deviations from the mean to
        reject data points.
    """
    is_in_std = np.absolute(data - np.mean(data)) < m * np.std(data)
    return data[is_in_std, :]


class StaticPoseData():
    """
    Class to store static poses into a nested dictionary.
    It is stored as in [20 poses][7 IMUs][data].
    self.data[pose_name][imu_name] = np.ndarray
    The data is defined as below.
    data = [mean x y z acceleration].
    so the np.ndarray's dimension is (No. of collected data
    x  xyz accelerations)
    """
    def __init__(self, pose_names, imu_names, filepath):
        """
        Initialize StaticPoseData class.

        Arguments
        ----------
        pose_names: list[str]
            Names of poses
        imu_names: list[str]
            Names of imus
        filepath: str
            Path to save the collected data
        """
        self.pose_names = pose_names

        self.imu_names = imu_names
        self.filepath = filepath
        self.data = OrderedDict()

        # Create nested dictionary to store data
        for pose_name in pose_names:
            self.data[pose_name] = OrderedDict()
            for imu_name in self.imu_names:
                self.data[pose_name][imu_name] = np.empty((0, 14), float)

    def append(self, pose_name, imu_name, data):
        """
        Append data to a dictionary whose keys are
        [pose_name][imu_name]

        Arguments
        ----------
        pose_name: str
            Names of poses
        imu_name: str
            Names of imus
        data: np.array
            Numpy array of size (1,3).
            Includes an accelerometer measurement.
        """
        self.data[pose_name][imu_name] = \
            np.append(self.data[pose_name][imu_name], np.array([data]), axis=0)

    def clean_data(self, verbose=False):
        """
        Cleans the data

        Arguments
        ---------
        `verbose`: `bool`
        """
        # Create nested dictionary to store data
        # retrigger
        data = copy.deepcopy(self.data)
        for pose_name in self.pose_names:
            for imu_name in self.imu_names:
                # 0th IMU is a fake IMU
                if imu_name == 'imu_link0':
                    quaternion = np.zeros(4)
                    acceleration = np.zeros(3)
                    joint = np.zeros(7)
                # For other IMUs
                else:
                    d = self.data[pose_name][imu_name]
                    # Quaternions
                    quaternions = reject_outliers(d[:, :4])
                    quaternion = np.mean(quaternions, axis=0)
                    # Linear Accelerations
                    accelerations = reject_outliers(d[:, 4:7])
                    acceleration = np.mean(accelerations, axis=0)
                    # Joint Angles
                    joints = reject_outliers(d[:, 7:])
                    joint = np.mean(joints, axis=0)

                data[pose_name][imu_name] = np.r_[quaternion, acceleration, joint]

                if verbose:
                    rospy.loginfo(
                        '[%s, %s] Mean Acceleration: (%.3f %.3f %.3f)' % (
                            pose_name, imu_name, acceleration[0], acceleration[1], acceleration[2]
                        )
                    )

        return data

    def save(self, data):
        """
        Save both the original collected data and the averaged data

        Arguments
        ---------
        `data`: `OrderedDict`
            The data from static data collection, saved to a pickle file.
        """
        with open(self.filepath, 'wb') as f:
            pickle.dump(data, f)


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
        with open(self.filepath, 'wb') as f:
            pickle.dump(data, f)
