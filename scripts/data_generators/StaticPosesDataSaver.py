#!/usr/bin/env python
import os
import sys
from collections import OrderedDict
import copy
import numpy as np
import pickle

import rospy
import rospkg
from sensor_msgs.msg import Imu

sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts.utils import get_poses_list_file  # noqa: E402
from scripts.controllers.PandaController import PandaController  # noqa: E402
from scripts.controllers.SawyerController import SawyerController  # noqa: E402


RAD2DEG = 180.0/np.pi


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
            for imu_name in imu_names:
                self.data[pose_name][imu_name] = np.empty((0, 10), float)

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
        data = copy.deepcopy(self.data)
        for pose_name in self.pose_names:
            for imu_name in self.imu_names:
                d = reject_outliers(self.data[pose_name][imu_name][:, :3])
                m = np.mean(d, axis=0)
                # s = np.std(d, axis=0)
                joints = reject_outliers(self.data[pose_name][imu_name][:, 3:])
                j = np.mean(joints, axis=0)
                data[pose_name][imu_name] = np.r_[m, j]

                if verbose:
                    rospy.loginfo('[%s, %s] Mean Acceleration: (%.3f %.3f %.3f)' % (pose_name, imu_name, m[0], m[1], m[2]))

        return data

    def save(self, data):
        """
        Save both the original collected data and the averaged data

        Arguments
        ---------
        `data`: `OrderedDict`
            The data from static data collection, saved to a pickle file.
        """
        ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
        filepath = os.path.join(ros_robotic_skin_path, self.filepath+'.pickle')

        with open(filepath, 'wb') as f:
            pickle.dump(data, f)


class StaticPoseDataSaver():
    """
    Class for collecting static pose data nd save them as a pickle file
    """
    def __init__(self, controller, poses_list, filepath='data/static_data'):
        """
        Initializes StaticPoseDataSaver class.

        Arguments
        -----------
        controller:
            Wrapped controller to control either Panda or Sawyer robot.
        poses_list: list
            A list of poses. Each pose is a list.
            It includes 7 joint positiosn, 7 joint velociites,
            and the Pose name
        filepath: str
            File path to save the collected data
        """
        self.controller = controller
        self.poses_list = poses_list

        self.pose_names = [pose[2] for pose in poses_list]
        self.joint_names = self.controller.joint_names
        self.imu_names = ['imu_link0', 'imu_link1', 'imu_link2',
                          'imu_link3', 'imu_link4', 'imu_link5', 'imu_link6']
        self.imu_topics = ['imu_data0', 'imu_data1', 'imu_data2',
                           'imu_data3', 'imu_data4', 'imu_data5', 'imu_data6']
        self.curr_pose_name = self.pose_names[0]
        self.ready = False

        # data storage
        self.data_storage = StaticPoseData(self.pose_names, self.imu_names, filepath)

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
        if self.ready:
            accel = data.linear_acceleration
            joint_angles = [self.controller.joint_angle(name) for name in self.joint_names]

            self.data_storage.append(
                self.curr_pose_name,            # for each defined initial pose
                data.header.frame_id,           # for each imu
                np.array([accel.x, accel.y, accel.z] + joint_angles))

    def set_poses(self, time=3.0):
        """
        Move to the defined poses and collect the IMU data
        for a given amount of time.

        time: float
            Time to collect the IMU data
        """
        for pose in self.poses_list:
            positions, _, pose_name = pose[0], pose[1], pose[2]  # noqa: F841
            self.controller.publish_positions(positions, 0.1)
            print('At Position: ' + pose_name, map(int, RAD2DEG*np.array(positions)))
            self.curr_pose_name = pose_name
            rospy.sleep(0.5)
            self.ready = True
            rospy.sleep(time)
            self.ready = False

    def save(self, save=True, verbose=False):
        """
        Save data to a pickle file.
        """
        data = self.data_storage.clean_data(verbose)

        if save:
            self.data_storage.save(data)


if __name__ == "__main__":
    # get poses from file?
    robot = sys.argv[1]
    if robot == 'panda':
        controller = PandaController()
        poses_list = get_poses_list_file('positions.txt')
    elif robot == 'sawyer':
        controller = SawyerController()
    else:
        raise ValueError("Must be either panda or sawyer")

    if len(sys.argv) > 2:
        try:
            poses_list = get_poses_list_file(sys.argv[2])
        except Exception:
            raise Exception("Could not initiate poses_lists from file!")
    else:
        # Poses Configuration
        poses_list = [
            [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], 'Pose_1'],
            [[3.47, -2.37, 1.38, 0.22, 3.13, 1.54, 1.16], [], 'Pose_2'],
            [[-1.10, -2.08, 5.68, 1.41, 4.13, 0.24, 2.70], [], 'Pose_3'],
            [[-0.75, -1.60, 1.56, 4.43, 1.54, 4.59, 6.61], [], 'Pose_4'],
            [[-0.61, -0.54, 3.76, 3.91, 5.05, 0.92, 6.88], [], 'Pose_5'],
            [[-1.39, -0.87, 4.01, 3.75, 5.56, 2.98, 4.88], [], 'Pose_6'],
            [[1.51, -2.47, 3.20, 1.29, 0.24, 4.91, 8.21], [], 'Pose_7'],
            [[0.25, -0.18, 5.13, 5.43, 2.78, 3.86, 6.72], [], 'Pose_8'],
            [[0.76, -1.96, 2.24, 1.54, 4.19, 5.22, 7.46], [], 'Pose_9'],
            [[0.03, -1.09, 2.63, 0.33, 3.87, 0.88, 2.92], [], 'Pose_10'],
            [[0.72, -1.00, 6.09, 2.61, 1.10, 4.13, 3.06], [], 'Pose_11'],
            [[1.69, -2.72, 0.14, 1.08, 2.14, 0.08, 9.13], [], 'Pose_12'],
            [[0.81, -1.89, 3.26, 1.42, 5.64, 0.14, 8.34], [], 'Pose_13'],
            [[-0.90, -3.10, 3.24, 0.16, 4.81, 4.94, 4.35], [], 'Pose_14'],
            [[1.36, -1.89, 2.73, 1.20, 3.08, 3.29, 3.88], [], 'Pose_15'],
            [[-0.36, -2.19, 3.91, 0.04, 2.15, 3.19, 5.18], [], 'Pose_16'],
            [[5.25, -0.55, 0.98, 4.15, 5.65, 3.65, 9.27], [], 'Pose_17'],
            [[2.52, -2.54, 2.07, 0.55, 3.26, 2.31, 4.72], [], 'Pose_18'],
            [[4.63, -0.70, 3.14, 3.41, 3.55, 0.69, 6.10], [], 'Pose_19'],
            [[5.41, -0.90, 5.86, 0.41, 1.69, 1.23, 4.34], [], 'Pose_20']
        ]

    filepath = '_'.join(['data/static_data', robot])
    sd = StaticPoseDataSaver(controller, poses_list, filepath)
    sd.set_poses()
    # sd.save(save=True, verbose=True)