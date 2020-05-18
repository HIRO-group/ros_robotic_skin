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
from scripts import utils  # noqa: E402
from scripts.controllers.RobotController import PandaController, SawyerController  # noqa: E402


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
        data = copy.deepcopy(self.data)
        for pose_name in self.pose_names:
            for imu_name in self.imu_names:
                d = reject_outliers(self.data[pose_name][imu_name][:, 4:7])
                m = np.mean(d, axis=0)
                # s = np.std(d, axis=0)
                joints = reject_outliers(self.data[pose_name][imu_name][:, 7:])
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
        # get imu names (with connected link info) and topic names.
        self.imu_names, self.imu_topics = utils.get_imu_names_and_topics()

        self.curr_pose_name = self.pose_names[0]
        self.ready = False

        # data storage
        self.data_storage = StaticPoseData(self.pose_names, self.imu_names, filepath)

        # Subscribe to IMUs
        for imu_topic in self.imu_topics:
            rospy.Subscriber(imu_topic, Imu, self.callback)

    def callback(self, data):
        """
        A callback function for IMU topics.

        Arguments
        ----------
        data: sensor_msgs.msg.Imu
            IMU data. Please refer to the official documentation.
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
        """
        if self.ready:
            accel = data.linear_acceleration
            qx = data.orientation.x
            qy = data.orientation.y
            qz = data.orientation.z
            qw = data.orientation.w
            joint_angles = [self.controller.joint_angle(name) for name in self.joint_names]

            self.data_storage.append(
                self.curr_pose_name,            # for each defined initial pose
                data.header.frame_id,           # for each imu
                np.array([qx, qy, qz, qw,
                          accel.x, accel.y, accel.z] + joint_angles))

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
    # robot = sys.argv[1]
    robot = 'panda'
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
    filepath = '_'.join(['data/static_data', robot])

    sd = StaticPoseDataSaver(controller, poses_list, filepath)
    sd.set_poses()
    sd.save(save=True, verbose=True)
