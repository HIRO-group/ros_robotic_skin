#!/usr/bin/env python
import os
import sys
from collections import OrderedDict
import copy
import numpy as np
import pickle

import tf
import rospy
import rospkg
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts.utils import get_poses_list_file  # noqa: E402
from scripts.controllers.PandaController import PandaController  # noqa: E402
from scripts.controllers.SawyerController import SawyerController  # noqa: E402

RAD2DEG = 180.0/np.pi
DATA_COLLECTION_TIME = 3.0
CONSTANT_VELOCITY = 2.0


def reject_outliers(data, m=1):
    """
    Rejects outliers in a dataset.

    Arguments
    ----------
    `data`: `np.array`
        The data.

    `m`: `int`
        The amount of standard deviations from
        the mean which is considered an outlier.

    Returns
    ----------
    returns: None
    """
    is_in_std = np.absolute(data - np.mean(data, axis=0)) < m * np.std(data, axis=0)
    indices = np.where(is_in_std is True)
    return data[indices], indices


class ConstantRotationData():
    """
    Class to store dynamic pose data into a nested dictionary.
    It is stored as in [20 poses][7 excited joints][7 IMUs][data].
    self.data[pose_name][joint_name][imu_name] = np.ndarray
    The data is defined as below.
    data = [acceleration, joint angles, joint velocities].
    """
    def __init__(self, pose_names, joint_names, imu_names, filepath):
        """
        Initialize StaticPoseData class.

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

        # Create nested dictionary to store data
        for pose_name in pose_names:
            self.data[pose_name] = OrderedDict()
            for joint_name in joint_names:
                self.data[pose_name][joint_name] = OrderedDict()
                for imu_name in imu_names:
                    self.data[pose_name][joint_name][imu_name] = np.empty((0, 15), float)

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
        self.data[pose_name][joint_name][imu_name] = \
            np.append(self.data[pose_name][joint_name][imu_name], np.array([data]), axis=0)

    def clean_data(self, verbose=False):
        """
        Cleans the data.

        Arguments
        ----------
        `verbose`: `bool`
        """
        # Create nested dictionary to store data
        data = copy.deepcopy(self.data)
        for pose_name in self.pose_names:
            for joint_name in self.joint_names:
                for imu_name in self.imu_names:
                    norm = np.linalg.norm(self.data[pose_name][joint_name][imu_name][:, :3], axis=1)
                    norm, in_std = reject_outliers(norm)
                    data[pose_name][joint_name][imu_name] = self.data[pose_name][joint_name][imu_name][in_std, :]

        return data

    def save(self, data, filter=False):
        """
        Saves the data to a pickle file.

        Arguments
        ----------
        `data`: `OrderedDict`
            The data to be saved

        `filter`: `bool`
            Whether the data is filtered of values
            that are not near the constant velocity defined in this file.
        """
        ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
        filepath = os.path.join(ros_robotic_skin_path, self.filepath+'.pickle')
        if filter:
            # filters the data and gets the constant angular velocity values
            data = self.filter_data(data)
        with open(filepath, 'wb') as f:
            pickle.dump(data, f)

    def filter_data(self, data):
        """
        Filters the data along values near `CONSTANT_VELOCITY`

        Arguments
        ----------
        `data`: `OrderedDict`
            The data from the data collection.
        """
        for pose in data.keys():
            poses_dict = data[pose]
            for joint in poses_dict.keys():
                all_joints = poses_dict[joint]
                for imu in all_joints.keys():
                    imu_points = all_joints[imu][0]
                    delete_idxs = []
                    for ind, val in enumerate(imu_points):
                        joint_vel = val[-1]
                        if abs(joint_vel - CONSTANT_VELOCITY) >= 0.25:
                            delete_idxs.append(ind)
                    res = np.delete(imu_points, delete_idxs, 0)
                    y = np.expand_dims(res, axis=0)
                    all_joints[imu] = y
        return data


class ConstantRotationDataSaver():
    """
    Class for collecting dynamic pose data nd save them as a pickle file
    """
    def __init__(self, controller, poses_list, filepath='data/constant_data'):
        """
        Initializes DynamicPoseDataSaver class.

        Arguments
        -----------
        controller:
            Wrapped controller to control either Panda or Sawyer robot.
        poses_list: list
            A list of poses. Each pose is a list.
            It includes 7 joint positiosn, 7 joint velociites, and Pose name
        filepath: str
            File path to save the collected data
        """
        self.controller = controller
        self.poses_list = poses_list

        # constant
        # TODO: get imu names automatically
        self.pose_names = [pose[2] for pose in poses_list]
        self.joint_names = self.controller.joint_names
        self.imu_names = ['imu_link0', 'imu_link1', 'imu_link2',
                          'imu_link3', 'imu_link4', 'imu_link5', 'imu_link6']
        self.imu_topics = ['imu_data0', 'imu_data1', 'imu_data2',
                           'imu_data3', 'imu_data4', 'imu_data5', 'imu_data6']

        self.ready = False
        self.curr_pose_name = self.pose_names[0]
        self.curr_joint_name = self.joint_names[0]

        # data storage
        self.data_storage = ConstantRotationData(self.pose_names, self.joint_names, self.imu_names, filepath)
        rospy.sleep(1)
        # Subscribe to IMUs
        for imu_topic in self.imu_topics:
            rospy.Subscriber(imu_topic, Imu, self.callback)
        self.tf_listener = tf.TransformListener()
        self.Q = {imu_name: Quaternion() for imu_name in self.imu_names}

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
            # acceleration of skin unit, followed by its acceleration
            accel = data.linear_acceleration
            q = self.Q[data.header.frame_id]
            # get the orientation of the imu
            J = np.array([self.controller.joint_angle(name) for name in self.joint_names])
            joint_velocity = self.controller.joint_velocity(self.curr_joint_name)
            # if self.curr_joint_name == 'right_j0'
            # and data.header.frame_id == 'imu_link0':
            # rospy.loginfo(utils.n2s(np.array([accel.x, accel.y, accel.z])))
            self.data_storage.append(
                self.curr_pose_name,            # for each defined initial pose
                self.curr_joint_name,           # for each excited joint
                data.header.frame_id,           # for each imu
                np.array([
                    q.x, q.y, q.z, q.w,
                    accel.x, accel.y, accel.z,
                    J[0], J[1], J[2], J[3], J[4], J[5], J[6],
                    joint_velocity])
            )

    def rotate_at_constant_vel(self):
        """
        This will move the joint of the robot arm like a sine wave
        for all joints for all defined poses.
        """
        for pose in self.poses_list:

            positions, _, pose_name = pose[0], pose[1], pose[2]  # noqa: F841
            self.curr_pose_name = pose_name
            self.controller.publish_positions(positions, sleep=1)
            print('At Position: ' + pose_name,
                  map(int, RAD2DEG*np.array(positions)))

            for i, joint_name in enumerate(self.joint_names):
                self.curr_joint_name = joint_name
                print(joint_name)

                # Prepare for publishing a trajectory
                pos = copy.deepcopy(positions)
                velocities = np.zeros(len(self.joint_names))
                velocities[i] = CONSTANT_VELOCITY
                accelerations = np.zeros(len(self.joint_names))

                # stopping time
                self.ready = True
                now = rospy.get_rostime()
                while True:
                    dt = (rospy.get_rostime() - now).to_sec()
                    pos += (velocities*dt)

                    self.controller.publish_trajectory(pos, velocities, accelerations, None)
                    if dt > DATA_COLLECTION_TIME:
                        break

                    for imu_name in self.imu_names:
                        try:
                            (trans, rot) = self.tf_listener.lookupTransform('/world', imu_name, rospy.Time(0))
                            self.Q[imu_name].x = rot[0]
                            self.Q[imu_name].y = rot[1]
                            self.Q[imu_name].z = rot[2]
                            self.Q[imu_name].w = rot[3]
                        except (tf.LookupException,
                                tf.ConnectivityException,
                                tf.ExtrapolationException):
                            continue

                self.ready = False
                rospy.sleep(1)

    def save(self, verbose=False, filter=False):
        """
        Save data to a pickle file.
        """
        data = self.data_storage.clean_data(verbose)

        self.data_storage.save(data, filter=filter)


if __name__ == "__main__":

    # used to get poses
    poses_list = get_poses_list_file('positions.txt')
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

    # [Pose, Joint, IMU, x, y, z]* number os samples according to hertz
    robot = sys.argv[1]
    if robot == 'panda':
        controller = PandaController()
    elif robot == 'sawyer':
        controller = SawyerController()
    else:
        raise ValueError("Must be either panda or sawyer")

    filepath = '_'.join(['data/constant_data', robot])
    cr = ConstantRotationDataSaver(controller, poses_list, filepath)
    cr.rotate_at_constant_vel()
    cr.save(verbose=True, filter=True)