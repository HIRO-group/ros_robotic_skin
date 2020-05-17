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
from std_msgs.msg import Float32

sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts import utils  # noqa: E402
from scripts.controllers.RobotController import PandaController, SawyerController  # noqa: E402


RAD2DEG = 180.0 / np.pi
OSCILLATION_TIME = 3.0


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

        # Create nested dictionary to store data
        for pose_name in pose_names:
            self.data[pose_name] = OrderedDict()
            for joint_name in joint_names:
                self.data[pose_name][joint_name] = OrderedDict()
                for imu_name in self.imu_names:
                    self.data[pose_name][joint_name][imu_name] = np.empty((0, 14), float)

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
                    # ang_vels = imu_data[:, 5]
                    joint_accs = imu_data[:, 6]
                    if eliminate_outliers:
                        # use hampel filter for outlier detection
                        # it actually doesn't affect the end result much.
                        norms = utils.hampel_filter_forloop(norms, 10)[0]
                        joint_accs = utils.hampel_filter_forloop(joint_accs, 10)[0]

                    # filter acceleration norms
                    filtered_norms = utils.low_pass_filter(norms, 100.)
                    # filtered_vels = utils.low_pass_filter(ang_vels, 100.)

                    # filter joint accelerations
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
                        cur_time = imu_data[idx, 3]
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

                    # d = self.data[pose_name][joint_name][imu_name][2:, :]
                    # norms = np.linalg.norm(d[:, :3], axis=1)
                    # norms, outliers_index = hampel_filter_forloop(norms, 10, 1)
                    # idx = np.argmax(norms)

                    # # Save maximum angular velocity A of all time
                    # w = d[:, 3]
                    # w, outliers_index = hampel_filter_forloop(w, 10, 1)
                    # max_w_idx = np.argmax(w)

                    # joint_accel = d[:, 4]
                    # ja, outliers_index = hampel_filter_forloop(joint_accel, 10, 1)
                    # max_ja_idx = np.argmax(ja)
                    """
                    joints = d[:, 4:]
                    window = 50
                    rospy.loginfo(
                      utils.n2s(
                      self.data[pose_name]
                      [joint_name]
                      [imu_name][idx-window:idx+window+1, :7], 3))
                    if joint_name == self.joint_names[0]
                        and (imu_name in self.imu_names[:2]):
                        rospy.loginfo(
                            '[%s, %s, %s]'%(pose_name, joint_name, imu_name))
                        rospy.loginfo(utils.n2s(d[:window+1, :9], 3))
                        rospy.loginfo(
                            '[%ith Norm=%.3f (%.2f, %.2f, %.2f),
                            w=%.2f, joint accel=%.2f'%\
                            (idx, norms[idx], d[idx,0],
                            d[idx,1], d[idx,2], w[idx], ja[idx]))
                        rospy.loginfo(
                            '[%ith Norm=%.3f (%.2f, %.2f, %.2f),
                            w=%.2f, joint accel=%.2f'%\
                            (max_w_idx,
                            norms[max_w_idx], d[max_w_idx,0],
                            d[max_w_idx,1], d[max_w_idx,2],
                            w[max_w_idx], ja[max_w_idx]))
                        rospy.loginfo('[%ith Norm=%.3f (%.2f, %.2f, %.2f),
                            w=%.2f, joint accel=%.2f'%\
                            (max_ja_idx, norms[max_ja_idx],
                            d[max_ja_idx,0], d[max_ja_idx,1],
                            d[max_ja_idx,2], w[max_ja_idx], ja[max_ja_idx]))
                    """

                    # data[pose_name][joint_name][imu_name] = self.data[pose_name][joint_name][imu_name][idx, :]
                    # data[pose_name][joint_name][imu_name][3] = w[idx]
                    # data[pose_name][joint_name][imu_name][4] = w[max_w_idx]

                    if verbose:
                        rospy.loginfo(data[pose_name][joint_name][imu_name])
                        d = data[pose_name][joint_name][imu_name]
                        rospy.loginfo('[%s, %s, %s] (%.3f, %.3f, %.3f)'
                                      % (pose_name, joint_name, imu_name, d[0], d[1], d[2]))
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
        self.time = None
        # constant
        self.pose_names = [pose[2] for pose in poses_list]
        self.joint_names = self.controller.joint_names

        # get imu names and topics through rostopic and xacro.
        self.imu_names, self.imu_topics = utils.get_imu_names_and_topics()

        """
        to-do get the joints the imus are connected to.
        """
        self.ready = False
        self.curr_positions = [0, 0, 0, 0, 0, 0, 0]
        self.curr_pose_name = self.pose_names[0]
        self.curr_joint_name = self.joint_names[0]
        self.max_angular_velocity = -np.inf
        self.prev_w = 0.0
        self.prev_t = 0.000001
        self.curr_acc = 0.0
        self.freq = rospy.get_param('/oscillation_frequency')
        self.A = rospy.get_param('/oscillation_magnitude')
        self.pubs = {}
        # add publishers that publish norm of imu data.
        for imu_name in self.imu_names:
            self.pubs[imu_name] = rospy.Publisher('/Anorm%s' % (list(imu_name)[-1]), Float32, queue_size=10)
        rospy.loginfo(self.joint_names)

        # data storage
        self.data_storage = DynamicPoseData(self.pose_names, self.joint_names, self.imu_names, filepath)
        rospy.sleep(1)
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
            msg = Float32()
            msg.data = np.linalg.norm(np.array([accel.x, accel.y, accel.z]))
            self.pubs[data.header.frame_id].publish(msg)
            if self.curr_joint_name == self.joint_names[0] and data.header.frame_id == 'imu_link0':
                pass
                # rospy.loginfo(utils.n2s(
                #   np.array([accel.x, accel.y, accel.z])))
            curr_t = rospy.get_rostime().to_sec()
            dt = curr_t - self.prev_t

            curr_w = self.controller.joint_velocity(self.curr_joint_name)
            curr_A = abs(curr_w)

            try:
                self.curr_acc = (curr_w - self.prev_w) / dt
            except Exception:
                pass

            if curr_A > self.max_angular_velocity:
                self.max_angular_velocity = curr_A
            curr_w = self.controller.joint_velocity(self.curr_joint_name)
            # time
            t = self.time if self.time is not None else -1
            self.data_storage.append(
                self.curr_pose_name,            # for each defined initial pose
                self.curr_joint_name,           # for each excited joint
                data.header.frame_id,           # for each imu
                np.array([accel.x, accel.y, accel.z,
                          t, self.A, curr_w, self.curr_acc] + joint_angles))

    def move_like_sine_dynamic(self):
        """
        This will move the joint of the robot arm like a sine wave
        for all joints for all defined poses.
        """
        self.controller.set_joint_position_speed(speed=1.0)

        for pose in self.poses_list:
            positions, _, pose_name = pose[0], pose[1], pose[2]  # noqa: F841
            self.curr_positions = positions
            self.curr_pose_name = pose_name
            # first, move to the position from <robot>_positions.txt
            self.controller.publish_positions(positions, sleep=2)
            print('At Position: ' + pose_name,
                  map(int, RAD2DEG*np.array(positions)))

            # each joint in pose p

            for i, joint_name in enumerate(self.joint_names):
                self.curr_joint_name = joint_name
                # max_angular velocity
                self.max_angular_velocity = -np.inf
                # go back to main position
                self.controller.publish_positions(positions, sleep=2)

                self.prev_w = self.controller.joint_velocity(self.curr_joint_name)
                self.prev_t = rospy.get_rostime().to_sec()

                # Prepare for publishing velocities
                velocities = np.zeros(len(self.joint_names))

                # stopping time
                self.ready = True
                now = rospy.get_rostime()
                while True:
                    # time within motion
                    t = (rospy.get_rostime() - now).to_sec()
                    self.time = t

                    # Oscillated Velocity pattern
                    velocity = self.A * np.sin(2 * pi * self.freq * t)

                    velocities[i] = velocity
                    self.controller.send_velocities(velocities)
                    if t > OSCILLATION_TIME:
                        break
                        # pass
                    self.controller.r.sleep()
                self.time = None
                # also stops sending data
                self.ready = False
                rospy.sleep(1)

    def save(self, save=True, verbose=False):
        """
        Save data to a pickle file.


        Arguments
        ----------
        `save`: `bool`
            If the data will be saved

        `verbose`: `bool`
        """

        data = self.data_storage.clean_data(verbose)

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
    dd.save(save=True, verbose=False)
