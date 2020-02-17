#!/usr/bin/env python
import os, sys
from collections import defaultdict
import copy
import datetime
import math
from math import pi
import numpy as np
import pickle

import rospy, rospkg
from sensor_msgs.msg import Imu

from SawyerController import SawyerController
from PandaController import PandaController

class StaticPoseData():
    def __init__(self, pose_names, imu_names, filepath):
        """
        Arguments
        ----------
        pose_names: list[str]
        imu_names: list[str]
        filepath: str
            Path to save the collected data
        """
        self.pose_names = pose_names
        self.imu_names = imu_names
        self.filepath = filepath
        self.data = defaultdict(list)

        # Create nested dictionary to store data
        for pose_name in pose_names:
            self.data[pose_name] = defaultdict(list)
            for imu_name in imu_names:
                self.data[pose_name][imu_name] = np.empty((0, 3), float) 

    def append(self, pose_name, imu_name, data):
        """
        Arguments
        ----------
        pose_name: str
        joint_name: str
        imu_name: str
        data: np.array
            Numpy array of size (0,4). 
            Includes an accelerometer measurement and a joint angle.
        """
        self.data[pose_name][imu_name] = \
            np.append(self.data[pose_name][imu_name], np.array([data]), axis=0)

    def _save(self, data, suffix=None):
        """
        Save collected data
        """
        ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
        filepath = self.filepath if suffix is None else self.filepath+suffix
        filepath = os.path.join(ros_robotic_skin_path, filepath+'.pickle')
        
        with open(filepath, 'wb') as f:
            pickle.dump(data, f)

    def save(self):
        # save original
        self._save(self.data)

        # Create nested dictionary to store data
        data = copy.deepcopy(self.data)
        for pose_name in self.pose_names:
            for imu_name in self.imu_names:
                mean_acceleration = np.mean(self.data[pose_name][imu_name], axis=0)
                data[pose_name][imu_name] = mean_acceleration

        # save mean acceleration data
        self._save(data, "_mean")


class StaticPoseDataSaver():
    def __init__(self,controller, poses_list, filepath='data/static_data'):
        self.controller = controller
        self.poses_list = poses_list

        self.pose_names = [pose[2] for pose in poses_list]
        self.imu_names = ['imu_link0', 'imu_link1', 'imu_link2', 'imu_link3', 'imu_link4', 'imu_link5', 'imu_link6']
        self.imu_topics = ['imu_data0', 'imu_data1', 'imu_data2', 'imu_data3', 'imu_data4', 'imu_data5', 'imu_data6']
        self.curr_pose_name = self.pose_names[0] 
        self.ready = False
        
        # data storage
        # TODO: Reduce to just 1 storage
        self.data_storage = StaticPoseData(self.pose_names, self.imu_names, filepath)
        
        # Subscribe to IMUs
        for imu_topic in self.imu_topics:
            rospy.Subscriber(imu_topic, Imu, self.callback)

    def callback(self, data):
        if self.ready:
            accel = data.linear_acceleration

            self.data_storage.append(
                self.curr_pose_name,            # for each defined initial pose
                data.header.frame_id,           # for each imu  
                np.array([accel.x, accel.y, accel.z]))

    def set_poses(self):
        for pose in self.poses_list:
            positions, _, pose_name = pose[0], pose[1], pose[2]
            self.controller.publish_positions(positions, 0.1)
            self.curr_pose_name = pose_name
            self.ready = True
            rospy.sleep(2)
            self.ready = False

    def save(self):
        self.data_storage.save()

# Lets generate poses for review
if __name__ == "__main__":
    # Poses Configuration
    poses_list = [
        [[3.47, -2.37, 1.38, 0.22, 3.13, 1.54, 1.16], [], 'Pose_1'],
        [[-1.10, -2.08, 5.68, 1.41, 4.13, 0.24, 2.70], [], 'Pose_2'],
        [[-0.75, -1.60, 1.56, 4.43, 1.54, 4.59, 6.61], [], 'Pose_3'],
        [[-0.61, -0.54, 3.76, 3.91, 5.05, 0.92, 6.88], [], 'Pose_4'],
        [[-1.39, -0.87, 4.01, 3.75, 5.56, 2.98, 4.88], [], 'Pose_5'],
        [[1.51, -2.47, 3.20, 1.29, 0.24, 4.91, 8.21], [], 'Pose_6'],
        [[0.25, -0.18, 5.13, 5.43, 2.78, 3.86, 6.72], [], 'Pose_7'],
        [[0.76, -1.96, 2.24, 1.54, 4.19, 5.22, 7.46], [], 'Pose_8'],
        [[0.03, -1.09, 2.63, 0.33, 3.87, 0.88, 2.92], [], 'Pose_9'],
        [[0.72, -1.00, 6.09, 2.61, 1.10, 4.13, 3.06], [], 'Pose_10'],
        [[1.69, -2.72, 0.14, 1.08, 2.14, 0.08, 9.13], [], 'Pose_11'],
        [[0.81, -1.89, 3.26, 1.42, 5.64, 0.14, 8.34], [], 'Pose_12'],
        [[-0.90, -3.10, 3.24, 0.16, 4.81, 4.94, 4.35], [], 'Pose_13'],
        [[1.36, -1.89, 2.73, 1.20, 3.08, 3.29, 3.88], [], 'Pose_14'],
        [[-0.36, -2.19, 3.91, 0.04, 2.15, 3.19, 5.18], [], 'Pose_16'],
        [[5.25, -0.55, 0.98, 4.15, 5.65, 3.65, 9.27], [], 'Pose_17'],
        [[2.52, -2.54, 2.07, 0.55, 3.26, 2.31, 4.72], [], 'Pose_18'],
        [[4.63, -0.70, 3.14, 3.41, 3.55, 0.69, 6.10], [], 'Pose_19'],
        [[5.41, -0.90, 5.86, 0.41, 1.69, 1.23, 4.34], [], 'Pose_20']
    ]    

    arg = sys.argv[1]
    if arg == 'panda':
        controller = PandaController()
    elif arg == 'sawyer':
        controller = SawyerController()
    else:
        raise ValueError("Must be either panda or sawyer")

    filepath = 'data/static_data'
    sd = StaticPoseDataSaver(controller, poses_list, filepath)
    sd.set_poses()
    sd.save()