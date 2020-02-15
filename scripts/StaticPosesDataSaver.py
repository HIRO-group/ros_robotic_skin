#!/usr/bin/env python
from SawyerPose import SawyerPose
from PandaPose import PandaPose
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from collections import defaultdict
import pickle

################################################
# Poses Configuration #####################
# Explained in detail in PandaPose file
################################################

class StaticPoseDataSaver():
    def __init__(self, robot_pose):
        self.rp = robot_pose
        self.rp.pose_string = ''
        # constant
        self.gravitation_constant = rospy.get_param('/gravity_constant')
        # data storage
        # TODO: Reduce to just 1 storage
        self.np_array_storage = np.array([['', '', '0', '0', '0']])
        self.data_ordered_dict = defaultdict(list)
        # Subscribe to IMUs
        self.get_imu_data()

    def callback(self, data):
        acceleration_data = data.linear_acceleration
        self.np_array_storage = np.vstack((self.np_array_storage,
                                    [self.rp.pose_string, data.header.frame_id, acceleration_data.x, acceleration_data.y,
                                    acceleration_data.z]))

    def get_imu_data(self):
        imu_list = ['imu_data0', 'imu_data1', 'imu_data2', 'imu_data3', 'imu_data4', 'imu_data5', 'imu_data6']
        for each_imu in imu_list:
            rospy.Subscriber(each_imu, Imu, self.callback)

    def set_poses(self, poses_list):
        self.rp.set_positions_list(poses_list, sleep=1)

    def structure_collected_data(self):
        for every_entry in self.np_array_storage:
            if not self.data_ordered_dict[every_entry[0]]:
                self.data_ordered_dict[every_entry[0]] = defaultdict(list)
                self.data_ordered_dict[every_entry[0]][every_entry[1]] = []
            elif not self.data_ordered_dict[every_entry[0]][every_entry[1]]:
                self.data_ordered_dict[every_entry[0]][every_entry[1]] = []
            self.data_ordered_dict[every_entry[0]][every_entry[1]].append(
                [every_entry[2], every_entry[3], every_entry[4]])
        # Delete the data_ordered_dict[''], because it ain't useful
        del self.data_ordered_dict['']
        for pose, imu_links in self.data_ordered_dict.items():
            for imu_link in imu_links:
                resulted_np_array = np.array(self.data_ordered_dict[pose][imu_link]).astype(np.float)
                avg_array = np.mean(resulted_np_array, axis=0) / self.gravitation_constant
                self.data_ordered_dict[pose][imu_link] = avg_array
        self.save_array_to_file()

    def save_array_to_file(self):
        with open('data/static_data.pickle', 'wb') as f:
            pickle.dump(self.data_ordered_dict, f)

# Lets generate poses for review
if __name__ == "__main__":
    poses_list = [
        [[-1, -1, -1, 0, -3, -1, -1], [], 'Pose_1']
    ]

    robot_pose = PandaPose()
    #robot_pose = SawyerPose()
    sd = StaticPoseDataSaver(robot_pose)
    sd.get_imu_data()
    sd.set_poses(poses_list)
    sd.structure_collected_data()
