#!/usr/bin/env python
from PandaPose import PandaPose
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from collections import defaultdict
import pickle

################################################
# Poses Configuration #####################
# Explained in detail in PandaPose file
poses_list = [
    [[-1, -1, -1, -3, -1, -1, -1], [], 'Pose_1']
]
################################################
global np_array_storage

def callback(data):
    global pp
    global np_array_storage
    acceleration_data = data.linear_acceleration
    np_array_storage = np.vstack((np_array_storage,
                                  [pp.pose_string, data.header.frame_id, acceleration_data.x, acceleration_data.y,
                                   acceleration_data.z]))


class PandaPosesDataSaver(PandaPose):
    def __init__(self):
        super(PandaPosesDataSaver, self).__init__()
        self.pose_string = ''
        self.data_ordered_dict = defaultdict(list)
        self.get_imu_data()
        self.gravitation_constant = rospy.get_param('/gravity_constant')

    def get_imu_data(self):
        imu_list = ['imu_data0', 'imu_data1', 'imu_data2', 'imu_data3', 'imu_data4', 'imu_data5', 'imu_data6']
        for each_imu in imu_list:
            rospy.Subscriber(each_imu, Imu, callback)

    def set_poses(self):
        self.set_poses_position_static(poses_list)

    def structure_collected_data(self):
        global np_array_storage
        for every_entry in np_array_storage:
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
    # global np_array_storage
    np_array_storage = np.array([['', '', '0', '0', '0']])
    pp = PandaPosesDataSaver()
    pp.get_imu_data()
    pp.set_poses()
    pp.structure_collected_data()
