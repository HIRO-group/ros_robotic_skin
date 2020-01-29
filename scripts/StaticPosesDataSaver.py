from PandaPose import PandaPose
import numpy as np
import random
import rospy
from sensor_msgs.msg import Imu
from collections import OrderedDict, defaultdict

################################################
###### Poses Configuration #####################
poses_list = [
    [[-1, -1, -1, -3, -1, -1, -1], [], 'Pose_1']
]
################################################
global np_array_storage


def callback(data):
    global pp
    global np_array_storage
    if data.header.frame_id == 'imu_link0':
        acceleration_data = data.linear_acceleration
        np_array_storage = np.vstack((np_array_storage,
                                      [pp.pose_string, 'imu_link0', acceleration_data.x, acceleration_data.y,
                                       acceleration_data.z]))

    elif data.header.frame_id == 'imu_link1':
        acceleration_data = data.linear_acceleration
        np.vstack((np_array_storage,
                   [pp.pose_string, 'imu_link1', acceleration_data.x, acceleration_data.y, acceleration_data.z]))

    elif data.header.frame_id == 'imu_link2':
        acceleration_data = data.linear_acceleration
        np_array_storage = np.vstack((np_array_storage,
                                      [pp.pose_string, 'imu_link2', acceleration_data.x, acceleration_data.y,
                                       acceleration_data.z]))

    elif data.header.frame_id == 'imu_link3':
        acceleration_data = data.linear_acceleration
        np_array_storage = np.vstack((np_array_storage,
                                      [pp.pose_string, 'imu_link3', acceleration_data.x, acceleration_data.y,
                                       acceleration_data.z]))

    elif data.header.frame_id == 'imu_link4':
        acceleration_data = data.linear_acceleration
        np_array_storage = np.vstack((np_array_storage,
                                      [pp.pose_string, 'imu_link4', acceleration_data.x, acceleration_data.y,
                                       acceleration_data.z]))

    elif data.header.frame_id == 'imu_link5':
        acceleration_data = data.linear_acceleration
        np_array_storage = np.vstack((np_array_storage,
                                      [pp.pose_string, 'imu_link5', acceleration_data.x, acceleration_data.y,
                                       acceleration_data.z]))

    elif data.header.frame_id == 'imu_link6':
        acceleration_data = data.linear_acceleration
        np_array_storage = np.vstack((np_array_storage,
                                      [pp.pose_string, 'imu_link6', acceleration_data.x, acceleration_data.y,
                                       acceleration_data.z]))

    else:
        raise Exception("I don't know what IMU link I am getting. Should I be getting this?: ", data.header.frame_id)


class PandaPosesDataSaver(PandaPose):
    def __init__(self):
        super(PandaPosesDataSaver, self).__init__()
        self.pose_string = ''
        self.data_ordered_dict = defaultdict(list)

    def get_imu_data(self):
        rospy.Subscriber('imu_data0', Imu, callback)
        rospy.Subscriber('imu_data1', Imu, callback)
        rospy.Subscriber('imu_data2', Imu, callback)
        rospy.Subscriber('imu_data3', Imu, callback)
        rospy.Subscriber('imu_data4', Imu, callback)
        rospy.Subscriber('imu_data5', Imu, callback)
        rospy.Subscriber('imu_data6', Imu, callback)
        # rospy.spin()

    def set_poses(self):
        self.set_poses_position_static(poses_list)

    def ready_the_data(self):
        global np_array_storage
        for every_entry in np_array_storage:
            # if self.data_ordered_dict[every_entry[1]][every_entry[0]]:
            #     self.data_ordered_dict[every_entry[1]][every_entry[0]] = [0, 0, 0]
            # self.data_ordered_dict[every_entry[1]][every_entry[0]].append(
            #     [every_entry[2], every_entry[3], every_entry[4]])
            if not self.data_ordered_dict[every_entry[0]]:
                self.data_ordered_dict[every_entry[0]] = defaultdict(list)
                self.data_ordered_dict[every_entry[0]][every_entry[1]] = []
            elif not self.data_ordered_dict[every_entry[0]][every_entry[1]]:
                self.data_ordered_dict[every_entry[0]][every_entry[1]] = []
            self.data_ordered_dict[every_entry[0]][every_entry[1]].append(
                [every_entry[2], every_entry[3], every_entry[4]])
        print(self.data_ordered_dict)


# Lets generate poses for review
if __name__ == "__main__":
    # global np_array_storage
    np_array_storage = np.array([['', '', 0, 0, 0]])
    pp = PandaPosesDataSaver()
    pp.get_imu_data()
    pp.set_poses()
    pp.ready_the_data()
