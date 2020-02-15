#!/usr/bin/env python
from collections import defaultdict
import datetime
import math
from math import pi
import numpy as np
import pickle
import rospy

from sensor_msgs.msg import Imu
from SawyerPose import SawyerPose

# TODO: Get max acceleration, as well joint angle at that point
# [pose] [joint: which is changing] [acclerometer_max, theta]
################################################
###### Poses Configuration #####################
poses_list = [
    [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1'],
    [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, -0.5, 0, 0, 0, 0], 'Pose_1']
]
################################################
global np_array_storage
global joint_int


def get_joint_int():
    global poses_list
    global joint_int
    # https://www.geeksforgeeks.org/python-index-of-non-zero-elements-in-python-list/
    joint_int = [idx for idx, val in enumerate(poses_list[0][1]) if val != 0][0] + 1
    return joint_int

def callback(data):
    global dd 
    global np_array_storage
    acceleration_data = data.linear_acceleration
    np_array_storage = np.vstack((np_array_storage,
                                  [dd.rp.pose_string, data.header.frame_id, acceleration_data.x, acceleration_data.y,
                                   acceleration_data.z]))

class DynamicPoseDataSaver():
    def __init__(self, robot_pose):
        self.rp = robot_pose
        self.rp.pose_string = ''
        self.data_ordered_dict = defaultdict(list)
        get_joint_int()

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
        self.rp.move_like_sine_dynamic()

    def move_like_sine_dynamic(self):
        """
        This will move the joint of the Panda ARM like a sine wave
        # TODO: add minutes delay
        :return:
        """
        d1 = datetime.datetime.now() + datetime.timedelta(minutes=1)
        while True:
            for each_degree in range(0, 360):
                self.rp.publish_velocity([0, 0, 5 * math.sin(math.radians(each_degree)), 0, 0, 0, 0], sleep=None)
            if d1 < datetime.datetime.now():
                break

    def ready_the_data(self):
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
                avg_array = np.mean(resulted_np_array, axis=0) / 9.81
                self.data_ordered_dict[pose][imu_link] = avg_array
        # save_file_array = np.array(self.data_ordered_dict)
        # output = open('myfile.pkl', 'wb')
        # pickle.dump(self.data_ordered_dict, output)
        # output.close()
        print(self.data_ordered_dict)


# Lets generate poses for review
if __name__ == "__main__":
    # global np_array_storage
    # [Pose, Joint, IMU, x, y, z]* number os samples according to hertz
    np_array_storage = np.array([['', 0, '', 0, 0, 0]])
    robot_pose = SawyerPose()
    dd = DynamicPoseDataSaver()
    dd.get_imu_data()
    dd.set_poses()
    dd.ready_the_data()
    # print(get_joint_int())
