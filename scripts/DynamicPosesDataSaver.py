#!/usr/bin/env python
from collections import defaultdict
import datetime
import math
from math import pi
import numpy as np
import pickle
import rospy

from sensor_msgs.msg import Imu
from SawyerController import SawyerController
from PandaController import PandaController

class DynamicPoseDataSaver():
    def __init__(self, controller, poses_list):
        self.controller = controller
        self.controller.pose_string = ''
        # constant
        self.poses_list = poses_list
        # data storage 
        self.np_array_storage = np.array([['', 0, '', 0, 0, 0]])
        self.np_array_storage = np.array([['', '', '0', '0', '0']])
        self.data_ordered_dict = defaultdict(list)
        # Subscribe to IMUs
        self.get_joint_int()
        self.get_imu_data()
    
    def get_joint_int(self):
        joint_int = [idx for idx, val in enumerate(self.poses_list[0][1]) if val != 0][0] + 1
        return joint_int
    
    def callback(self, data):
        acceleration_data = data.linear_acceleration
        self.np_array_storage = np.vstack((self.np_array_storage,
                                    [self.controller.pose_string, data.header.frame_id, acceleration_data.x, acceleration_data.y,
                                    acceleration_data.z]))

    def get_imu_data(self):
        imu_list = ['imu_data0', 'imu_data1', 'imu_data2', 'imu_data3', 'imu_data4', 'imu_data5', 'imu_data6']
        for each_imu in imu_list:
            rospy.Subscriber(each_imu, Imu, self.callback)

    def set_poses(self):
        self.move_like_sine_dynamic()

    def move_like_sine_dynamic(self):
        """
        This will move the joint of the Panda ARM like a sine wave
        # TODO: add minutes delay
        :return:
        """
        d1 = datetime.datetime.now() + datetime.timedelta(minutes=1)
        self.controller._limb.set_joint_position_speed(speed=1.0)

        dt = 1/rospy.get_param('/dynamic_frequency')
        t = 0.0
        freq = 2.0

        while True:
            velocity = 2*math.sin(2 * pi * freq * t)
            self.controller.publish_velocities([velocity, 0, 0, 0, 0, 0, 0])
            t += dt
            if d1 < datetime.datetime.now():
                break

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
                avg_array = np.mean(resulted_np_array, axis=0) / 9.81
                self.data_ordered_dict[pose][imu_link] = avg_array
        # save_file_array = np.array(self.data_ordered_dict)
        # output = open('myfile.pkl', 'wb')
        # pickle.dump(self.data_ordered_dict, output)
        # output.close()
        print(self.data_ordered_dict)


# Lets generate poses for review
# TODO: Get max acceleration, as well joint angle at that point
if __name__ == "__main__":
    # Poses Configuration
    # [pose] [joint: which is changing] [acclerometer_max, theta]
    poses_list = [
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1'],
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0, 0, -0.5, 0, 0, 0, 0], 'Pose_1']
    ]
    
    # [Pose, Joint, IMU, x, y, z]* number os samples according to hertz
    # controller = PandaController()
    controller = SawyerController()
    dd = DynamicPoseDataSaver(controller, poses_list)
    dd.set_poses()
    dd.structure_collected_data()
