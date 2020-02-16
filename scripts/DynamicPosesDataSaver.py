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
        joint_int = [idx for idx, val in enumerate(self.poses_list[0][0]) if val != 0][0] + 1
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
        self.controller._limb.set_joint_position_speed(speed=1.0)

        # TODO: decide on Amplitude and frequency
        dt = 1/rospy.get_param('/dynamic_frequency')
        t = 0.0
        freq = 2.0
        A = 2.0

        for pose in self.poses_list:
            positions, _, pose_string = pose[0], pose[1], pose[2]
            self.controller.pose_string = pose_string
            self.controller.publish_positions(positions, sleep=1)
            print('At Position: ' + pose_string, positions)

            for i in range(7):
                velocities = [0.0]*7
                accelerations = [0.0]*7
                d1 = datetime.datetime.now() + datetime.timedelta(seconds=5)
                poss = positions
                while True:
                    position = A/(2*pi*freq)*math.cos(2 * pi * freq * t)
                    velocity = A*math.sin(2 * pi * freq * t)
                    acceleration = 2*pi*freq*A*math.cos(2 * pi * freq * t)
                    poss[i] = positions[i] + position
                    velocities[i] = velocity
                    accelerations[i] = acceleration
                    #self.controller.publish_velocities(velocities)
                    self.controller.publish_trajectory(positions, velocities, accelerations)

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

    # [Pose, Joint, IMU, x, y, z]* number os samples according to hertz
    # controller = PandaController()
    controller = SawyerController()
    dd = DynamicPoseDataSaver(controller, poses_list)
    dd.set_poses()
    dd.structure_collected_data()
