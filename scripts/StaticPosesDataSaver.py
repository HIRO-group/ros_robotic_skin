from PandaPose import PandaPose
import numpy as np
import random
import rospy
from sensor_msgs.msg import Imu

################################################
###### Poses Configuration #####################
poses_list = [
    [[-1, -1, -1, -3, -1, -1, -1], [], 'Pose_1']
]
################################################
np_array_imu_0 = np.array([0, 0, 0])
np_array_imu_1 = np.array([0, 0, 0])
np_array_imu_2 = np.array([0, 0, 0])
np_array_imu_3 = np.array([0, 0, 0])
np_array_imu_4 = np.array([0, 0, 0])
np_array_imu_5 = np.array([0, 0, 0])
np_array_imu_6 = np.array([0, 0, 0])


def callback(data):
    if data.header.frame_id == 'imu_link0':
        acceleration_data = data.linear_acceleration
        np.vstack((np_array_imu_0, [acceleration_data.x, acceleration_data.y, acceleration_data.z]))

    elif data.header.frame_id == 'imu_link1':
        acceleration_data = data.linear_acceleration
        np.vstack((np_array_imu_1, [acceleration_data.x, acceleration_data.y, acceleration_data.z]))

    elif data.header.frame_id == 'imu_link2':
        acceleration_data = data.linear_acceleration
        np.vstack((np_array_imu_2, [acceleration_data.x, acceleration_data.y, acceleration_data.z]))

    elif data.header.frame_id == 'imu_link3':
        acceleration_data = data.linear_acceleration
        np.vstack((np_array_imu_3, [acceleration_data.x, acceleration_data.y, acceleration_data.z]))

    elif data.header.frame_id == 'imu_link4':
        acceleration_data = data.linear_acceleration
        np.vstack((np_array_imu_4, [acceleration_data.x, acceleration_data.y, acceleration_data.z]))

    elif data.header.frame_id == 'imu_link5':
        acceleration_data = data.linear_acceleration
        np.vstack((np_array_imu_5, [acceleration_data.x, acceleration_data.y, acceleration_data.z]))

    elif data.header.frame_id == 'imu_link6':
        acceleration_data = data.linear_acceleration
        np.vstack((np_array_imu_6, [acceleration_data.x, acceleration_data.y, acceleration_data.z]))

    else:
        raise Exception("I don't know what IMU link I am getting. Should I be getting this?: ", data.header.frame_id)


class PandaPosesDataSaver(PandaPose):
    def __init__(self):
        super(PandaPosesDataSaver, self).__init__()
        self.pose_string = ''

    def get_imu_data(self):
        rospy.Subscriber('imu_data0', Imu, callback)
        rospy.Subscriber('imu_data1', Imu, callback)
        rospy.Subscriber('imu_data2', Imu, callback)
        rospy.Subscriber('imu_data3', Imu, callback)
        rospy.Subscriber('imu_data4', Imu, callback)
        rospy.Subscriber('imu_data5', Imu, callback)
        rospy.Subscriber('imu_data6', Imu, callback)
        rospy.spin()

    def set_poses(self):
        self.set_poses_position(poses_list)


# Lets generate poses for review
if __name__ == "__main__":
    pp = PandaPosesDataSaver()
    pp.get_imu_data()
    pp.set_poses()
