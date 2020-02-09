"""
Setting the pose and getting linear accelerations of IMU in interest.
You have to add ros_robotic_skin to PYTHONPATH for the PandaPose module to be imported correctly.Better add it to
bashrc.
"""
from scripts.PandaPose import PandaPose
from sensor_msgs.msg import Imu
import rospy
from time import sleep

global interested_imu_link


def callback(data):
    """
    The accelerometer callback function. As input we would get data with many attributes
    like linear acceleration which is our favourite
    :param data: Some data Structure
        This contains attributes like linear acceleration of accelerometer in IMU
    :return:
    """
    global interested_imu_link
    if data.header.frame_id == interested_imu_link:
        acceleration_data = data.linear_acceleration
        print('X-axis acceleration: %.3f, Y-axis acceleration: %.3f, Z-axis acceleration: %.3f' % (acceleration_data.x,
              acceleration_data.y, acceleration_data.z))
    sleep(0.5)


class PoseAcceleration(PandaPose):
    """
    Main purpose of this class is to make a class and use PandaPose to get our job done
    """

    def __init__(self, pose):
        """
        Initialising the class
        :param pose: list of list
            The data structure is defined in PandaPose file
        """
        super(PoseAcceleration, self).__init__()
        self.pose = pose

    def get_imu_data(self):
        """
        Main use of this function is to initialize the callbacks to all IMU's
        :return: None
        """
        # Loop through all IMU's and subscribe to callback
        imu_list = ['imu_data0', 'imu_data1', 'imu_data2', 'imu_data3', 'imu_data4', 'imu_data5', 'imu_data6']
        for each_imu in imu_list:
            rospy.Subscriber(each_imu, Imu, callback)

    def set_pose_and_print_acceleration(self):
        """
        Set positions of franks, as well initialize callback functions to get accelerometer data
        :return: None
        """
        self.set_poses_position_static(self.pose)
        self.get_imu_data()


if __name__ == "__main__":
    # The string should be the imu_link which you want to see the acceleration values
    interested_imu_link = 'imu_link6'
    # Checking one of the axes with below poses_list. Please keep only one of them uncommented, but rest of them
    # commented
    poses_list = [
        [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1']
    ]
    poses_list = [
        [[0.0, 1.6, 0, 0.0, 0.0, 0.0, 0.0], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1']
    ]
    poses_list = [
        [[0.0, 1.6, -1.6, 0.0, 0.0, 0.0, 0.0], [0, 0, 0.5, 0, 0, 0, 0], 'Pose_1']
    ]
    PoseAcceleration(poses_list).set_pose_and_print_acceleration()
    while True:
        sleep(0.5)
