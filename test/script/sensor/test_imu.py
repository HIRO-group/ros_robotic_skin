#! /usr/bin/env python
import os
import unittest
import rospy
import rospkg
from sensor_msgs.msg import Imu

ROS_ROBOTIC_SKIN_PATH = rospkg.RosPack().get_path('ros_robotic_skin')
SAVEDIR = os.path.join(ROS_ROBOTIC_SKIN_PATH, 'data')


class TestIMU(unittest.TestCase):
    n_imu = 7
    received_messages = [False] * n_imu

    def test_published(self):
        print('hello')
        rospy.init_node('test_imu', anonymous=True)

        # create subscribers for num_imus topics
        for i in range(self.n_imu):
            rospy.Subscriber('imu_data{}'.format(i), Imu, self.imu_callback)

        rospy.sleep(2)

        count = 0
        while not rospy.is_shutdown() and count < 10:
            count += 1

        for i_imu in range(self.n_imu):
            self.assertTrue(self.received_messages[i_imu])

    def imu_callback(self, msg):
        i_imu = int(msg.header.frame_id[-1])
        self.received_messages[i_imu] = True


if __name__ == '__main__':
    import rostest
    rostest.rosrun('ros_robotic_skin', 'test_imu', TestIMU)
