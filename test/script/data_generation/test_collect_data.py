#! /usr/bin/env python
import os
import unittest
import rospy
import rospkg

ROS_ROBOTIC_SKIN_PATH = rospkg.RosPack().get_path('ros_robotic_skin')
SAVEDIR = os.path.join(ROS_ROBOTIC_SKIN_PATH, 'data')


class TestCollectData(unittest.TestCase):
    def test_file_created(self):
        # Check if data is collected
        filepath = os.path.join(SAVEDIR, 'static_data_panda.pickle')
        self.assertTrue(os.path.isfile(filepath))

        filepath = os.path.join(SAVEDIR, 'dynamic_data_panda.pickle')
        self.assertTrue(os.path.isfile(filepath))


if __name__ == '__main__':
    import rostest
    rostest.rosrun('ros_robotic_skin', 'test_collect_data', TestCollectData)
