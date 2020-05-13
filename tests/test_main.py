#!/usr/bin/env python

import os
import sys
import unittest
import rostest
import rospkg
import rospy

ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
controllers_path = os.path.join(ros_robotic_skin_path, 'scripts')
sys.path.insert(0, controllers_path)
from controllers.RobotController import PandaController  # noqa: E402


class MainTestCase(unittest.TestCase):
    def tester(self):
        rc = PandaController(7)
        rc.publish_positions([1, 1, 1, 1, 1, 1, 1])
        rc.publish_positions([0, 0, 0, 0, 0, 0, 0])

        # self.assertAlmostEqual(rc)
        self.assertTrue(rc.is_sim)


if __name__ == '__main__':
    rospy.sleep(30)
    rostest.rosrun('ros_robotic_skin', 'test_main', MainTestCase)
