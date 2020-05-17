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
    def test(self):
        rc = PandaController(7)
        names = rc.joint_names
        # rc.publish_positions([0, 0, 0, 0, 0, 0, 0])
        # self.assertAlmostEqual(rc)
        self.assertEqual(len(names), 7)


if __name__ == '__main__':
    rospy.sleep(30)
    rostest.rosrun('ros_robotic_skin', 'test_main', MainTestCase)
