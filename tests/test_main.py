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
from controllers.RobotController import RobotController
os.system("pwd")


class MainTestCase(unittest.TestCase):
    def tester(self):
        rc = RobotController(7)
        pass


if __name__ == '__main__':
    rostest.rosrun('ros_robotic_skin', 'test_main', MainTestCase)
