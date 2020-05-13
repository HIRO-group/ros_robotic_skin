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
from controllers.RobotController import PandaController


class MainTestCase(unittest.TestCase):
    def tester(self):
        rc = PandaController(7)
        self.assertTrue(rc.is_sim)


if __name__ == '__main__':
    # rospy.wait_for_service('/panda_joint_trajectory_controller/query_state')
    rospy.sleep(40)
    rostest.rosrun('ros_robotic_skin', 'test_main', MainTestCase)
