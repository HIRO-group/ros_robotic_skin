#!/usr/bin/env python

import os
import sys
import unittest
import rostest

sys.path.insert(0, os.path.abspath('..'))


class MainTestCase(unittest.TestCase):
    def test_(self):
        pass


if __name__ == '__main__':
    rostest.rosrun('ros_robotic_skin', 'test_main', MainTestCase)
