#!/usr/bin/env python

import os
import sys
import unittest 
import rostest

sys.path.insert(0, os.path.abspath('..'))
from scripts.utils import (
    detect_SU_and_DoF_nums,
    collect_static_accelerometer_values,
    average_accelerometer_values,
    create_activity_matrix,
    merge_activity_matrix,
    sort_merged_activity_matrix
)

class MainTestCase(unittest.TestCase):
    def test_(self):
        pass

    def test_detect_SU_and_DoF_nums(self):
        pass

    def test_collect_static_accelerometer_values(self):
        pass

    def test_average_accelerometer_values(self):
        pass

    def test_create_activity_matrix(self):
        pass

    def test_merge_activity_matrix(self):
        pass

    def test_sort_merged_activity_matrix(self):
        pass

if __name__ == '__main__':
    rostest.rosrun('ros_robotic_skin', 'test_main', MainTestCase)