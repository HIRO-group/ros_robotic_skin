#! /usr/bin/env python

import unittest
import rospy


class TestPanda(unittest.TestCase):
    def test_param_loaded(self):
        dynamic_frequency = rospy.get_param('dynamic_frequency')
        self.assertGreater(dynamic_frequency, 0)

        gravity_constant = rospy.get_param('gravity_constant')
        self.assertEqual(gravity_constant, 9.81)

        oscillation_magnitude = rospy.get_param('oscillation_magnitude')
        oscillation_frequency = rospy.get_param('oscillation_frequency')
        self.assertTrue(isinstance(oscillation_magnitude, list))
        self.assertTrue(isinstance(oscillation_frequency, list))
        self.assertTrue(len(oscillation_magnitude) == len(oscillation_frequency))

        oscillation_time = rospy.get_param('oscillation_time')
        self.assertGreater(oscillation_time, 0)

        is_sim = rospy.get_param('is_sim')
        self.assertFalse(is_sim)

        rest_time = rospy.get_param('rest_time')
        self.assertGreater(rest_time, 0)

    def test_controller(self):
        panda_joint_position_controller = rospy.get_param('panda_joint_position_controller')
        keys = ['type', 'arm_id', 'joint_names', 'joint_velocities']
        for key in keys:
            self.assertTrue(key in panda_joint_position_controller.keys())

        panda_joint_velocity_controller = rospy.get_param('panda_joint_velocity_controller')
        keys = ['type', 'arm_id', 'joint_names']
        for key in keys:
            self.assertTrue(key in panda_joint_velocity_controller.keys())

    def test_publish_topics(self):
        # TODO: Add some important topics
        pass


if __name__ == '__main__':
    import rostest
    rostest.rosrun('ros_robotic_skin', 'test_panda', TestPanda)
