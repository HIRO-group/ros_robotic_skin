#! /usr/bin/env python

import unittest
import rospy


class TestSimulation(unittest.TestCase):
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
        self.assertTrue(is_sim)

        rest_time = rospy.get_param('rest_time')
        self.assertGreater(rest_time, 0)

    def test_controller(self):
        joint_state_controller = rospy.get_param('joint_state_controller')
        self.assertTrue('type' in joint_state_controller.keys())
        self.assertEqual(joint_state_controller['publish_rate'], 100)

        panda_joint_trajectory_controller = rospy.get_param('panda_joint_trajectory_controller')
        keys = ['type', 'joints', 'gains', 'state_publish_rate', 'constraints']
        for key in keys:
            self.assertTrue(key in panda_joint_trajectory_controller.keys())

        panda_hand_controller = rospy.get_param('panda_hand_controller')
        keys = ['type', 'joints', 'gains', 'state_publish_rate']
        for key in keys:
            self.assertTrue(key in panda_hand_controller.keys())

        n_joint = 7
        for i_joint in range(1, n_joint+1):
            ith_position_controller = rospy.get_param('panda_joint%i_position_controller' % (i_joint))
            ith_velocity_controller = rospy.get_param('panda_joint%i_velocity_controller' % (i_joint))
            self.assertTrue('type' in ith_position_controller.keys())
            self.assertTrue('joint' in ith_position_controller.keys())
            self.assertTrue('type' in ith_velocity_controller.keys())
            self.assertTrue('joint' in ith_velocity_controller.keys())

    def test_publish_topics(self):
        # TODO: Add some important topics
        pass


if __name__ == '__main__':
    import rostest
    rostest.rosrun('ros_robotic_skin', 'test_simulation', TestSimulation)
