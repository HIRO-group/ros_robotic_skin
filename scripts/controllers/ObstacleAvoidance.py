#!/usr/bin/env python

from PandaController import PandaController
import numpy as np
import rospy
import tf


class ObstacleAvoidance(object):
    def __init__(self):

        self.panda_controller = PandaController()
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(rospy.get_param('/cartesian_controller_frequency'))

        self.control_points = []

    def __is_array_in_list(self, element, list_of_vectors):
        for vector in list_of_vectors:
            if np.array_equal(element, vector):
                return True
        return False

    def get_control_points(self):
        self.control_points = []
        for i in range(2, 8):
            while True:
                try:
                    (trans0, rot0) = self.tf_listener.lookupTransform('world', 'panda_link{}'.format(i), rospy.Time())
                    (trans1, rot1) = self.tf_listener.lookupTransform('world', 'panda_link{}'.format(i+1), rospy.Time())
                    print('panda_link{}'.format(i) + ' and ' + 'panda_link{}'.format(i+1))
                    break
                except(tf.LookupException,
                       tf.ConnectivityException,
                       tf.ExtrapolationException):
                    continue

            point0 = np.array(trans0)
            point1 = np.array(trans1)
            point_middle = point0 + 0.5 * (point1-point0)  # This index can be used to get other points than the middle point
            new_points = [point0, point1, point_middle]
            for point in new_points:
                if not self.__is_array_in_list(point, self.control_points):
                    self.control_points.append(point)

    def get_obstacle_points(self):
        self.obstacle_points = [np.array([0., 0.,  0.333]),
                                np.array([-0.19582668, -0.24056836,  0.2727109]),
                                np.array([-0.09791334, -0.12028418,  0.30285545]),
                                np.array([-0.13986038, -0.27334841,  0.22172569]),
                                np.array([-0.16784353, -0.25695838,  0.24721829]),
                                np.array([0.21203928, -0.26132252,  0.04770196]),
                                np.array([0.03608945, -0.26733546,  0.13471382]),
                                np.array([0.27500713, -0.24394749,  0.1066694]),
                                np.array([0.2435232, -0.252635,  0.07718568]),
                                np.array([0.20461844, -0.25810337,  0.18600441]),
                                np.array([0.23981278, -0.25102543,  0.1463369])]


if __name__ == "__main__":
    obstacle_avoidance = ObstacleAvoidance()
    obstacle_avoidance.get_control_points()
    obstacle_avoidance.get_obstacle_points()
    print('-----------------------------')
    print(obstacle_avoidance.control_points)
    print(len(obstacle_avoidance.control_points))
    print('-----------------------------')
    print(obstacle_avoidance.obstacle_points)
