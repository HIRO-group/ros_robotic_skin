#!/usr/bin/env python

from PandaController import PandaController
from CartesianController import CartesianController
import numpy as np
import rospy
import tf


class ObstacleAvoidance(CartesianController):
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
        self.obstacle_points = [np.array([0., 0.,  0.333])]

    def go_to_point(self, position_desired):
        """
        Move to a point an stop when arrived

        Parameters
        ----------
        position_desired : numpy.ndarray
            3x1 end effector desired position vector
            Only position, not orientation
        """
        position_desired = np.array(position_desired)
        self.error = position_desired - self.position
        while np.linalg.norm(self.error) > self.error_threshold:
            self.position = self.__get_current_end_effector_position()
            xd_dot = self.__compute_command_velocity(position_desired)
            print(xd_dot)
            # Add flacco algorithm for end effector
            # xc_dot = end_effector_algorithm(xd_dot)
            # self.q_dot = self.__compute_command_q_dot(xc_dot)
            self.panda_controller.send_velocities(self.q_dot)
            self.rate.sleep()
        self.stop()
        return 0


if __name__ == "__main__":
    obstacle_avoidance = ObstacleAvoidance()
    obstacle_avoidance.get_control_points()
    obstacle_avoidance.get_obstacle_points()
    print('-----------------------------')
    print(obstacle_avoidance.control_points)
    print(len(obstacle_avoidance.control_points))
    print('-----------------------------')
    print(obstacle_avoidance.obstacle_points)
