#!/usr/bin/env python

from CartesianController import CartesianController
import numpy as np
import rospy
import tf


NUMBER_OF_CONTROL_POINTS = 8
ALPHA = 6
RHO = 0.4
Q_DOT_MIN = np.array([-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100])
Q_DOT_MAX = np.array([+2.1750, +2.1750, +2.1750, +2.1750, +2.6100, +2.6100, +2.6100])


class ObstacleAvoidance(CartesianController):
    def __init__(self):

        super(ObstacleAvoidance, self).__init__()
        self.position = np.zeros(3)
        self.control_points = []

    def get_control_points(self):
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('world', 'end_effector', rospy.Time(0))
                self.position = np.array(trans)
                for i in range(NUMBER_OF_CONTROL_POINTS):
                    (trans, rot) = self.tf_listener.lookupTransform('world', 'control_point{}'.format(i), rospy.Time(0))
                    self.control_points.append(np.array(trans))
                break

            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue

    def get_distance_vectors_body(self):
        D_vectors = []
        number_of_control_points = len(self.control_points)
        number_of_obstacle_points = len(self.obstacle_points)
        for i in range(number_of_control_points):
            d_vectors_i = []
            for j in range(number_of_obstacle_points):
                d_vectors_i = d_vectors_i + [self.obstacle_points[j] - self.control_points[i]]
            D_vectors = D_vectors + [d_vectors_i]
        return D_vectors

    def search_smallest_vector(self, vectors_list):
        norms = [np.linalg.norm(v) for v in vectors_list]
        i = norms.index(min(norms))
        return vectors_list[i]

    def select_most_restrictive(self, q_dot_max_list, q_dot_min_list):
        q_dot_max = Q_DOT_MAX
        q_dot_min = Q_DOT_MIN
        for i in range(7):
            max_values = [vector[i] for vector in q_dot_max_list]
            min_values = [vector[i] for vector in q_dot_min_list]
            q_dot_max[i] = min(max_values)
            q_dot_min[i] = max(min_values)
        return (q_dot_min, q_dot_max)

    def end_effector_algorithm(self, xd):
        np.array(xd)
        V_max = 1  # m/s
        alpha = 6  # shape vector
        rho = 0.4  # m

        D = xd[0:2]
        D_norm = np.linalg.norm(D)
        D_unit_vec = D/D_norm

        v_repulse = (V_max / (1 + np.exp((D_norm*(rho/2)-1)*alpha)))*D_unit_vec

        xd[0:2] = D + v_repulse

        xc = xd

        return xc

    def body_algorithm(self):
        D = self.get_distance_vectors_body()

        q_dot_max_list = []
        q_dot_min_list = []

        for (i, distances_control_point_i) in enumerate(D):
            smallest_distance = self.search_smallest_vector(distances_control_point_i)
            distance_norm = np.linalg.norm(smallest_distance)
            unit_distance = smallest_distance / distance_norm
            # Risk function
            f = 1 / (1 + np.exp(distance_norm * 2 / RHO - 1) * ALPHA)
            # Get partial jacobian i
            Jacobian_message = self.get_jacobian(self.q, 'control_point{}'.format(i))
            J = np.array(Jacobian_message.J.J)
            J.shape = (Jacobian_message.J.rows, Jacobian_message.J.columns)
            # Risk vector projected in joint space
            s = np.dot(np.linalg.pinv(J)[:, :3], unit_distance) * f
            q_dot_max_i = Q_DOT_MAX
            q_dot_min_i = Q_DOT_MIN
            for i in range(len(s)):
                if s[i] >= 0:
                    q_dot_max_i[i] = Q_DOT_MAX[i] * (1 - f)
                else:
                    q_dot_min_i[i] = Q_DOT_MIN[i] * (1 - f)
            q_dot_max_list = q_dot_max_list + [q_dot_max_i]
            q_dot_min_list = q_dot_min_list + [q_dot_min_i]

        (q_dot_min, q_dot_max) = self.select_most_restrictive(q_dot_max_list, q_dot_min_list)
        return (q_dot_min, q_dot_max)

    def apply_restrictions(self, q_dot_min, q_dot_max):
        for i in range(7):
            if self.q_dot[i] > q_dot_max[i]:
                self.q_dot[i] = q_dot_max[i]
            elif self.q_dot[i] < q_dot_min[i]:
                self.q_dot[i] = q_dot_min[i]

    def is_array_in_list(self, element, list_of_vectors):
        for vector in list_of_vectors:
            if np.array_equal(element, vector):
                return True
        return False

    def get_obstacle_points(self):
        self.obstacle_points = np.array([0.65, 0, 0.3])

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
            self.get_control_points()
            self.get_obstacle_points()
            xd_dot = self.compute_command_velocity(position_desired)
            # Add flacco algorithm for end effector to get cartesian velocity xc_dot
            # TODO: Calc |D(P,P)|
            # TODO: Calc v(P,O)
            # TODO: Return xc_dot
            # xc_dot = self.end_effector_algorithm(xd_dot)
            xc_dot = xd_dot
            # Compute the corresponding joint velocities q_dot
            self.q_dot = self.compute_command_q_dot(xc_dot)
            # Compute the joint velocity restrictions and apply them
            (q_dot_min, q_dot_max) = self.body_algorithm()
            self.apply_restrictions(q_dot_min, q_dot_max)
            # print("min", q_dot_min)
            # print("max", q_dot_max)
            # Publish velocities
            self.panda_controller.send_velocities(self.q_dot)
            self.rate.sleep()
        self.stop()
        return 0


if __name__ == "__main__":
    controller = ObstacleAvoidance()
    while not rospy.is_shutdown():
        controller.go_to_point(np.array([0.4, 0, 0.3]))
        controller.go_to_point(np.array([0.65, 0, 0.3]))
