#!/usr/bin/env python

from CartesianController import CartesianController
import numpy as np
import rospy
import tf
# import matplotlib.pyplot as plt

MAX_RATE = 1.0
NUMBER_OF_CONTROL_POINTS = 1
ALPHA = 6.0
C = 5.0
RHO = 0.4
V_MAX = 0.21
Q_DOT_MIN = np.array([-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100])
Q_DOT_MAX = np.array([+2.1750, +2.1750, +2.1750, +2.1750, +2.6100, +2.6100, +2.6100])


class ObstacleAvoidance(CartesianController):
    def __init__(self):

        super(ObstacleAvoidance, self).__init__()
        self.position = np.zeros(3)
        self.control_points = []
        self.Vi = np.zeros(3)
        # self.Vi_last = np.zeros(3)

    def get_control_points(self):
        # t = time.time()
        self.control_points = []
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
        # print(time.time() - t)

    def get_distance_vectors_end_effector(self):
        d_vectors_i = []
        number_of_obstacle_points = len(self.obstacle_points)

        for j in range(number_of_obstacle_points):
            d_vectors_i = d_vectors_i + [self.obstacle_points[j] - self.position]

        return d_vectors_i

    def get_distance_vectors_body(self):
        # t = time.time()
        D_vectors = []
        number_of_control_points = len(self.control_points)
        number_of_obstacle_points = len(self.obstacle_points)
        for i in range(number_of_control_points):
            d_vectors_i = []
            for j in range(number_of_obstacle_points):
                d_vectors_i = d_vectors_i + [self.obstacle_points[j] - self.control_points[i]]
            D_vectors = D_vectors + [d_vectors_i]

        # print(time.time() - t)
        return D_vectors

    def get_repulsive_distance(self):
        repulse_vector = []
        distance_vectors = self.get_distance_vectors_end_effector()

        for D in distance_vectors:

            D_norm = np.linalg.norm(D)
            D_unit_vec = D/D_norm

            # print("D_norm: {}".format(D_norm))

            v_mag_repulse = V_MAX * (1 / (1 + np.exp((D_norm * (2 / RHO) - 1) * ALPHA)))
            v_repulse = v_mag_repulse * D_unit_vec

            # print("Repulsive vector norm: {}".format(v_mag_repulse))
            # print("-------------------------------")

            repulse_vector.append(v_repulse)

        max_repulse_vector = self.search_largest_vector(repulse_vector)

        return max_repulse_vector

    def get_repulsive_distance_velocity(self):

        self.Vi = self.get_repulsive_distance()
        try:
            self.Vi_last
        except AttributeError:
            self.Vi_last = self.Vi

        Vi_dot = self.Vi - self.Vi_last
        self.Vi_last = self.Vi

        Vi_dot_magnitude = np.linalg.norm(Vi_dot)

        np.divide(Vi_dot, Vi_dot_magnitude)

        if Vi_dot_magnitude != 0:
            a = Vi_dot / Vi_dot_magnitude
        else:
            return self.Vi

        r = self.Vi / np.linalg.norm(self.Vi)
        beta = np.arccos(np.dot(a, r))
        print(beta)

        if beta > np.pi / 2:
            # The obstacle is moving away from us.
            # Don't use velocity information
            return self.Vi
        else:
            n = np.cross(a, r)
            v = np.cross(n, a)
            new_angle = beta - (np.pi/2 - beta) * np.exp(-(C * (2 / MAX_RATE * Vi_dot_magnitude - 1)))
            V_new = np.linalg.norm(self.Vi) * (np.cos(new_angle) * a + np.sin(new_angle) * v)

            print(new_angle)
            print('------------------------------------')

            return V_new

    def search_smallest_vector(self, vectors_list):
        norms = [np.linalg.norm(v) for v in vectors_list]
        i = norms.index(min(norms))
        return vectors_list[i]

    def search_largest_vector(self, vectors_list):
        norms = [np.linalg.norm(v) for v in vectors_list]
        i = norms.index(max(norms))
        return vectors_list[i]

    def select_most_restrictive(self, q_dot_max_list, q_dot_min_list):
        q_dot_max = np.array(Q_DOT_MAX)
        q_dot_min = np.array(Q_DOT_MIN)
        for i in range(7):
            max_values = [vector[i] for vector in q_dot_max_list]
            min_values = [vector[i] for vector in q_dot_min_list]
            q_dot_max[i] = min(max_values)
            q_dot_min[i] = max(min_values)
        return (q_dot_min, q_dot_max)

    def end_effector_algorithm(self, xd_dot):
        repulsive_vector = np.block([self.get_repulsive_distance(), np.array([0, 0, 0])])
        # repulsive_vector = np.block([self.get_repulsive_distance_velocity(), np.array([0, 0, 0])])
        repulsive_vector.shape = (6, 1)
        xc_dot = xd_dot - repulsive_vector

        return xc_dot

    def body_algorithm(self):
        # t = time.time()
        D = self.get_distance_vectors_body()

        q_dot_max_list = []
        q_dot_min_list = []

        for (i, distances_control_point_i) in enumerate(D):
            smallest_distance = self.search_smallest_vector(distances_control_point_i)
            distance_norm = np.linalg.norm(smallest_distance)
            unit_distance = smallest_distance / distance_norm
            # Risk function
            f = 1 / (1 + np.exp((distance_norm * 2 / RHO - 1) * ALPHA))
            # Get partial jacobian i
            Jacobian_message = self.get_jacobian(self.q, 'control_point{}'.format(i))
            J = np.array(Jacobian_message.J.J)
            J.shape = (Jacobian_message.J.rows, Jacobian_message.J.columns)
            # Risk vector projected in joint space
            s = np.dot(np.linalg.pinv(J)[:, :3], unit_distance) * f
            q_dot_max_i = np.array(Q_DOT_MAX)
            q_dot_min_i = np.array(Q_DOT_MIN)
            for i in range(len(s)):
                if s[i] >= 0:
                    q_dot_max_i[i] = Q_DOT_MAX[i] * (1 - f)
                else:
                    q_dot_min_i[i] = Q_DOT_MIN[i] * (1 - f)
            q_dot_max_list = q_dot_max_list + [q_dot_max_i]
            q_dot_min_list = q_dot_min_list + [q_dot_min_i]

            print("distance: {}".format(distance_norm))
            print("f: {}".format(f))

        (q_dot_min, q_dot_max) = self.select_most_restrictive(q_dot_max_list, q_dot_min_list)
        # print(time.time() - t)

        # print('q_dot_min{}'.format(q_dot_min))
        # print('q_dot_max{}'.format(q_dot_max))
        # print('------------------------')

        return (q_dot_min, q_dot_max)

    def apply_restrictions(self, q_dot_min, q_dot_max):
        print('Before: {}'.format(np.array(self.q_dot)))
        for i in range(7):
            if self.q_dot[i] > q_dot_max[i]:
                self.q_dot[i] = q_dot_max[i]
            elif self.q_dot[i] < q_dot_min[i]:
                self.q_dot[i] = q_dot_min[i]
        print('After: {}'.format(np.array(self.q_dot)))
        print("---------------")

    def is_array_in_list(self, element, list_of_vectors):
        for vector in list_of_vectors:
            if np.array_equal(element, vector):
                return True
        return False

    def get_obstacle_points(self):

        # self.obstacle_points = [np.array([0.65, 0, 0.3])]
        self.obstacle_points = [np.array([-0.2, 0.00013597, 0.47066])]

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
            # print("xd_dot norm: {}".format(np.linalg.norm(xd_dot)))
            # Add flacco algorithm for end effector to get cartesian velocity xc_dot
            # xc_dot = self.end_effector_algorithm(xd_dot)
            xc_dot = xd_dot
            # Compute the corresponding joint velocities q_dot
            self.q_dot = self.compute_command_q_dot(xc_dot)
            # Compute the joint velocity restrictions and apply them
            (q_dot_min, q_dot_max) = self.body_algorithm()
            self.apply_restrictions(q_dot_min, q_dot_max)
            # Publish velocities
            self.panda_controller.send_velocities(self.q_dot)
            self.rate.sleep()
        self.stop()
        return 0


if __name__ == "__main__":
    # controller = ObstacleAvoidance()
    # controller.get_obstacle_points()
    # controller.get_control_points()
    # print(controller.get_distance_vectors_end_effector())
    # print(controller.get_repulsive_distance())
    # controller.V_i = np.array([0, 0, 0.1])
    # print(controller.get_repulsive_distance_velocity())

    # distance_norm = np.arange(0.0, 0.5, 0.01)
    # f = 1 / (1 + np.exp((distance_norm * 2 / RHO - 1) * ALPHA))
    # plt.plot(distance_norm, f)
    # plt.show()

    cartesian_controller = ObstacleAvoidance()
    trajectory = np.array([[0.4, 0, 0.3], [0.65, 0, 0.3]])
    while not rospy.is_shutdown():
        cartesian_controller.go_to_point(trajectory[0])
        cartesian_controller.go_to_point(trajectory[1])
