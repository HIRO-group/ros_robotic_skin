#!/usr/bin/env python

from CartesianPositionController import CartesianPositionController
import numpy as np
import rospy
import tf
from ros_robotic_skin.msg import PointArray

NUMBER_OF_CONTROL_POINTS = 7

# Parameters of the flacco paper
MAX_RATE = 1.0  # Obstacle velocity estimation equation
ALPHA = 6.0
C = 5.0
RHO = 0.4
V_MAX = 0.21
Q_DOT_MIN = np.array([-2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100])
Q_DOT_MAX = np.array([+2.1750, +2.1750, +2.1750, +2.1750, +2.6100, +2.6100, +2.6100])


class ObstacleAvoidanceController(CartesianPositionController):
    def __init__(self):
        super(ObstacleAvoidanceController, self).__init__()
        self.control_points = []
        self.obstacle_points = [np.array([5.00, 5.00, 5.00])]
        self.Vi = np.zeros(3)
        self.sphere_radiuses = (0.23, 0.24, 0.2, 0.237, 0.225, 0.20, 0.27)
        rospy.Subscriber("live_points", PointArray, self.CallbackLivePoints)

    def CallbackLivePoints(self, msg):
        self.obstacle_points = []
        for i in range(len(msg.points)):
            self.obstacle_points.append(np.array([msg.points[i].x, msg.points[i].y, msg.points[i].z]))
        # self.obstacle_points = np.array([[0.8, 0, 0.3]])

    def get_control_points(self):
        """
        Updates the location of the end_effector in self.position.
        Updates the location of the control points in self.control_points.
        """
        self.control_points = []
        while not rospy.is_shutdown():
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

    def get_distance_vectors_end_effector(self):
        """
        Get distance vectors that go from obstacle points to the end effector

        Returns
        -------
        d_vectors: list
            List of np.ndarray s, as many as obstacle points there are
        """
        return [(self.obstacle_points[j] - self.position) for j in range(len(self.obstacle_points))]

    def get_distance_vectors_body(self):
        """
        Get distance vectors that go from obstacle points to control points

        Returns
        -------
        D_vectors: list
            List of lists that contain np.ndarrays, as many as obstacle points there are
        """
        D_matrix = []
        for i in range(NUMBER_OF_CONTROL_POINTS):
            D_matrix.append([(self.obstacle_points[j] - self.control_points[i]) for j in range(len(self.obstacle_points))])
        return D_matrix

    def get_repulsive_vector(self):
        """
        Repulsive vector algorithm (Only distance information)

        Returns
        -------
        max_repulse_vector: np.ndarray
            Repulsive vector that the algorithm computes having into account obstacle data
        """
        Ds = self.get_distance_vectors_end_effector()
        if Ds:
            D = self.search_smallest_vector(Ds)
            magnitude = V_MAX * (1 / (1 + np.exp(((np.linalg.norm(D)) * (2 / RHO) - 1) * ALPHA)))
            unitary_vector = D/np.linalg.norm(D)
            return magnitude * unitary_vector
        else:
            return np.zeros(3)

    def search_smallest_vector(self, vectors_list):
        """
        Algorithm that takes a list of vectors and returns the smallest

        Parameters
        ----------
        vectors_list : list
            List of np.ndarrays

        Returns
        -------
        vectors_list[i]: np.ndarray
            The smallest vector in vectors_list
        """
        norms = [np.linalg.norm(v) for v in vectors_list]
        return vectors_list[norms.index(min(norms))]

    def select_most_restrictive(self, q_dot_max_list, q_dot_min_list):
        """
        This function gets a list of restriction vectors and returns the most restrictive combination

        Parameters
        ----------
        q_dot_max_list : list
            list of restriction vectors
        q_dot_min_list : list
            list of restriction vectors

        Returns
        -------
        q_dot_min: np.ndarray
            restriction vector
        q_dot_max: np.ndarray
            restriction vectors
        """

        q_dot_max = np.array(Q_DOT_MAX)
        q_dot_min = np.array(Q_DOT_MIN)
        for i in range(7):
            max_values = [vector[i] for vector in q_dot_max_list]
            min_values = [vector[i] for vector in q_dot_min_list]
            q_dot_max[i] = min(max_values)
            q_dot_min[i] = max(min_values)
        return (q_dot_min, q_dot_max)

    def apply_restrictions(self, q_dot_min, q_dot_max):
        """
        Applies the restrictions contained in q_dot_min and q_dot_max to self.q_dot member of the controller

        Parameters
        ----------
        q_dot_min : [type]
            [description]
        q_dot_max : [type]
            [description]
        """
        # print('Before: {}'.format(np.array(self.q_dot)))
        for i in range(7):
            if self.q_dot[i] > q_dot_max[i]:
                self.q_dot[i] = q_dot_max[i]
            elif self.q_dot[i] < q_dot_min[i]:
                self.q_dot[i] = q_dot_min[i]
        # print('After: {}'.format(np.array(self.q_dot)))
        # print("---------------")

    def end_effector_algorithm(self, xd_dot):
        """
        Algorithm that modifies the end effector velocity with the repulsive vectors, that depend on the obstacles sensed

        Parameters
        ----------
        xd_dot : np.ndarray
            6x1 desired end effector velocity vector

        Returns
        -------
        xc_dot : np.ndarray
            6x1 modified end effector velocity vector (Modified with obstacle information)
        """
        repulsive_vector = np.block([self.get_repulsive_vector(), np.array([0, 0, 0])])
        repulsive_vector.shape = (6, 1)

        return xd_dot - repulsive_vector

    def body_algorithm(self):
        """
        Algorithm that computes the restrictions to joint velocities depending on obstacles data.

        Returns
        -------
        q_dot_min: np.ndarray
            restriction vector
        q_dot_max: np.ndarray
            restriction vectors
        """

        q_dot_max_list = []
        q_dot_min_list = []

        for (i, distances_control_point_i) in enumerate(self.get_distance_vectors_body()):
            if distances_control_point_i:
                smallest_distance = self.search_smallest_vector(distances_control_point_i)
            else:
                return (np.array(Q_DOT_MIN), np.array(Q_DOT_MAX))
            distance_norm = np.linalg.norm(smallest_distance) - self.sphere_radiuses[i]
            unitary_vector = smallest_distance / distance_norm
            f = 1 / (1 + np.exp((distance_norm * 2 / RHO - 1) * ALPHA))

            Jacobian_message = self.get_jacobian(self.q, 'control_point{}'.format(i))
            J = np.array(Jacobian_message.J.J)
            J.shape = (Jacobian_message.J.rows, Jacobian_message.J.columns)

            s = np.dot(np.linalg.pinv(J)[:, :3], unitary_vector) * f
            q_dot_max_i = np.array(Q_DOT_MAX)
            q_dot_min_i = np.array(Q_DOT_MIN)
            for i in range(len(s)):
                if s[i] >= 0:
                    q_dot_max_i[i] = Q_DOT_MAX[i] * (1 - f)
                else:
                    q_dot_min_i[i] = Q_DOT_MIN[i] * (1 - f)
            q_dot_max_list = q_dot_max_list + [q_dot_max_i]
            q_dot_min_list = q_dot_min_list + [q_dot_min_i]

        (q_dot_min, q_dot_max) = self.select_most_restrictive(q_dot_max_list, q_dot_min_list)

        return (q_dot_min, q_dot_max)

    def go_to_point(self, position_desired):
        """
        Main function. Move to a point an stop when arrived

        Parameters
        ----------
        position_desired : numpy.ndarray
            3x1 end effector desired position vector
            Only position, not orientation
        """
        position_desired = np.array(position_desired)
        self.error = position_desired - self.position
        while np.linalg.norm(self.error) > self.error_threshold:
            # Update lists
            self.get_control_points()
            # Obtained the desired end effector velocity, in the case there were no obstacles
            xd_dot = self.compute_command_velocity(position_desired)

            # Modify the velocity with the obstacles information
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

    obstacle_avoidance_controller = ObstacleAvoidanceController()
    x0 = 0.6
    y0 = 0.0
    z0 = 0.3
    r = 0.2
    resolution = 0.4
    trajectory = obstacle_avoidance_controller.get_trajectory_points_in_circle_yz_plane(r, x0, y0, z0, resolution)
    while not rospy.is_shutdown():
        for point in trajectory:
            if not rospy.is_shutdown():
                obstacle_avoidance_controller.go_to_point(point)
