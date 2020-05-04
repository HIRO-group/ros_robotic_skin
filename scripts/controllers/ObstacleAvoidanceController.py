#!/usr/bin/env python

from CartesianPositionController import CartesianPositionController
import numpy as np
import rospy
import tf
from ros_robotic_skin.msg import IdxPoint

NUMBER_OF_CONTROL_POINTS = 1

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
        self.obstacle_points = [np.array([0, 0, 0])]  # Very far away
        self.obstacle_id = [-1]
        self.Vi = np.zeros(3)

        rospy.Subscriber("obstacle_points", IdxPoint, self.callback_live_points)
    
    def callback_live_points(self, data):
        id = data.idx
        if id in self.obstacle_id:
            idx = self.obstacle_id.index(id)
            self.obstacle_id.pop(idx)
            self.obstacle_points.pop(idx)
            
        self.obstacle_id.append(id)
        self.obstacle_points.append(np.array([data.point.x, data.point.y, data.point.z]))
        #print(len(self.obstacle_points))

    def get_control_points(self):
        """
        Updates the location of the end_effector in self.position.
        Updates the location of the control points in self.control_points.
        """
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

    def get_distance_vectors_end_effector(self):
        """
        Get distance vectors that go from obstacle points to the end effector

        Returns
        -------
        d_vectors: list
            List of np.ndarray s, as many as obstacle points there are
        """
        d_vectors = []
        number_of_obstacle_points = len(self.obstacle_points)
        for j in range(number_of_obstacle_points):
            d_vectors = d_vectors + [self.obstacle_points[j] - self.position]
        return d_vectors

    def get_distance_vectors_body(self):
        """
        Get distance vectors that go from obstacle points to control points

        Returns
        -------
        D_vectors: list
            List of lists that contain np.ndarrays, as many as obstacle points there are
        """
        D_vectors = []
        number_of_control_points = len(self.control_points)
        number_of_obstacle_points = len(self.obstacle_points)
        for i in range(number_of_control_points):
            d_vectors_i = []
            for j in range(number_of_obstacle_points):
                d_vectors_i = d_vectors_i + [self.obstacle_points[j] - self.control_points[i]]
            D_vectors = D_vectors + [d_vectors_i]
        return D_vectors

    def get_repulsive_distance(self):
        """
        Repulsive vector algorithm (Only distance information)

        Returns
        -------
        max_repulse_vector: np.ndarray
            Repulsive vector that the algorithm computes having into account obstacle data
        """
        repulse_vector = []
        distance_vectors = self.get_distance_vectors_end_effector()
        # D = search_smallest_vector(distance_vectors)
        # D_norm = np.linalg.norm(D)
        # D_unit_vec = D/D_norm

        # v_mag_repulse = V_MAX * (1 / (1 + np.exp((D_norm * (2 / RHO) - 1) * ALPHA)))
        # v_repulse = v_mag_repulse * D_unit_vec
        # return v_repulse

        for D in distance_vectors:

            D_norm = np.linalg.norm(D)
            D_unit_vec = D/D_norm

            v_mag_repulse = V_MAX * (1 / (1 + np.exp((D_norm * (2 / RHO) - 1) * ALPHA)))
            v_repulse = v_mag_repulse * D_unit_vec

            repulse_vector.append(v_repulse)

        max_repulse_vector = self.search_largest_vector(repulse_vector)

        return max_repulse_vector

    def get_repulsive_distance_velocity(self):
        """
        Repulsive vector algorithm (distance and rate of change information)

        Returns
        -------
        max_repulse_vector: np.ndarray
            Repulsive vector that the algorithm computes having into account obstacle data.
            The angle of the vector is changed by this algorithm
        """

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

        if beta > np.pi / 2:
            return self.Vi  # The obstacle is moving away from us. Don't use velocity information
        else:
            n = np.cross(a, r)
            v = np.cross(n, a)
            new_angle = beta - (np.pi/2 - beta) * np.exp(-(C * (2 / MAX_RATE * Vi_dot_magnitude - 1)))
            V_new = np.linalg.norm(self.Vi) * (np.cos(new_angle) * a + np.sin(new_angle) * v)
            return V_new

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
        i = norms.index(min(norms))
        return vectors_list[i]

    def search_largest_vector(self, vectors_list):
        """
        Algorithm that takes a list of vectors and returns the largest

        Parameters
        ----------
        vectors_list : list
            List of np.ndarrays

        Returns
        -------
        vectors_list[i]: np.ndarray
            The largest vector in vectors_list
        """
        norms = [np.linalg.norm(v) for v in vectors_list]
        i = norms.index(max(norms))
        return vectors_list[i]

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
        # Uncomment one of the following options to decide what algorithm you want to use
        repulsive_vector = np.block([self.get_repulsive_distance(), np.array([0, 0, 0])])
        # repulsive_vector = np.block([self.get_repulsive_distance_velocity(), np.array([0, 0, 0])])

        repulsive_vector.shape = (6, 1)

        xc_dot = xd_dot - repulsive_vector

        return xc_dot

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

        (q_dot_min, q_dot_max) = self.select_most_restrictive(q_dot_max_list, q_dot_min_list)

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
        for i in range(7):
            if self.q_dot[i] > q_dot_max[i]:
                self.q_dot[i] = q_dot_max[i]
            elif self.q_dot[i] < q_dot_min[i]:
                self.q_dot[i] = q_dot_min[i]

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
            # self.get_obstacle_points()

            # Obtained the desired end effector velocity, in the case there were no obstacles
            xd_dot = self.compute_command_velocity(position_desired)

            # Modify the velocity with the obstacles information
            # Add flacco algorithm for end effector to get cartesian velocity xc_dot
            xc_dot = self.end_effector_algorithm(xd_dot)
            # xc_dot = xd_dot

            # Compute the corresponding joint velocities q_dot
            self.q_dot = self.compute_command_q_dot(xc_dot)
            # Compute the joint velocity restrictions and apply them
            # (q_dot_min, q_dot_max) = self.body_algorithm()
            # self.apply_restrictions(q_dot_min, q_dot_max)
            # Publish velocities
            self.panda_controller.send_velocities(self.q_dot)
            self.rate.sleep()
        self.stop()
        return 0

    def get_obstacle_points(self):
        """
        In the future this will compute obstacle points.
        Right now it's just a list of points to test the algorithm.
        """
        self.obstacle_points = np.array([[-0.1, 0.00013597, 0.47066]])


if __name__ == "__main__":

    obstacle_avoidance_controller = ObstacleAvoidanceController()
    trajectory = np.array([[0.5, 0, 0.8], [0.8, 0, 0.8]])
    while not rospy.is_shutdown():
        obstacle_avoidance_controller.go_to_point(trajectory[0])
        obstacle_avoidance_controller.go_to_point(trajectory[1])
