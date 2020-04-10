#!/usr/bin/env python

from PandaController import PandaController
import numpy as np
import rospy
import tf

from ros_robotic_skin.srv import getJacobian
from sensor_msgs.msg import JointState


class CartesianController(object):
    """
    Cartesian controller built on top of PandaController. clean code
    """

    # TODO: Fix: When trajectory points are to close to each other, the movement is intermittent, because self.stop() is called too much.
    # TODO: Avoid self collision

    def callback_joint_states(self, data):
        self.q = data.position

    def __init__(self):
        self.panda_controller = PandaController()
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(rospy.get_param('/cartesian_controller_frequency'))
        self.q = [0 for i in range(9)]
        self.q_dot = np.zeros(7)
        self.p_gain = rospy.get_param('/cartesian_controller_p_gain')
        self.error_threshold = rospy.get_param('/cartesian_error_threshold')
        rospy.on_shutdown(self.return_home)

        self.position = self.get_current_end_effector_position()

        rospy.wait_for_service('get_jacobian')
        self.get_jacobian = rospy.ServiceProxy('get_jacobian', getJacobian)
        rospy.Subscriber("joint_states", JointState, self.callback_joint_states)

    def get_current_end_effector_position(self):
        """
        Get the current end effector position in cartesian coordinates

        Returns
        -------
        trans : numpy.ndarray
            3x1 vector containing current end effector position vector
        """
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('world', 'end_effector', rospy.Time(0))
                return np.array(trans)
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue

    def compute_command_velocity(self, position_desired):
        """
        Compute the velocity vector in cartesian coordinates that leads to the Point position_desired at a given time instant.

        Parameters
        ----------
        position_desired : numpy.ndarray
            3x1 end effector desired position vector

        Returns
        -------
        v: numpy.ndarray
            6x1 velocity vector in cartesian coordinates

        """
        self.error = position_desired - self.position

        v_trans = self.p_gain * self.error / np.linalg.norm(self.error)
        v_rot = np.array([0, 0, 0])
        v = np.block([v_trans, v_rot])
        v.shape = (6, 1)
        return v

    def compute_command_q_dot(self, v):
        """
        Convert cartesian velocity vector to joint space through Jacobian

        Parameters
        ----------
        v : numpy.ndarray
             6x1 velocity vector in cartesian coordinates

        Returns
        -------
        q_dot : numpy.ndarray
            7x1 joint velocity vector
        """
        Jacobian_message = self.get_jacobian(self.q, 'panda_link8')
        J = np.array(Jacobian_message.J.J)
        J.shape = (Jacobian_message.J.rows, Jacobian_message.J.columns)
        q_dot = np.dot(np.linalg.pinv(J), v)
        return q_dot

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
            self.position = self.get_current_end_effector_position()
            velocity = self.compute_command_velocity(position_desired)
            self.q_dot = self.compute_command_q_dot(velocity)
            self.panda_controller.send_velocities(self.q_dot)
            self.rate.sleep()
        self.stop()
        return 0

    def go_to_points_in_trajectory(self, trajectory):
        """
        Move through a series of points and stop when arrived.

        Parameters
        ----------
        trajectory : list
            List containing as many 3x1 np.ndarray position vectors as desired.
            Position vectors do not specify rotation
        """
        for point in trajectory:
            self.go_to_point(point)
            self.rate.sleep()
        self.stop()
        return 0

    def get_trajectory_points_in_circle_yz_plane(self, r, x0, y0, z0, resolution):
        # Get trajectory points in a circle at plane x = X, centered at (x0, y0, z0), with radius r
        """
        Get trajectory points in a circle at plane x = X, centered at (x0, y0, z0), with radius r

        Parameters
        ----------
        r: float
            [description]
        x0: float
            [description]
        y0: float
            [description]
        z0: float
            [description]
        resolution: float
            [description]

        Returns
        -------
        trajectory: list
            [description]
        """

        trajectory = []

        for theta in np.arange(0, 2*np.pi, resolution):
            x = x0
            y = y0 + r * np.cos(theta)
            z = z0 + r * np.sin(theta)
            trajectory.append([x, y, z])
        return trajectory

    def stop(self):
        """
        Stop all joints
        """
        q_dot = np.zeros(7)
        self.panda_controller.send_velocities(q_dot)
        return 0

    def return_home(self):
        self.stop()
        q = [0, np.pi/8, 0, -np.pi/2, 0, 3 * np.pi / 4, np.pi/8]
        self.panda_controller.publish_positions(q, 2)


if __name__ == "__main__":
    x0 = 0.6
    y0 = 0.0
    z0 = 0.3
    r = 0.1
    resolution = 0.2

    # Loop that trajectory
    cartesian_controller = CartesianController()
    trajectory = np.array([[0.4, 0, 0.3], [0.7, 0, 0.3]])
    while not rospy.is_shutdown():
        cartesian_controller.go_to_points_in_trajectory(trajectory)
