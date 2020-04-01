#!/usr/bin/env python

from PandaController import PandaController
import numpy as np
import rospy
import moveit_commander
import tf


class CartesianController(object):
    """
    Cartesian controller built on top of PandaController. clean code
    """

    # TODO: Fix: When trajectory points are to close to each other, the movement is intermittent, because self.stop() is called too much.
    # TODO: Avoid self collision

    gain = .2
    FREQUENCY = 100.
    ERROR_THRESHOLD = 0.01

    def __init__(self):
        self.panda_controller = PandaController()
        self.move_group_commander = moveit_commander.move_group.MoveGroupCommander('panda_arm')
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(self.FREQUENCY)
        self.q_dot = np.zeros(7)
        rospy.on_shutdown(self.stop)

        self.position = self.__get_current_end_effector_position()

    def __get_current_end_effector_position(self):
        """
        Get the current end effector position in cartesian coordinates

        Returns
        -------
        trans : numpy.ndarray
            3x1 vector containing current end effector position vector
        """
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('world', 'panda_hand', rospy.Time(0))
                return np.array(trans)
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue

    def __compute_command_velocity(self, position_desired):
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
        v_trans = self.gain * self.error / np.linalg.norm(self.error)
        v_rot = np.array([0, 0, 0])
        v = np.block([v_trans, v_rot])
        v.shape = (6, 1)
        return v

    def __compute_command_q_dot(self, v):
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
        q = self.move_group_commander.get_current_joint_values()
        J = self.move_group_commander.get_jacobian_matrix(q)
        q_dot = np.linalg.pinv(J) * v
        return q_dot

    def go_to_point(self, position_desired):
        """
        Move to a point an stop when arrived

        Parameters
        ----------
        position_desired : numpy.ndarray
            3x1 end effector desired position vector
        """
        position_desired = np.array(position_desired)
        self.error = position_desired - self.position
        while not np.linalg.norm(self.error) < self.ERROR_THRESHOLD:
            self.position = self.__get_current_end_effector_position()
            velocity = self.__compute_command_velocity(position_desired)
            self.q_dot = self.__compute_command_q_dot(velocity)
            self.panda_controller.send_velocities(self.q_dot)
            self.rate.sleep()
        self.stop()
        return 0

    def go_to_points_in_trajectory(self, trajectory):
        """
        Move through a series of points and stop when arrived

        Parameters
        ----------
        trajectory : list
            list containing as many np.ndarray vectors as desired
        """
        for point in trajectory:
            self.go_to_point(point)
            self.rate.sleep()
        self.stop()
        return 0

    def stop(self):
        """
        Stop all joints
        """
        q_dot = np.zeros(7)
        self.panda_controller.send_velocities(q_dot)
        return 0


if __name__ == "__main__":

    # Get trajectory points in a circle at plane x = X, centered at (X,a,b), with radius r

    trajectory = []
    X = 0.3
    a = 0.0
    b = 0.3
    r = 0.1
    for theta in np.arange(0, 2*np.pi, 0.2):
        x = X
        y = a + r * np.cos(theta)
        z = b + r * np.sin(theta)
        trajectory.append([x, y, z])

    # Loop that trajectory
    cartesian_controller = CartesianController()
    while not rospy.is_shutdown():
        cartesian_controller.go_to_points_in_trajectory(trajectory)
