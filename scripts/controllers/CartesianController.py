#!/usr/bin/env python

from PandaController import PandaController
import numpy as np
import rospy
import moveit_commander
import tf


class CartesianController(object):
    """
    Cartesian controller built on top of PandaController.
    """
    K = .2
    FREQUENCY = 100.
    ERROR_THRESHOLD = 0.01

    def __init__(self):
        self.panda_controller = PandaController()
        self.move_group_commander = moveit_commander.move_group.MoveGroupCommander('panda_arm')
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(self.FREQUENCY)
        rospy.on_shutdown(self.stop)

        self.P = self.__get_current_end_effector_position()

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

    def __compute_command_velocity(self, Pd):
        """
        Compute the velocity vector in cartesian coordinates that leads to the Point Pd at a given time instant.

        Parameters
        ----------
        Pd : numpy.ndarray
            3x1 end effector desired position vector

        Returns
        -------
        v: numpy.ndarray
            6x1 velocity vector in cartesian coordinates

        """
        self.error = Pd - self.P
        v_trans = self.K * self.error / np.linalg.norm(self.error)
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

    def command_point(self, Pd):
        """
        Move to a point an stop when arrived

        Parameters
        ----------
        Pd : numpy.ndarray
            3x1 end effector desired position vector
        """
        Pd = np.array(Pd)
        self.error = Pd - self.P
        while not np.linalg.norm(self.error) < self.ERROR_THRESHOLD:
            self.P = self.__get_current_end_effector_position()
            v = self.__compute_command_velocity(Pd)
            q_dot = self.__compute_command_q_dot(v)
            self.panda_controller.send_velocities(q_dot)
        self.stop()
        return 0

    def command_trajectory(self, trajectory):
        """
        Move through a series of points and stop when arrived

        Parameters
        ----------
        trajectory : list
            list containing as many np.ndarray vectors as desired
        """
        for point in trajectory:
            self.command_point(point)
        return 0

    def stop(self):
        """
        Stop all joints
        """
        q_dot = [0 for i in range(7)]
        self.panda_controller.send_velocities(q_dot)
        return 0


if __name__ == "__main__":

    # Get trajectory points in a circle at plane x = X, centered at (X,a,b), with radius r
    trajectory = []
    X = 0.3
    a = 0.0
    b = 0.3
    r = 0.2
    for theta in np.arange(0, 2*3.14, 0.01):
        x = X
        y = a + r * np.cos(theta)
        z = b + r * np.sin(theta)
        trajectory.append([x, y, z])

    # Loop that trajectory
    cartesian_controller = CartesianController()
    while not rospy.is_shutdown():
        cartesian_controller.command_trajectory(trajectory)
