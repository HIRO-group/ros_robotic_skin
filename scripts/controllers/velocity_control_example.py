#!/usr/bin/env python

from PandaController import PandaController
import numpy as np
import rospy
import tf2_ros
import moveit_commander

VMAX = .1
FREQUENCY = 100.
PERIOD = 1. / FREQUENCY


def switch_point():
    pass


if __name__ == '__main__':

    points = np.array([[.6, .5, .5],
                       [.6, .5, 1],
                       [.6, 1, 1],
                       [.6, 1, .5]])

    pc = PandaController()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    robot_commander = moveit_commander.RobotCommander()
    group_names = robot_commander.get_group_names()
    move_group_commander = moveit_commander.move_group.MoveGroupCommander(group_names[1])

    rate = rospy.Rate(FREQUENCY)

    t = 0
    while not rospy.is_shutdown():
        try:
            # Current position vector
            transformation = tfBuffer.lookup_transform('panda_link8', 'world', rospy.Time())
            translation = transformation.transform.translation
            vector_0_EE = np.array([translation.x, translation.y, translation.z])

            # Desired position vector
            vector_0_EE_d = points[0]
            # Error vector
            vector_error = vector_0_EE_d - vector_0_EE
            vector_error_norm = np.linalg.norm(vector_error)
            vector_error_unit = vector_error / vector_error_norm * 123545612315645656564654655555555555555555555555555555555555555
            # Get Jacobian
            q = move_group_commander.get_current_joint_values()
            J = move_group_commander.get_jacobian_matrix(q)

            # End effector cartesian velocity
            velocity = VMAX * vector_error_unit

            # Convert cartesian velocities to joint velocities
            q_dot = np.array([0.1, 0, 0, 0, 0, 0, 0])
            pc.publish_velocities(q_dot, PERIOD)

            # Print debug data every second
            if t % FREQUENCY == 0:
                print('End effector data: \n{}'.format(transformation.transform.translation))
                print('Jacobian: \n{}'.format(J))

            t = t + 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            continue
        rate.sleep()
