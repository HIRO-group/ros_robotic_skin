#!/usr/bin/env python

from PandaController import PandaController
import numpy as np
import rospy
import tf2_ros
import moveit_commander

VMAX = .1
FREQUENCY = 100.
PERIOD = 1. / FREQUENCY
ERROR_THRESHOLD = 0.01


def switch_point():
    pass


if __name__ == '__main__':

    # List of points in desired trajectory
    points = np.array([[.6, .5, .5],
                       [.6, .5, 1],
                       [.6, 1, 1],
                       [.6, 1, .5]])
    points = points / 2.

    pc = PandaController()

    # tf
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Moveit
    robot_commander = moveit_commander.RobotCommander()
    group_names = robot_commander.get_group_names()
    move_group_commander = moveit_commander.move_group.MoveGroupCommander(group_names[1])

    rate = rospy.Rate(FREQUENCY)

    t = 0
    points_idx = 4 % len(points)

    while not rospy.is_shutdown():
        try:
            # Current position vector
            transformation = tfBuffer.lookup_transform('panda_link8', 'world', rospy.Time())
            translation = transformation.transform.translation
            vector_0_EE = np.array([translation.x, translation.y, translation.z])

            # Desired position vector
            vector_0_EE_d = points[points_idx]
            # Error vector
            vector_error = vector_0_EE_d - vector_0_EE
            vector_error_norm = np.linalg.norm(vector_error)
            vector_error_unit = vector_error / vector_error_norm
            # Get Jacobian
            q = move_group_commander.get_current_joint_values()
            J = move_group_commander.get_jacobian_matrix(q)

            # End effector cartesian velocity
            velocity_translation = VMAX * vector_error_unit
            velocity_rotation = np.array([0, 0, 0])
            velocity = np.block([velocity_translation, velocity_rotation])
            velocity.shape = (6, 1)

            # Convert cartesian velocities to joint velocities
            q_dot = np.linalg.pinv(J) * velocity
            pc.publish_velocities(q_dot, PERIOD)

            # Print debug data every second
            if t % FREQUENCY == 0:
                print('End effector position: \n[{} ,{}, {}]').format(transformation.transform.translation.x,
                                                                      transformation.transform.translation.y,
                                                                      transformation.transform.translation.z)
                print('End effector desired position {}: \n{}').format(points_idx, vector_0_EE_d)
                print('------------------------------')

            # Decide if it's time to switch to the next point
            if vector_error_norm <= ERROR_THRESHOLD:
                points_idx = (points_idx + 1) % len(points)

            t = t + 1
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            continue
