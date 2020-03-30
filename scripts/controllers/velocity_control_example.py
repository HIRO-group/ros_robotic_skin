#!/usr/bin/env python

from PandaController import PandaController
import numpy as np
import rospy
import moveit_commander

VMAX = .1
FREQUENCY = 100.
PERIOD = 1. / FREQUENCY
ERROR_THRESHOLD = 0.01
q_dot_before = [.1 for i in range(7)]


def stop():
    q_dot = [0 for i in range(7)]
    pc.send_velocities(q_dot)


if __name__ == '__main__':

    # List of points in desired trajectory
    points = np.array([[.6, .5, .5],
                       [.6, .5, 1],
                       [.6, 1, 1],
                       [.6, 1, .5]])
    points = points / 2

    pc = PandaController()

    # Moveit
    robot_commander = moveit_commander.RobotCommander()
    group_names = robot_commander.get_group_names()
    move_group_commander = moveit_commander.move_group.MoveGroupCommander(group_names[1])

    rate = rospy.Rate(FREQUENCY)
    rospy.on_shutdown(stop)

    t = 0
    # Start point
    points_idx = 0 % len(points)
    points_idx_before = points_idx

    while not rospy.is_shutdown():

        # Current position vector
        transformation = move_group_commander.get_current_pose()
        translation = transformation.pose.position
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
        pc.send_velocities(q_dot)

        # Print debug data every second
        if t % FREQUENCY == 0 or points_idx != points_idx_before:
            print('ROS Time: {}'.format(rospy.get_time()))
            print('Desired position [{}]: \n{}').format(points_idx, vector_0_EE_d)
            print('Current position: \n[{}]').format(vector_0_EE)
            print('------------------------------')

        # Decide if it's time to switch to the next point
        points_idx_before = points_idx
        if vector_error_norm <= ERROR_THRESHOLD:
            points_idx = (points_idx + 1) % len(points)

        t = t + 1
        rate.sleep()
