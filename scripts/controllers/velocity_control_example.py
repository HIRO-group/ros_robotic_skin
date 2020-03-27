#!/usr/bin/env python

from PandaController import PandaController
import numpy as np
import rospy
import tf2_ros

VMAX = .1
FREQUENCY = 50.
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

    rate = rospy.Rate(FREQUENCY)
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

            # End effector cartesian velocity
            velocity = VMAX * vector_error_unit
            # TODO
            # q = Jinv * velocity
            # pc.publish_velocities(q, PERIOD)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            continue
        rate.sleep()
