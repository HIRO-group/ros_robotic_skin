#!/usr/bin/env python

import rospy
import tf
from visualization_msgs.msg import Marker


def make_marker(is_add, id, xyz, namespace="speheres", rgb=(1.0, 0, 0), radius=0.05):
    msg = Marker()
    msg.pose.position.x = xyz[0]
    msg.pose.position.y = xyz[1]
    msg.pose.position.z = xyz[2]
    msg.header.frame_id = "world"
    msg.type = Marker.SPHERE
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    msg.header.stamp = rospy.Time()
    msg.ns = namespace
    if is_add:
        msg.action = Marker.ADD
    else:
        msg.action = Marker.DELETE
    msg.id = id
    msg.scale.x = radius
    msg.scale.y = radius
    msg.scale.z = radius
    msg.color.r = rgb[0]
    msg.color.g = rgb[1]
    msg.color.b = rgb[2]
    msg.color.a = 0.8
    return msg


RADIUS = (0.23, 0.24, 0.2, 0.237, 0.225, 0.20, 0.27)

rospy.init_node('spheres_visualizer')
tf_listener = tf.TransformListener()
pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)

while not rospy.is_shutdown():
    for i in range(len(RADIUS)):
        while True:
            try:
                (trans, rot) = tf_listener.lookupTransform('world', 'control_point{}'.format(i), rospy.Time(0))
                break
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue

        msg = make_marker(True, i, trans, radius=RADIUS[i])
        pub.publish(msg)
