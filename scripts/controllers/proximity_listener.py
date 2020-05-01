#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker
from ros_robotic_skin.msg import PointArray
from geometry_msgs.msg import Point

memory = []
THRESHOLD = 0.03
idx = 100000
SPHERE_RADIUS = (0.23, 0.24, 0.2, 0.237, 0.225, 0.20, 0.27)


def make_marker(is_add, id, xyz, namespace="live_readings", rgb=(1.0, 0, 0), radius=0.05):
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
    msg.color.a = 1.0
    return msg


def is_in_sphere(vector):
    for i in range(len(SPHERE_RADIUS)):
        (trans, rot) = tf_listener.lookupTransform('world', 'control_point{}'.format(i), rospy.Time(0))
        sphere_center = np.array(trans)
        distance = np.linalg.norm(sphere_center - vector)
        if distance < SPHERE_RADIUS[i]:
            return True
    return False


def save_point(vector):
    global memory, idx

    if memory:
        for point in memory:
            cond1 = np.linalg.norm(vector - point) < THRESHOLD
            cond2 = is_in_sphere(vector)
            if cond1 or cond2:
                # print("Not saving anything")
                return 0
    memory.append(vector)
    idx = idx + 1
    # print(idx - 100000)
    msg = make_marker(True, idx, vector, namespace="memory", rgb=(0, 1.0, 0))
    pub.publish(msg)

    points = []
    for mem in memory:
        point = Point()
        point.x = mem[0]
        point.y = mem[1]
        point.z = mem[2]
        points.append(point)
    pub2.publish(points)

    return 0


def callback(data):
    while True:
        try:
            (trans, rot) = tf_listener.lookupTransform('world', 'proximity_link{}'.format(data.header.frame_id[-1]), rospy.Time(0))
            break
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            continue

    if data.ranges[0] == float('inf'):
        # Delete point
        position_vector = np.array([0, 0, 0])
        msg = make_marker(False, int(data.header.frame_id[-1]), position_vector)
        pub.publish(msg)

    else:
        # Show point
        translation1 = np.array(trans)
        r = Rotation.from_quat(rot)
        translation2 = np.array([data.ranges[0], 0, 0])  # Apply rotation in rot to this vector

        position_vector = translation1 + r.apply(translation2)

        msg = make_marker(True, int(data.header.frame_id[-1]),   position_vector, radius=0.07)
        pub.publish(msg)
        save_point(position_vector)


rospy.init_node('proximity_listener')
tf_listener = tf.TransformListener()
for i in range(4):
    rospy.Subscriber("proximity_data{}".format(i), LaserScan, callback)

pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)
pub2 = rospy.Publisher('obstacle_points', PointArray, queue_size=100)
rospy.spin()

# TODO Add the transform feature (krishna quaternions)
# TODO Memory of objects (have into account if it's  new point and visualize those points, is it in the body spheres)
