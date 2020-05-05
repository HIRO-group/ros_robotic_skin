#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker
# from ros_robotic_skin.msg import PointArray
# from ros_robotic_skin.msg import IdxPoint
# from geometry_msgs.msg import Point
import re
from time import time


class ProximityListener(object):
    def __init__(self, num_sensors, distance_threshold, use_memory=True):
        self.object_distance_threshold = distance_threshold
        self.SPHERE_RADIUS = (0.23, 0.24, 0.2, 0.237, 0.225, 0.20, 0.27)
        self.live_points = [np.array([20.0, 20.0, 20.0]) for i in range(num_sensors)]

        # Variables for printing stats
        self.number_of_vectors = 0
        self.time_reading2reading = []
        self.init_time = time()
        self.time_specific_sensor = time()
        self.num_sensors = num_sensors

        rospy.init_node('proximity_listener')
        rospy.on_shutdown(self.__shutdown_callback)
        # Publishing to
        self.pub_visualization = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        # self.pub_object = rospy.Publisher('obstacle_points', IdxPoint, queue_size=1)
        # Subscribed to
        self.tf_listener = tf.TransformListener()
        for i in range(num_sensors):
            rospy.Subscriber("proximity_data{}".format(i), LaserScan, self.__callback)

        rospy.spin()

    def __callback(self, data):
        """
        Process proximity sensor data to determine of an object should be
        added to the avoidance controller list

        Parameters
        ----------
        data : LaserScan
            data.ranges[0] contains the distance data for the proximity sensor
        """

        point_id = int(re.match('.*?([0-9]+)$', data.header.frame_id).group(1))
        distance_reading = data.ranges[0]

        # Stats
        if point_id == ID_STATS:
            self.time_reading2reading.append(time() - self.time_specific_sensor)
            self.time_specific_sensor = time()
            # self.readings_list.append(distance_reading)

        if distance_reading == float('inf'):
            # Delete old point
            # msg = self.__make_marker(False, point_id, np.array([0, 0, 0]))
            # Remove visualization
            # self.pub_visualization.publish(msg)
            # self.__publish_no_obstacle(point_id)
            self.live_points[point_id] = np.array([20.0, 20.0, 20.0])
            if point_id == ID_STATS:
                self.readings_list.append(-1)

        else:
            # Get proximity sensor's pose
            try:
                (trans, rot) = self.tf_listener.lookupTransform('world', 'proximity_link{}'.format(point_id), rospy.Time(0))

            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                (trans, rot) = ([0.0, 0.0, 0.0], [0, 0, 0, 1])
            # Locate point in 3D space
            translation1 = np.array(trans)
            r = Rotation.from_quat(rot)
            translation2 = np.array([distance_reading, 0, 0])  # Apply rotation in rot to this vector
            position_vector = translation1 + r.apply(translation2)
            # position_vector = distance_reading
            self.live_points[point_id] = position_vector
            # Stats
            if point_id == ID_STATS:
                self.number_of_vectors = self.number_of_vectors + 1

    def __shutdown_callback(self):
        """
        Print some stats at shutdown
        """
        end_time = time()
        print("Number of sensors in use: {}".format(self.num_sensors))
        print("Sensor being tracked: {}".format(ID_STATS))
        print("-------------------------------------------------")
        print("Vectors computed per second by i: {}".format(self.number_of_vectors/(end_time-self.init_time)))
        self.time_reading2reading.pop(0)
        avg = np.average(self.time_reading2reading)
        print("Min time i: {}".format(min(self.time_reading2reading)))
        print("Avg time and frequency i: {}, {}".format(avg, avg**-1))
        print("Max time i: {}".format(max(self.time_reading2reading)))

    def __is_in_sphere(self, vector):
        """
        Determine if an obstacle is inside of the robot arm and therefore not
        really an obstacle

        Parameters
        ----------
        vector: list
             xyz position of potential obstacle point

        Returns
        -------
        bool:
            True if point is a new obstacle

        """
        for i in range(len(self.SPHERE_RADIUS)):
            (trans, _) = self.tf_listener.lookupTransform('world', 'control_point{}'.format(i), rospy.Time(0))
            sphere_center = np.array(trans)
            distance = np.linalg.norm(sphere_center - vector)
            if distance < self.SPHERE_RADIUS[i]:
                return True
        return False

    def __make_marker(self, is_add, id, xyz, namespace="live_readings", rgb=(1.0, 0, 0), radius=0.05):
        """
        Creates a marker to visualize a point

        Parameters
        ----------
        is_add: bool
            is point being added or deleted
        id: int
            unique identifier of object to be added
        xyz: list
            xyz position of new point
        namespace: string
            namespace for object visualization in rvis, used to toggle visualization
        rgb: tuple
            red, green, blue values of marker
        radius: float
            radius of sphere added to visualization

        Returns
        -------
        msg: Marker
            New point to be visualized in Rvis
        """
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


if __name__ == "__main__":
    # Steps:
    # Suscribbe to all proximity sensors and check (time, data...)
    # Place readings in space and check (time, data...)
    # Pack the readings in a python list. This list should be edited one element at a time.
    # Publish the list in the appropiate message type.
    # In another node visualize the points in the published list.
    # Leverage old memory code, check if it's in a sphere,...

    ID_STATS = 146
    num_sensors = 161
    distance_threshold = 0.3
    proximity_listener = ProximityListener(num_sensors, distance_threshold, use_memory=False)
