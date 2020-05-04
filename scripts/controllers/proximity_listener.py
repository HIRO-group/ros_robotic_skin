#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import LaserScan
import numpy as np
from scipy.spatial.transform import Rotation
from visualization_msgs.msg import Marker
from ros_robotic_skin.msg import PointArray
from ros_robotic_skin.msg import IdxPoint
from geometry_msgs.msg import Point


class ProximityListener(object):
    def __init__(self, num_sensors, distance_threshold, use_memory=True):
        self.memory = []
        self.object_distance_threshold = distance_threshold
        self.memory_idx = 100000
        self.SPHERE_RADIUS = (0.23, 0.24, 0.2, 0.237, 0.225, 0.20, 0.27)

        rospy.init_node('proximity_listener')
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=100)
        self.pub_object = rospy.Publisher('obstacle_points', IdxPoint, queue_size=100)

        self.tf_listener = tf.TransformListener()

        for i in range(num_sensors):
            rospy.Subscriber("proximity_data{}".format(i), LaserScan, self.__callback)

        rospy.spin()


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


    def __save_point(self, vector, id=-1):
        """
        add an obstacle from the obstacle avoidance controller

        Parameters
        ----------
        id: int
            unique identifier of object to be added
        vector: list
            xyz position of new obstacle point
        """
        # check if point already exists in memroy
        if self.memory:
            for point in self.memory:

                less_than_threshold = np.linalg.norm(vector - point) < self.object_distance_threshold
                in_sphere = self.__is_in_sphere(vector)
                if less_than_threshold or in_sphere:
                    return 0
        self.memory.append(vector)


        # visualization of memory points
        self.memory_idx = self.memory_idx + 1
        msg = self.__make_marker(True, self.memory_idx, vector, namespace="memory", rgb=(0, 1.0, 0))
        self.pub.publish(msg)


        self.__publish_live_obstacle(vector, id)
        return 0

    def __publish_live_obstacle(self, vector, id):
        """
        Add an obstacle from the obstacle avoidance controller

        Parameters
        ----------
        id: int
            unique identifier of object to be added
        vector: list
            xyz position of new obstacle point
        """
        idx_point = IdxPoint()
        idx_point.idx = id

        # create the physical point
        point = Point()
        point.x = vector[0]
        point.y = vector[1]
        point.z = vector[2]
        idx_point.point = point

        self.pub_object.publish(idx_point)

    def __publish_no_obstacle(self, id):
        """
        Removes an obstacle from the obstacle avoidance controller given an id

        Parameters
        ----------
        id: int
            unique identifier of object to be removed
        """
        idx_point = IdxPoint()
        idx_point.idx = id

        # create the physical point at the bottom of the robot
        point = Point()
        point.x = float(0)
        point.y = float(0)
        point.z = float(0)
        idx_point.point = point

        self.pub_object.publish(idx_point)


    def __callback(self, data):
        """
        Process proximity sensor data to determine of an object should be
        added to the avoidance controller list

        Parameters
        ----------
        data : LaserScan
            data.ranges[0] contains the distance data for the proximity sensor
        """
        distance_reading = data.ranges[0]
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('world', 'proximity_link{}'.format(data.header.frame_id[-1]), rospy.Time(0))
                break
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue

        if distance_reading == float('inf'):
            # Delete old point
            point_id = int(data.header.frame_id[-1])
            msg = self.__make_marker(False, point_id, np.array([0, 0, 0]))
            # Remove visualization
            self.pub.publish(msg)
            self.__publish_no_obstacle(int(data.header.frame_id[-1]))


        else:
            # Locate point in 3D space
            translation1 = np.array(trans)
            r = Rotation.from_quat(rot)
            translation2 = np.array([data.ranges[0], 0, 0])  # Apply rotation in rot to this vector
            position_vector = translation1 + r.apply(translation2)

            # Visualize new point
            msg = self.__make_marker(True, int(data.header.frame_id[-1]),   position_vector, radius=0.07)
            self.pub.publish(msg)

            # Save point in memory
            self.__save_point(position_vector, id=int(data.header.frame_id[-1]))


if __name__ == "__main__":
    num_sensors = 4
    distance_threshold = 0.05
    proximity_listener = ProximityListener(num_sensors, distance_threshold, use_memory=False)
