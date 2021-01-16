import argparse
import numpy as np

import rospy
import tf
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from tf import TransformListener


class RangePublisher:
    def __init__(self, obstacle_position, topic_name='/proximity_data6'):
        # Prepare publisher
        self.obstacle_position = obstacle_position
        self.proximity_publisher = rospy.Publisher(topic_name, Range, queue_size=10)
        self.start_subscriber = rospy.Subscriber('/at_start_position', Bool, self.start_callback)
        self.listener = TransformListener()
        self.r = rospy.Rate(100)

        self.range_msg = Range()
        self.range_msg.header.frame_id = topic_name
        self.range_msg.header.stamp = rospy.Time.now()
        self.range_msg.range = np.inf

        self.at_start_position = False

    def get_ee_position(self):
        try:
            (position, quaternion) = self.listener.lookupTransform("/world", "/panda_link8", rospy.Time(0))
            return position
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def start_callback(self, msg):
        self.at_start_position = msg.data

    def spin(self):
        while not rospy.is_shutdown():
            ee_position = self.get_ee_position()

            if self.at_start_position and ee_position is not None:
                distance = self.obstacle_position - ee_position
                self.range_msg.range = np.linalg.norm(distance)

            self.proximity_publisher.publish(self.range_msg)
            self.r.sleep()


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('-ox', '--obstacle_x', type=float, default=0.3)
    parser.add_argument('-oy', '--obstacle_y', type=float, default=-0.2)
    parser.add_argument('-oz', '--obstacle_z', type=float, default=0.5)

    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()

    # Init ROS Node
    rospy.init_node('range_publisher', anonymous=True)

    obstacle_position = np.array([
        args.obstacle_x,
        args.obstacle_y,
        args.obstacle_z
    ])

    publisher = RangePublisher(obstacle_position)
    publisher.spin()
