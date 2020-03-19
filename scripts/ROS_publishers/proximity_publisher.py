#!/usr/bin/env python

"""
This is a ROS proximity data publisher
"""
import sys
import rospy
from sensor_msgs.msg import Range
import vl53l1x
import argparse
import rospkg


def publish_proximity(config_file, debug):
    """
    publish_proximity() function
    """
    ps = vl53l1x.VL53L1X_ProximitySensor(config_file)
    rospy_node_name = 'proximity_publisher_%s' % (ps.config_dict['proximity_sensor_number'])
    rospy.init_node(rospy_node_name, anonymous=True)
    pub = rospy.Publisher('/proximity_data%s' % (ps.config_dict['proximity_sensor_number']), Range, queue_size=10)
    rate = rospy.Rate(100)  # Start publishing at 100hz
    range_msg = Range()
    range_msg.radiation_type = 1
    range_msg.min_range = 0
    range_msg.max_range = 3
    # Reference: https://www.st.com/resource/en/datasheet/vl53l1x.pdf
    range_msg.field_of_view = 39.60
    while not rospy.is_shutdown():
        range_msg.range = ps.read()
        if debug:
            print(range_msg)
        pub.publish(range_msg)
        rate.sleep()
    ps.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--config_file', action="store",
                        dest="config_file", required=True,
                        help="The configuration yaml file name in package's config folder")
    parser.add_argument('--DEBUG', action="store",
                        dest="debug", required=False, default=False,
                        help="Whether to print debug strings or not")
    args = parser.parse_args()
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    config_file = ros_robotic_skin_path + "/config/" + args.config_file
    publish_proximity(config_file, args.debug)
