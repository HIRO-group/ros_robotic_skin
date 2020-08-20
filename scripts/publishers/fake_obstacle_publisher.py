#!/usr/bin/env python

import rospy
from ros_robotic_skin.msg import PointArray
from geometry_msgs.msg import Point

def talker():
    pub = rospy.Publisher('/live_points', PointArray, queue_size=1)
    rospy.init_node('obstacle_publisher', anonymous=True)
    rate = rospy.Rate(50) # 10hz

    point = Point()
    point.x = 0.5
    point.y = -0.25
    point.z = 0.75

    while not rospy.is_shutdown():
        print("publish")
        pub.publish([point])
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass