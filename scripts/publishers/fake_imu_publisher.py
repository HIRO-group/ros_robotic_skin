#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Imu


if __name__ == "__main__":
    if len(sys.argv) < 1:
        rospy.logerr("usage: fake_imu_publisher.py imu_number")

    i_imu = int(sys.argv[1])

    rospy.init_node('fake_imu_publisher{}'.format(i_imu))
    pub = rospy.Publisher('/imu_data{}'.format(i_imu), Imu, queue_size=10)
    imu_msg = Imu()
    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        imu_msg.header.frame_id = 'imu_link' + str(i_imu)
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = 1.0
        imu_msg.linear_acceleration.y = 1.0
        imu_msg.linear_acceleration.z = 1.0
        imu_msg.angular_velocity.x = 1.0
        imu_msg.angular_velocity.y = 1.0
        imu_msg.angular_velocity.z = 1.0
        pub.publish(imu_msg)
        r.sleep()
