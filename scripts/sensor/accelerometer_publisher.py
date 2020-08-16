import os
import argparse

import rospy
import rospkg
from sensors import IMU
from sensor_msgs.msg import Imu
ROS_ROBOSKIN_DIR = rospkg.RosPack().get_path('ros_robotic_skin')


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--filename', type=str, default='environment_variables.yaml',
                        help="The environment variable yaml file name in config directory")
    parser.add_argument('--imu_num', type=int, required=True,
                        help="Publish topic as this given number")
    parser.add_argument('--raspi_bus', type=int, required=True,
                        help="Raspi's bus number for the given specific imu")

    return parser.parse_args()


if __name__ == "__main__":
    """
    code for publishing real accelerometer data in ROS.
    """
    args = parse_arguments()

    # Init ROS Node
    rospy.init_node('talker_{imu_num}'.format(imu_num=args.imu_num), anonymous=True)
    # Prepare publisher
    pub = rospy.Publisher('/imu_data{imu_num}'.format(imu_num=args.imu_num), Imu, queue_size=10)
    r = rospy.Rate(100)

    # Prepare IMU
    imu = IMU(raspi_bus=args.raspi_bus)

    # Publish IMU topics
    imu_msg = Imu()
    while not rospy.is_shutdown():
        data = imu.read()
        imu_msg.header.frame_id = 'imu_data{imu_num}'.format(imu_num=args.imu_num)
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = data[0]
        imu_msg.linear_acceleration.y = data[1]
        imu_msg.linear_acceleration.z = data[2]
        imu_msg.angular_velocity.x = data[3]
        imu_msg.angular_velocity.y = data[4]
        imu_msg.angular_velocity.z = data[5]
        pub.publish(imu_msg)
        r.sleep()
