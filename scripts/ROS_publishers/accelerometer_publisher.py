"""
This will serve as quick example for sending the accelerometer to required ROS core node
By setting all environment variables as clearly explained in my blog
https://krishnachaitanya9.github.io/posts/ros_publish_subscribe/
"""
from robotic_skin.sensor.lsm6ds3_accel import LSM6DS3_acclerometer
import os
from sensor_msgs.msg import Imu
import rospy
import rospkg
import rosparam
import argparse

# Global Variables
# I know setting them is bad, this is just a clean straight example of a Proof-Of-Concept
# That can be used to understand how things work
# You need to set all of them below, Else script might misbehave
# Also make sure no white spaces in the variables

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--config_file', action="store",
                        dest="config_file", required=True,
                        help="The configuration yaml file name in package's config folder")
    args = parser.parse_args()
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    config_file = ros_robotic_skin_path + "/config/" + args.config_file
    accel = LSM6DS3_acclerometer(config_file)
    rospy.init_node('talker_' + str(accel.config_dict['imu_number']), anonymous=True)
    pub = rospy.Publisher('/imu_data' + str(accel.config_dict['imu_number']), Imu, queue_size=10)
    r = rospy.Rate(100)
    imu_msg = Imu()
    while not rospy.is_shutdown():
        data0_list = accel.read()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = data0_list[0]
        imu_msg.linear_acceleration.y = data0_list[1]
        imu_msg.linear_acceleration.z = data0_list[2]
        pub.publish(imu_msg)
        r.sleep()
