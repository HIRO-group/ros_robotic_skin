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
    config_data = rosparam.load_file(config_file)
    imu_number = rospy.get_param("imu_number", config_data)
    GRAVITATIONAL_CONSTANT = rospy.get_param("GRAVITATIONAL_CONSTANT", config_data)
    RPi_bus_num = rospy.get_param("RPi_bus_num", config_data)
    ros_core_ip = rospy.get_param("ros_core_ip", config_data)  # The ROS Core IP
    # 11311 is the default port, you shouldn't change this unless you know what you are doing
    ros_core_port = rospy.get_param("ros_core_port", config_data)
    RPi_IP = rospy.get_param("RPi_IP", config_data)  # The IP of RPi which will be sending packets
    # First Let's initialize all the environment variables so that ROS doesn't whine about it
    os.environ["ROS_MASTER_URI"] = 'http://%s:%d' % (ros_core_ip, ros_core_port)
    os.environ["ROS_IP"] = RPi_IP
    # Okay so now lets initialize the accelerometer and send the packets to ROS Core
    accel = LSM6DS3_acclerometer(bus_num=RPi_bus_num)
    rospy.init_node('talker_' + str(imu_number), anonymous=True)
    pub = rospy.Publisher('/imu_data' + str(imu_number), Imu, queue_size=10)
    r = rospy.Rate(100)
    imu_msg = Imu()
    while not rospy.is_shutdown():
        data0_list = accel.read()
        imu_msg.linear_acceleration.x = data0_list[0] * GRAVITATIONAL_CONSTANT
        imu_msg.linear_acceleration.y = data0_list[1] * GRAVITATIONAL_CONSTANT
        imu_msg.linear_acceleration.z = data0_list[2] * GRAVITATIONAL_CONSTANT
        pub.publish(imu_msg)
        r.sleep()
