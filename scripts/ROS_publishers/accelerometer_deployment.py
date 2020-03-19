from robotic_skin.sensor.lsm6ds3_accel import LSM6DS3_acclerometer
import json
import os
from sensor_msgs.msg import Imu
import rospy
import argparse
import rospkg

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--accel1_config_file', action="store",
                        dest="accel1_config_file", required=True,
                        help="The configuration yaml file name of acceleromter 1 in package's config folder")
    parser.add_argument('--accel2_config_file', action="store",
                        dest="accel2_config_file", required=True,
                        help="The configuration yaml file name of acceleromter 2 in package's config folder")
    args = parser.parse_args()
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    config_file_accel1 = ros_robotic_skin_path + "/config/" + args.accel1_config_file
    config_file_accel2 = ros_robotic_skin_path + "/config/" + args.accel2_config_file
    accel_0 = LSM6DS3_acclerometer(config_file_accel1)
    accel_1 = LSM6DS3_acclerometer(config_file_accel2)
    ros_node_name = 'talker_%s_%s' % (str(accel_0.config_dict['imu_number']), str(accel_1.config_dict['imu_number']))
    rospy.init_node(ros_node_name, anonymous=True)
    pub0 = rospy.Publisher('/imu_data' + str(accel_0.config_dict['imu_number']), Imu, queue_size=10)
    pub1 = rospy.Publisher('/imu_data' + str(accel_1.config_dict['imu_number']), Imu, queue_size=10)
    r = rospy.Rate(100)
    imu_msg0 = Imu()
    imu_msg1 = Imu()
    while not rospy.is_shutdown():
        data0_list = accel_0.read()
        data1_list = accel_1.read()
        imu_msg0.linear_acceleration.x = data0_list[0]
        imu_msg0.linear_acceleration.y = data0_list[1]
        imu_msg0.linear_acceleration.z = data0_list[2]
        pub0.publish(imu_msg0)
        imu_msg1.linear_acceleration.x = data1_list[0]
        imu_msg1.linear_acceleration.y = data1_list[1]
        imu_msg1.linear_acceleration.z = data1_list[2]
        pub1.publish(imu_msg0)
        r.sleep()
