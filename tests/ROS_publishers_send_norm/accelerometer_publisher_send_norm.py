from robotic_skin.sensor.lsm6ds3_accel import LSM6DS3_acclerometer
import rospy
import rospkg
import argparse
import math


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
    pub = rospy.Publisher('/imu_data' + str(accel.config_dict['imu_number']) + 'norm', float, queue_size=10)
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        data0_list = accel.read()
        norm = math.sqrt(data0_list[0]*data0_list[0] +
                         data0_list[1]*data0_list[1] +
                         data0_list[2]*data0_list[2])
        pub.publish(norm)
        r.sleep()
