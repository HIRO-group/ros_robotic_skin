from robotic_skin.sensor.lsm6ds3_accel import LSM6DS3_acclerometer
from std_msgs.msg import Float32
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
    rospy.init_node('talker_%s' % str(accel.config_dict['imu_number']), anonymous=True)
    pub = rospy.Publisher('/imu_data%s' % str(accel.config_dict['imu_number']) + 'norm', Float32, queue_size=10)
    r = rospy.Rate(100)
    norm_float = Float32()
    while not rospy.is_shutdown():
        data0_list = accel.read()
        norm = math.sqrt(data0_list[0]*data0_list[0] +
                         data0_list[1]*data0_list[1] +
                         data0_list[2]*data0_list[2])
        if norm < 9.81:
            print("Norm less than 9.81")
        norm_float.data = norm
        pub.publish(norm_float)
        r.sleep()