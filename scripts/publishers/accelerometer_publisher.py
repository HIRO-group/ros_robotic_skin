from robotic_skin.sensor.lsm6ds3 import LSM6DS3_IMU
from time import sleep
from sensor_msgs.msg import Imu
import rospy
import rospkg
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--config_file', action="store",
                        dest="config_file", required=True,
                        help="The configuration yaml file name in package's config folder")
    args = parser.parse_args()
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    config_file = ros_robotic_skin_path + "/config/" + args.config_file
    accel_gyro = LSM6DS3_IMU(config_file)
    rospy.init_node('talker_%s' % str(accel_gyro.config_dict['imu_number']), anonymous=True)
    pub = rospy.Publisher('/imu_data%s' % str(accel_gyro.config_dict['imu_number']), Imu, queue_size=10)
    imu_msg = Imu()
    while not rospy.is_shutdown():
        accel_gyro_list = accel_gyro.read()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = accel_gyro_list[0]
        imu_msg.linear_acceleration.y = accel_gyro_list[1]
        imu_msg.linear_acceleration.z = accel_gyro_list[2]
        imu_msg.angular_velocity.x = accel_gyro_list[3]
        imu_msg.angular_velocity.y = accel_gyro_list[4]
        imu_msg.angular_velocity.z = accel_gyro_list[5]
        pub.publish(imu_msg)
        # For some reason the rospy sleep doesn't work here
        # IDK why, so it's better to use python sleep
        sleep(0.1)
