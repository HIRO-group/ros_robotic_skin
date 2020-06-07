from robotic_skin.sensor.lsm6ds3_accel import LSM6DS3_acclerometer
from robotic_skin.sensor.lsm6ds3_gyro import LSM6DS3_gyroscope
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
    accel = LSM6DS3_acclerometer(config_file)
    gyro = LSM6DS3_gyroscope(config_file)
    rospy.init_node('talker_%s' % str(accel.config_dict['imu_number']), anonymous=True)
    pub = rospy.Publisher('/imu_data%s' % str(accel.config_dict['imu_number']), Imu, queue_size=10)
    imu_msg = Imu()
    while not rospy.is_shutdown():
        accel_list = accel.read()
        gyro_list = gyro.read()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = accel_list[0]
        imu_msg.linear_acceleration.y = accel_list[1]
        imu_msg.linear_acceleration.z = accel_list[2]
        imu_msg.angular_velocity.x = gyro_list[0]
        imu_msg.angular_velocity.y = gyro_list[1]
        imu_msg.angular_velocity.z = gyro_list[2]
        pub.publish(imu_msg)
        # For some reason the rospy sleep doesn't work here
        # IDK why, so it's better to use python sleep
        sleep(0.1)
