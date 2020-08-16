from roboskin.sensor.lsm6ds3 import LSM6DS3_IMU
from sensor_msgs.msg import Imu
import rospy
import os
import rospkg
import argparse

if __name__ == "__main__":
    """
    code for publishing real accelerometer data in ROS.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--config_file', action="store",
                        dest="config_file", required=True,
                        help="The configuration yaml file name in package's config folder")
    args = parser.parse_args()
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')

    config_file = os.path.join(ros_robotic_skin_path, 'config', args.config_file)
    accel_gyro = LSM6DS3_IMU(config_file)
    imu_number = str(accel_gyro.config_dict['imu_number'])

    rospy.init_node('talker_{imu_number}'.format(imu_number=imu_number), anonymous=True)
    pub = rospy.Publisher('/imu_data{imu_number}'.format(imu_number=imu_number), Imu, queue_size=10)
    r = rospy.Rate(100)

    imu_msg = Imu()
    while not rospy.is_shutdown():
        accel_gyro_list = accel_gyro.read()
        imu_msg.header.frame_id = imu_number
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = accel_gyro_list[0]
        imu_msg.linear_acceleration.y = accel_gyro_list[1]
        imu_msg.linear_acceleration.z = accel_gyro_list[2]
        imu_msg.angular_velocity.x = accel_gyro_list[3]
        imu_msg.angular_velocity.y = accel_gyro_list[4]
        imu_msg.angular_velocity.z = accel_gyro_list[5]
        pub.publish(imu_msg)
        r.sleep()
