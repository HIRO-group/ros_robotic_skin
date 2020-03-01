#include "PoseEstimationPublisher.h"

ros::NodeHandle* PosePublisher::nh;
ros::Publisher PosePublisher::imu0_pose;
//ros::Publisher PosePublisher::imu1_pose;
//ros::Publisher PosePublisher::imu2_pose;
//ros::Publisher PosePublisher::imu3_pose;
//ros::Publisher PosePublisher::imu4_pose;
//ros::Publisher PosePublisher::imu5_pose;
//ros::Publisher PosePublisher::imu6_pose;
//ros::Publisher PosePublisher::imu7_pose;
ros::Time PosePublisher::last_time_;
ImuFilter PosePublisher::filter_;
bool PosePublisher::initialized;
bool PosePublisher::stateless;
int main(int argc, char** argv) {
    ros::init(argc, argv, "IMU_Pose_Estimator");
    // Declaring the ROS Handle in main and passing the pointer to it around to other classes to use it
    ros::NodeHandle nhp;
    PosePublisher::nh = &nhp;
    PosePublisher::initialized = false;
    PosePublisher::stateless = false;
    PosePublisher::imu0_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu0_pose", 10);
    PosePublisher::init();
    return 0;
}
