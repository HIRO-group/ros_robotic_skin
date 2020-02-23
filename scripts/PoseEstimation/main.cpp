#include <cmath>
#include "PoseEstimationPublisher.h"

ros::NodeHandle* PosePublisher::nh;
ros::Publisher PosePublisher::imu0_pose;
ros::Publisher PosePublisher::imu1_pose;
ros::Publisher PosePublisher::imu2_pose;
ros::Publisher PosePublisher::imu3_pose;
ros::Publisher PosePublisher::imu4_pose;
ros::Publisher PosePublisher::imu5_pose;
ros::Publisher PosePublisher::imu6_pose;
ros::Publisher PosePublisher::imu7_pose;
int main(int argc, char** argv) {
    ros::init(argc,argv,"IMU_Pose_Estimator");
    ros::NodeHandle nhp;
    PosePublisher::nh = &nhp;
    PosePublisher::imu0_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu0_pose", 100);
    PosePublisher::imu1_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu1_pose", 100);
    PosePublisher::imu2_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu2_pose", 100);
    PosePublisher::imu3_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu3_pose", 100);
    PosePublisher::imu4_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu4_pose", 100);
    PosePublisher::imu5_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu5_pose", 100);
    PosePublisher::imu6_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu6_pose", 100);
    PosePublisher::imu7_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>("imu7_pose", 100);
    PosePublisher::init();
    return 0;
}
