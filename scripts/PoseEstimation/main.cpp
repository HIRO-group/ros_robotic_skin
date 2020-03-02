#include "PoseEstimationPublisher.h"


ros::NodeHandle* PosePublisher::nh;
ros::Publisher PosePublisher::imu_pose;
ros::Time PosePublisher::last_time;
ImuFilter PosePublisher::filter_;
bool PosePublisher::initialized;
bool PosePublisher::stateless;
std::string PosePublisher::imu_pose_publisher_name;
std::string PosePublisher::original_imu_publisher_name;
int main(int argc, char** argv) {
    std::string node_name(argv[1]);
    ros::init(argc, argv, node_name);

    // Declaring the ROS Handle in main and passing the pointer to it around to other classes to use it
    ros::NodeHandle nhp;
    ros::NodeHandle nh_private("~");
    if(!nh_private.getParam("imu_pose_publisher_name", PosePublisher::imu_pose_publisher_name)){
        std::cout << "IMU Pose publisher name not given" << std::endl;
        exit(-1);
    }
    if(!nh_private.getParam("original_imu_publisher_name", PosePublisher::original_imu_publisher_name)){
        std::cout << "Original IMU publisher name not given" << std::endl;
        exit(-1);
    }
    PosePublisher::nh = &nhp;
    PosePublisher::initialized = false;
    PosePublisher::stateless = false;
    PosePublisher::imu_pose = PosePublisher::nh->advertise<geometry_msgs::Quaternion>(PosePublisher::imu_pose_publisher_name, 10);
    PosePublisher::init();
    return 0;
}
