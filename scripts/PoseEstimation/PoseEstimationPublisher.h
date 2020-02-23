//
// Created by hiro on 2/22/20.
//

#ifndef POSEESTIMATION_POSEESTIMATIONPUBLISHER_H
#define POSEESTIMATION_POSEESTIMATIONPUBLISHER_H
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"

class PosePublisher {
public:
    static ros::NodeHandle* nh;
    static ros::Publisher imu0_pose;
    static ros::Publisher imu1_pose;
    static ros::Publisher imu2_pose;
    static ros::Publisher imu3_pose;
    static ros::Publisher imu4_pose;
    static ros::Publisher imu5_pose;
    static ros::Publisher imu6_pose;
    static ros::Publisher imu7_pose;
    static void init();
    static void get_imu_data();
    static void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

};


#endif //POSEESTIMATION_POSEESTIMATIONPUBLISHER_H
