#ifndef POSEESTIMATION_POSEESTIMATIONPUBLISHER_H
#define POSEESTIMATION_POSEESTIMATIONPUBLISHER_H

#include <imu_filter_madgwick/imu_filter.h>
#include "imu_filter_madgwick/stateless_orientation.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include <iostream>

class PosePublisher {
    // This class is made so that everything required can be encapsulated in one class
public:
    /*Why pointer to a node handle? Because you initialize it in the main function only
     * and next pass on to other functions to use it*/
    static ros::NodeHandle* nh;
    static ros::Time last_time;

    static ImuFilter filter_;
    /*Below are static ROS Publishers*/
    static bool initialized;
    static bool stateless;
    static ros::Publisher imu_pose;
    static std::string imu_pose_publisher_name;
    static std::string original_imu_publisher_name;
    static void init();
    static void get_imu_data();
    static void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

};


#endif //POSEESTIMATION_POSEESTIMATIONPUBLISHER_H
