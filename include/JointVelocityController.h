#ifndef JOINT_VELOCITY_CONTROLLER_H
#define JOINT_VELOCITY_CONTROLLER_H

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "controller_manager_msgs/SwitchController.h"
#include "Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"


class JointVelocityController
{
    private:
        std::vector<ros::Publisher> publishers;
        ros::Publisher realPublisher;
        std_msgs::Float64 msg;
        std_msgs::Float64MultiArray msgarray;

 public:
    ros::NodeHandle n;
    JointVelocityController();
    ~JointVelocityController();
    void sendVelocities(const Eigen::VectorXd vel);

};

#endif // JOINT_VELOCITY_CONTROLLER_H
