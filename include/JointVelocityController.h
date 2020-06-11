#ifndef JOINT_VELOCITY_CONTROLLER_H
#define JOINT_VELOCITY_CONTROLLER_H

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "controller_manager_msgs/SwitchController.h"
#include "Eigen/Dense"

class JointVelocityController
{
    private:
        ros::NodeHandle n;
        std::vector<ros::Publisher> publishers;
        std_msgs::Float64 msg;

    public:
        JointVelocityController();
        ~JointVelocityController();
        void sendVelocities(const Eigen::VectorXd vel);

};

#endif // JOINT_VELOCITY_CONTROLLER_H