#ifndef JOINT_VELOCITY_CONTROLLER_H
#define JOINT_VELOCITY_CONTROLLER_H

#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "controller_manager_msgs/SwitchController.h"
#include "Eigen/Dense"
#include "std_msgs/Float64MultiArray.h"
using namespace std;

class JointVelocityController
{
    public:
        JointVelocityController(bool isSim);
        ~JointVelocityController();
        void sendVelocities(const Eigen::VectorXd velocities);

    private:
        // variable
        bool isSim;
        // ROS instances
        ros::NodeHandle n;
        vector<ros::Publisher> velocityPublishers;
        ros::Publisher realVelocityPublisher;
        std_msgs::Float64 msg;
        std_msgs::Float64MultiArray msgarray;
        // functions
        void _switchController(
            vector<string> positionControllerNames,
            vector<string> velocityControllerNames);
        void _initializeSwitchController(bool isSim);
        void _prepareSimVelocityPublisher();
        void _prepareRealVelocityPublisher();
        void _sendSimVelocities(const Eigen::VectorXd velocities);
        void _sendRealVelocities(const Eigen::VectorXd velocities);
};

#endif // JOINT_VELOCITY_CONTROLLER_H
