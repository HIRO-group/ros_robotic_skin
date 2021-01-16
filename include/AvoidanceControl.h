#ifndef AVOIDANCE_CONTROL_H
#define AVOIDANCE_CONTROL_H

#include <iostream>
#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Bool.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_listener.h"
#include "Eigen/Dense"
#include "KDLSolver.h"
#include "JointVelocityController.h"

using namespace std;


class AvoidanceControl
{
    private:
        bool isSim;

        ros::NodeHandle nh;
        ros::Rate rate{100};

        sensor_msgs::Range distance;
        sensor_msgs::Range publishDistance;
        ros::Subscriber distanceSubscriber;
        ros::Subscriber jointStateSubscriber;
        ros::Publisher distancePublisher;
        ros::Publisher startPublisher;
        JointVelocityController jointVelocityController;

        tf::StampedTransform transform;
        tf::TransformListener transform_listener;
        KDLSolver kdlSolver;

        Eigen::Vector3d endEffectorPositionVector, positionErrorVector, desiredEEVelocity;
        Eigen::VectorXd q, jointMiddleValues{7}, jointRanges{7};

        double positionErrorThreshold{0.01}, secondaryTaskGain{5.0};

        void distanceCallBack(const sensor_msgs::Range::ConstPtr& msg);
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
        void readEndEffectorPosition();
        Eigen::VectorXd EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity);

        const double desiredVelocity = 0.25;
        const double maxRepulsiveVelocity = 1.0;
        bool atStartingPoint = false;

        Eigen::Vector3d desiredPosition;
        Eigen::Vector3d startPosition;

    public:
        AvoidanceControl(bool isSim);
        ~AvoidanceControl();
        void moveToPosition(const Eigen::Vector3d desiredPositionVector);
        void moveToStart();
        void moveToGoal();
};

#endif // AVOIDANCE_CONTROL_H
