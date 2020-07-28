#ifndef KDL_SOLVER_H
#define KDL_SOLVER_H

#include <string>
#include "ros/ros.h"
#include "Eigen/Dense"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"

class KDLSolver
{
private:
    ros::NodeHandle n;
    std::string robot_desc_string;
    KDL::Tree kdlTree;
    std::unique_ptr<KDL::Chain[]> kdlChainsControlPoints;
    std::unique_ptr<KDL::Chain[]> kdlChainsJoints;
    int controlPointCount{0};

public:
    KDLSolver();
    Eigen::MatrixXd computeJacobian(std::string controlPointName, Eigen::VectorXd q);
    Eigen::MatrixXd computeJacobian2(std::string controlPointName, Eigen::VectorXd q, double t, double nrm);
    Eigen::MatrixXd forwardKinematicsJoints(const Eigen::VectorXd & q);
    Eigen::Vector3d forwardKinematicsControlPoints(std::string controlPointName, Eigen::VectorXd q);
    int getNumberControlPoints();

    struct closest_point
    {
        int segmentId;
        Eigen::Vector3d segmentPointA;
        Eigen::Vector3d segmentPointB;
        double t;
        float distance_to_obs = FLT_MAX;
        Eigen::Vector3d control_point;
    };
};

#endif // KDL_SOLVER_H