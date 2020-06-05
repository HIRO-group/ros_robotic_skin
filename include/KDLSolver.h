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
    KDL::Chain kdlChain;

public:
    KDLSolver();
    ~KDLSolver();
    Eigen::MatrixXd computeJacobian(std::string controlPointName, Eigen::VectorXd q);
    Eigen::MatrixXd forwardKinematics(std::string controlPointName, Eigen::VectorXd q);
};

#endif // KDL_SOLVER_H