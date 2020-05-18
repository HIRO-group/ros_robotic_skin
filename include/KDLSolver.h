#ifndef KDL_SOLVER_H
#define KDL_SOLVER_H

#include <string>
#include "ros/ros.h"
#include "Eigen/Dense"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/jacobian.hpp"
#include <kdl/chainjnttojacsolver.hpp>
#include "ros_robotic_skin/getJacobian.h"

class KDLSolver
{
private:
    ros::NodeHandle n;
    std::string robot_desc_string;
    KDL::Tree kdlTree;

public:
    KDLSolver();
    ~KDLSolver();
    Eigen::MatrixXd computeJacobian(std::string controlPointName, Eigen::VectorXd q);
};

#endif // KDL_SOLVER_H