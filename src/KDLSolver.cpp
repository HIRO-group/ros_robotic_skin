#include "KDLSolver.h"

KDLSolver::KDLSolver() {
    n.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, kdlTree))
        ROS_ERROR("Failed to construct kdl tree");
}

KDLSolver::~KDLSolver() {
}

Eigen::MatrixXd KDLSolver::computeJacobian(std::string controlPointName, Eigen::VectorXd q) {
    KDL::Chain kdlChain;
    kdlTree.getChain("panda_link0", controlPointName, kdlChain);
    int number_joints = kdlChain.getNrOfJoints();

    KDL::ChainJntToJacSolver JSolver = KDL::ChainJntToJacSolver(kdlChain);
    KDL::Jacobian J; J.resize(number_joints);
    KDL::JntArray KDLJointArray(7); KDLJointArray.data = q;
    JSolver.JntToJac(KDLJointArray, J);
    return J.data;
}
