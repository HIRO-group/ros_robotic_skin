#include "KDLSolver.h"
#include "kdl/segment.hpp"
#include "kdl/frames.hpp"

KDLSolver::KDLSolver()
{
    n.param("robot_description", robot_desc_string, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string, kdlTree))
        ROS_ERROR("Failed to construct kdl tree");


    for (KDL::SegmentMap::const_iterator seg = kdlTree.getSegments().begin(); seg != kdlTree.getSegments().end(); seg++)
    {
        if (seg->first.find("control_point") != std::string::npos)
        {
            controlPointCount++;
        }
    }

    kdlChainsControlPoints = std::make_unique<KDL::Chain[]>(controlPointCount + 1);
    kdlChainsJoints = std::make_unique<KDL::Chain[]>(10); // 0 through 8 + ee
    kdlTree.getChain("panda_link0", "end_effector", kdlChainsControlPoints[0]);
    for (int i = 0; i < controlPointCount; i++)
    {
        kdlTree.getChain("panda_link0", std::string("control_point") + std::to_string(i), kdlChainsControlPoints[i+1]);
    }
    for (int i = 0; i < 9; i++)
    {
        kdlTree.getChain("panda_link0", std::string("panda_link") + std::to_string(i), kdlChainsJoints[i]);
    }
    kdlTree.getChain("panda_link0", std::string("end_effector"), kdlChainsJoints[9]);

}


Eigen::MatrixXd KDLSolver::computeJacobian(std::string controlPointName, Eigen::VectorXd q)
{
    // Read the element in the KDL chain array that needs to be indexed
    int index{0};
    if (controlPointName == "end_effector")
    {
        index = 0;
    }
    else
    {
        index = std::stoi(controlPointName.substr(controlPointName.find_first_of("0123456789"), controlPointName.length() - 1)) + 1;
    }

    int number_joints = kdlChainsControlPoints[index].getNrOfJoints();

    KDL::ChainJntToJacSolver JSolver = KDL::ChainJntToJacSolver(kdlChainsControlPoints[index]);
    KDL::Jacobian J; J.resize(number_joints);
    KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(kdlChainsControlPoints[index].getNrOfJoints());
    JSolver.JntToJac(KDLJointArray, J);
    return J.data;
}

Eigen::MatrixXd KDLSolver::computeJacobian2(std::string linkName, Eigen::VectorXd q, double t, double nrm)
{
    KDL::Chain kdlChain;
    kdlTree.getChain("panda_link0", linkName, kdlChain);
    KDL::Frame newFrame(KDL::Vector(0, 0, - (1-t) * nrm));

    kdlChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), newFrame, KDL::RigidBodyInertia::Zero()));
    int number_joints = kdlChain.getNrOfJoints();
    KDL::ChainJntToJacSolver JSolver = KDL::ChainJntToJacSolver(kdlChain);
    KDL::Jacobian J; J.resize(number_joints);
    KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(number_joints);
    JSolver.JntToJac(KDLJointArray, J);
    return J.data;
}

Eigen::Vector3d KDLSolver::forwardKinematicsControlPoints(std::string controlPointName, Eigen::VectorXd q)
{
    int index{0};
    if (controlPointName == "end_effector")
    {
        index = 0;
    }
    else
    {
        index = std::stoi(controlPointName.substr(controlPointName.find_first_of("0123456789"), controlPointName.length() - 1)) + 1;
    }

    KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChainsControlPoints[index]);
    KDL::Frame controlPointFrame;
    KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(kdlChainsControlPoints[index].getNrOfJoints());
    FKSolver.JntToCart(KDLJointArray, controlPointFrame);
    Eigen::Vector3d controlPointPositionVector;
    controlPointPositionVector << controlPointFrame.p(0), controlPointFrame.p(1), controlPointFrame.p(2);
    return controlPointPositionVector;
}

Eigen::MatrixXd KDLSolver::forwardKinematicsJoints(const Eigen::VectorXd & q)
{
    std::vector<int> jointNumbers = {1, 3, 4, 5, 7, 9};
    Eigen::MatrixXd result(3,jointNumbers.size());
    int index = 0;
    for (auto &&i : jointNumbers)
    {
        KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChainsJoints[i]);
        KDL::Frame kdlFrame;
        KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(kdlChainsJoints[i].getNrOfJoints());
        FKSolver.JntToCart(KDLJointArray, kdlFrame);
        result.col(index++) << kdlFrame.p(0), kdlFrame.p(1), kdlFrame.p(2);
    }

    return result;
}

int KDLSolver::getNumberControlPoints()
{
    return controlPointCount;
}