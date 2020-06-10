#include "KDLSolver.h"

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

    this->kdlChains = std::make_unique<KDL::Chain[]>(controlPointCount + 1);
    kdlTree.getChain("panda_link0", "end_effector", kdlChains[0]);
    for (int i = 0; i < controlPointCount; i++)
    {
        kdlTree.getChain("panda_link0", std::string("control_point") + std::to_string(i), kdlChains[i+1]);
    }
}

KDLSolver::~KDLSolver()
{
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
        index = std::stoi(controlPointName.substr(controlPointName.find_first_of("0123456789"), controlPointName.length() - 1));
    }

    int number_joints = kdlChains[index].getNrOfJoints();

    KDL::ChainJntToJacSolver JSolver = KDL::ChainJntToJacSolver(kdlChains[index]);
    KDL::Jacobian J; J.resize(number_joints);
    KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(kdlChains[index].getNrOfJoints());
    JSolver.JntToJac(KDLJointArray, J);
    return J.data;
}

Eigen::Vector3d KDLSolver::forwardKinematics(std::string controlPointName, Eigen::VectorXd q)
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

    KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChains[index]);
    KDL::Frame controlPointFrame;
    KDL::JntArray KDLJointArray(7); KDLJointArray.data = q; KDLJointArray.resize(kdlChains[index].getNrOfJoints());
    FKSolver.JntToCart(KDLJointArray, controlPointFrame);
    Eigen::Vector3d controlPointPositionVector;
    controlPointPositionVector << controlPointFrame.p(0), controlPointFrame.p(1), controlPointFrame.p(2);
    return controlPointPositionVector;
}

int KDLSolver::getNumberControlPoints()
{
    return controlPointCount;
}