#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>
#include<kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarrayvel.hpp>
#include <ros/console.h>
#include <kdl/tree.hpp>

#include "ros/ros.h"
#include "ros_robotic_skin/getJacobian.h"



// https://medium.com/@sarvagya.vaish/forward-kinematics-using-orocos-kdl-da7035f9c8e
// http://mirror.umd.edu/roswiki/pr2_mechanism(2f)Tutorials(2f)Coding(20)a(20)realtime(20)Cartesian(20)controller(20)with(20)KDL.html

class TreeHelper{

public:
    std::string robot_desc_string;
    KDL::Tree kdlTree;
    // Get KDL::Tree from parameter server
    TreeHelper(ros::NodeHandle &n){
        n.param("robot_description", robot_desc_string, std::string());
        get_kdl_string();

    }

    bool get_kdl_string(){
        if (!kdl_parser::treeFromString(robot_desc_string, kdlTree)){
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        return true;
    }

    bool compute_jacobian(ros_robotic_skin::getJacobian::Request& req,
                      ros_robotic_skin::getJacobian::Response& res){
        // Select the end effector and get the chain from base (panda_link0) to selected end effector
        KDL::Chain kdlChain;
        kdlTree.getChain("panda_link0", req.end_effector_name, kdlChain);
        int number_joints = kdlChain.getNrOfJoints();

        // Current joint positions
        KDL::JntArray jointAngles = KDL::JntArray(number_joints);
        for (int i = 0; i < number_joints; i++)
            jointAngles(i) = req.joint_states[i+2];

        // Compute jacobian
        KDL::ChainJntToJacSolver JSolver = KDL::ChainJntToJacSolver(kdlChain); //OUT
        KDL::Jacobian  J;
        J.resize(number_joints);
        JSolver.JntToJac(jointAngles, J);

        // Get jacobian in list form
        std::vector <double> J_vector;
        for (unsigned int i = 0 ; i < 6 ; i++)
        {
            for (unsigned int j = 0 ; j < number_joints ; j++)
                J_vector.push_back(J(i,j));
        }

        res.J.end_effector_name = req.end_effector_name;
        res.J.rows = 6;
        res.J.columns = number_joints;
        res.J.J = J_vector;

        return true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "KDL_server");
    ros::NodeHandle n;
    TreeHelper my_tree(n);

    ros::ServiceServer service = n.advertiseService("get_jacobian", &TreeHelper::compute_jacobian, &my_tree);
    ROS_INFO("Ready to compute Jacobian");
    ros::spin();

    return 0;

    // CODE CURRENTLY NOT IN USE

    // KDL::ChainFkSolverPos_recursive FKSolver =
    //     KDL::ChainFkSolverPos_recursive(kdlChain);
    // KDL::Frame eeFrame;
    // FKSolver.JntToCart(jointAngles, eeFrame);

    // double d[16];
    // eeFrame.Make4x4(d);

    // std::cout <<"-------------------------------"<< std::endl;

    // KDL::Twist xdot;
    // KDL::JntArrayVel qdot = KDL::JntArrayVel(kdlChain.getNrOfJoints());
    // qdot.qdot(0) = 0.1;
    // qdot.qdot(1) = 0.1;
    // qdot.qdot(2) = 0.1;
    // qdot.qdot(3) = 0.1;
    // qdot.qdot(4) = 0.1;
    // qdot.qdot(5) = 0.1;
    // qdot.qdot(6) = 0.1;

    // for (unsigned int i = 0 ; i < 6 ; i++)
    // {
    //     xdot(i) = 0;
    //     for (unsigned int j = 0 ; j < kdlChain.getNrOfJoints() ; j++)
    //     {
    //         xdot(i) += J(i,j) * qdot.qdot(j);
    //     }
    //     std::cout << xdot(i) << std::endl;
    // }
}