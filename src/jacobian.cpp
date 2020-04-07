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



// https://medium.com/@sarvagya.vaish/forward-kinematics-using-orocos-kdl-da7035f9c8e
// http://mirror.umd.edu/roswiki/pr2_mechanism(2f)Tutorials(2f)Coding(20)a(20)realtime(20)Cartesian(20)controller(20)with(20)KDL.html


int main(int argc, char** argv)
{
    KDL::Chain kdlChain;
    KDL::Tree kdlTree;
    std::string filename = "/home/ander/catkin_ws/src/ros_robotic_skin/panda_arm_hand.urdf";
    if (!kdl_parser::treeFromFile(filename, kdlTree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }

    kdlTree.getChain("panda_link0", "panda_link8", kdlChain);

    KDL::JntArray jointAngles = KDL::JntArray(kdlChain.getNrOfJoints());
    jointAngles(0) = 0;
    jointAngles(1) = 0;
    jointAngles(2) = 0;
    jointAngles(3) = 0;
    jointAngles(4) = 0;
    jointAngles(5) = 0;
    jointAngles(6) = 0;


    KDL::ChainFkSolverPos_recursive FKSolver =
        KDL::ChainFkSolverPos_recursive(kdlChain);
    KDL::Frame eeFrame;
    FKSolver.JntToCart(jointAngles, eeFrame);

    double d[16];
    eeFrame.Make4x4(d);

    std::cout <<"-------------------------------"<< std::endl;

    KDL::ChainJntToJacSolver JSolver = KDL::ChainJntToJacSolver(kdlChain);
    KDL::Jacobian  J;
    J.resize(kdlChain.getNrOfJoints());
    JSolver.JntToJac(jointAngles, J);


    KDL::Twist xdot;
    KDL::JntArrayVel qdot = KDL::JntArrayVel(kdlChain.getNrOfJoints());
    qdot.qdot(0) = 0.1;
    qdot.qdot(1) = 0.1;
    qdot.qdot(2) = 0.1;
    qdot.qdot(3) = 0.1;
    qdot.qdot(4) = 0.1;
    qdot.qdot(5) = 0.1;
    qdot.qdot(6) = 0.1;

    std::vector <float> vector;

    for (unsigned int i = 0 ; i < 6 ; i++)
    {
        xdot(i) = 0;
        for (unsigned int j = 0 ; j < kdlChain.getNrOfJoints() ; j++)
        {
            xdot(i) += J(i,j) * qdot.qdot(j);
            vector.push_back(J(i,j));
        }
        std::cout << xdot(i) << std::endl;
    }

    double arr[vector.size()];
    for (int i = 0; i < vector.size(); i++)
    {
        arr[i] = vector[i];
    }
}