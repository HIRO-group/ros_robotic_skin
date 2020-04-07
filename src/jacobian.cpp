#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>
#include<kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarrayvel.hpp>



// https://medium.com/@sarvagya.vaish/forward-kinematics-using-orocos-kdl-da7035f9c8e
// http://mirror.umd.edu/roswiki/pr2_mechanism(2f)Tutorials(2f)Coding(20)a(20)realtime(20)Cartesian(20)controller(20)with(20)KDL.html

void print_matrix(double* d, int rows, int columns)
{
    double matrix[rows][columns];

    for (int i = 0; i < rows * columns; i++)
        {
            matrix[i/4][i%4] = d[i];
        }



    for(int x = 0; x < rows; x++)
    {
        for(int y = 0; y < columns; y++)
        {
            std::cout << std::to_string(matrix[x][y]) << "  ";  // display the current element out of the array
        }
    std::cout << std::endl;  // when the inner loop is done, go to a new line
    }
}

int main(int argc, char** argv)
{
    KDL::Chain kdlChain = KDL::Chain();

    KDL::Joint joint1(KDL::Joint::None);
    KDL::Frame frame1 = KDL::Frame(KDL::Vector(0.0, 1.0, 0.0));
    kdlChain.addSegment(KDL::Segment(joint1, frame1));

    KDL::Joint joint2(KDL::Joint::RotZ);
    KDL::Frame frame2 = KDL::Frame(KDL::Vector(0.0, 2.0, 0.0));
    kdlChain.addSegment(KDL::Segment(joint2, frame2));

    KDL::Joint joint3(KDL::Joint::RotZ);
    KDL::Frame frame3 = KDL::Frame(KDL::Rotation::EulerZYX(0.0, 0.0, -M_PI / 2)) *
                        KDL::Frame(KDL::Vector(0.0, 0.0, 2.0));
    kdlChain.addSegment(KDL::Segment(joint3, frame3));

    KDL::Joint joint4(KDL::Joint::RotZ);
    KDL::Frame frame4 = KDL::Frame(KDL::Rotation::EulerZYX(0.0, 0.0, M_PI / 2)) *
                        KDL::Frame(KDL::Vector(1.0, 1.0, 0.0));
    kdlChain.addSegment(KDL::Segment(joint4, frame4));

    KDL::JntArray jointAngles = KDL::JntArray(3);
    jointAngles(0) = -M_PI / 4.;       // Joint 1
    jointAngles(1) = M_PI / 2.;        // Joint 2
    jointAngles(2) = M_PI;             // Joint 3

    KDL::ChainFkSolverPos_recursive FKSolver =
        KDL::ChainFkSolverPos_recursive(kdlChain);
    KDL::Frame eeFrame;
    FKSolver.JntToCart(jointAngles, eeFrame);

    double d[16];
    eeFrame.Make4x4(d);

    print_matrix(d, 4, 4);

    KDL::ChainJntToJacSolver JSolver = KDL::ChainJntToJacSolver(kdlChain);
    KDL::Jacobian  J;
    J.resize(3);
    JSolver.JntToJac(jointAngles, J);


    KDL::Twist xdot;
    KDL::JntArrayVel qdot = KDL::JntArrayVel(3);
    qdot.qdot(0) = 0.1;
    qdot.qdot(1) = 0.1;
    qdot.qdot(2) = 0.1;

    for (unsigned int i = 0 ; i < 6 ; i++)
    {
        xdot(i) = 0;
        for (unsigned int j = 0 ; j < 3 ; j++)
            xdot(i) += J(i,j) * qdot.qdot(j);
    }

    for(int x = 0; x < 6; x++)
    {
        for(int y = 0; y < 3; y++)
        {
            std::cout << std::to_string(J(x,y)) << "  ";  // display the current element out of the array
        }
    std::cout << std::endl;  // when the inner loop is done, go to a new line
    }
}