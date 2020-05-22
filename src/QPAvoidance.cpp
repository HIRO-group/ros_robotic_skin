#include "QPAvoidance.h"

#define CGAL_QP_NO_ASSERTIONS

QPAvoidance::QPAvoidance(/* args */)
{
    jointVelocityLimitsMin << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
    jointVelocityLimitsMax << +2.1750, +2.1750, +2.1750, +2.1750, +2.6100, +2.6100, +2.6100;
    qp = CGAL::Quadratic_program<double>(CGAL::SMALLER, false, 0, false, 0);
}

QPAvoidance::~QPAvoidance()
{
}

void QPAvoidance::EigenSetA(Eigen::MatrixXd A)
{
    A.transposeInPlace();
    for(int i=0; i < A.rows(); i++)
    {
        for(int j=0; j<A.cols(); j++)
            qp.set_a(i, j,  A(i,j));
    }
}

void QPAvoidance::EigenSetB(Eigen::VectorXd v)
{
    for(int i=0; i < v.size(); i++)
    {
        qp.set_b(i, v(i));
    }
}

void QPAvoidance::EigenSetH(Eigen::MatrixXd H)
{
    H.transposeInPlace();
    for(int i=0; i < H.rows(); i++)
    {
        for(int j=0; j<H.cols(); j++)
        {
            if (H(i,j) < 1e-10) qp.set_d(i, j, 0.0);
            else qp.set_d(i, j, H(i,j));
        }
    }
}

void QPAvoidance::EigenSetf(Eigen::VectorXd f)
{
    for(int i=0; i<f.size(); i++)
    {
        if (f(i) < 1e-10) qp.set_c(i, 0.0);
        else qp.set_c(i, f(i));
    }
}

void QPAvoidance::EigenSetL(Eigen::VectorXd l)
{
    for(int i=0; i<l.size(); i++)
        qp.set_l(i, true, l(i));
}

void QPAvoidance::EigenSetU(Eigen::VectorXd u)
{
    for(int i=0; i<u.size(); i++)
        qp.set_u(i, true, u(i));
}



double QPAvoidance::computeDampingFactor(double omega)
{
    if (omega >= omega0)
    {
        return 0.0;
    }
    else
    {
        return dampingFactor0 * std::pow((1 - omega/omega0),2);
    }
}

Eigen::VectorXd QPAvoidance::computeJointVelocities(Eigen::VectorXd q, Eigen::Vector3d xDot)
{
    Eigen::VectorXd qDot(7);

    Eigen::MatrixXd J = kdlSolver.computeJacobian(std::string ("end_effector"), q); J = J.block(0,0,3,7);
    Eigen::MatrixXd Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd qGroundTruth = Jpinv * xDot ;

    Eigen::MatrixXd H = J.transpose() * J; //- computeDampingFactor(std::sqrt((J*J.transpose()).determinant())) * Eigen::MatrixXd::Identity(7,7);
    Eigen::VectorXd f = - xDot.transpose() * J;




    EigenSetH(H);
    EigenSetf(f);
    EigenSetL(jointVelocityLimitsMin);
    EigenSetU(jointVelocityLimitsMax);
    // EigenSetA(A);
    // EigenSetB(b);

    solution = CGAL::solve_quadratic_program(qp, ET());
    assert (solution.solves_quadratic_program(qp));

    int i = 0;
    for (CGAL::Quadratic_program_solution<ET>::Variable_value_iterator it = solution.variable_values_begin(); it < solution.variable_values_end(); ++it)
    {
        qDot(i++) = CGAL::to_double(*it);
    }

    std::cout << solution;
    // std::cout << "Ground truth:\n"  << qGroundTruth << std::endl;
    // std::cout << "Quadratic programming output:\n"  << qDot << std::endl;

    return qDot;
}

// Checked simple problem
// Check that qGroundTruth moves the robot in the right direction
// Error in defining H\

/*
Observation problems with very small and big numbers!
    H = 1e+3 * Eigen::MatrixXd::Identity(2,2); This makes sense but result is 4.66597e-310, 4.66597e-310, 7.74652e-315. limits for double are 5.0e-345 to 1.7e308.
    f = Eigen::VectorXd::Constant(2, 0.0);
*/
