#include <iostream>
#include <math.h>
#include <cassert>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/Gmpzf.h>
#include "Eigen/Dense"
#include "KDLSolver.h"

typedef CGAL::Gmpzf ET;

class QPAvoidance
{
private:
    KDLSolver kdlSolver;
    CGAL::Quadratic_program<double> qp;
    CGAL::Quadratic_program_solution<ET> solution;
    void EigenSetA(Eigen::MatrixXd A);
    void EigenSetB(Eigen::VectorXd v);
    void EigenSetL(Eigen::VectorXd l);
    void EigenSetU(Eigen::VectorXd u);
    void EigenSetH(Eigen::MatrixXd D);
    void EigenSetf(Eigen::VectorXd c);
    double dampingFactor0{0.1}, omega0{0.001};

    double computeDampingFactor(double omega);

    Eigen::VectorXd jointVelocityLimitsMin{7};
    Eigen::VectorXd jointVelocityLimitsMax{7};

public:
    QPAvoidance(/* args */);
    ~QPAvoidance();
    Eigen::VectorXd computeJointVelocities(Eigen::VectorXd q, Eigen::Vector3d xDot);
};