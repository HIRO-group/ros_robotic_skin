#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include "KDLSolver.h"

#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"
#include <math.h>


class QPAvoidance
{
private:
    KDLSolver kdlSolver;
    double dampingFactor0{0.1}, omega0{0.001};
    double computeDampingFactor(double omega);

    Eigen::VectorXd jointVelocityLimitsMin{7}, jointVelocityLimitsMax{7}, jointLimitsMin{7}, jointLimitsMax{7};

    //void computeAandbMatrices(Eigen::MatrixXd& A, Eigen::MatrixXd& b);
    Eigen::VectorXd gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, KDLSolver::closest_point closestPoint, Eigen::VectorXd q);
    double computebvalue(double distanceNorm);
    Eigen::VectorXd qDot{7};
    Eigen::VectorXd algLib(Eigen::MatrixXd H, Eigen::VectorXd f, Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::VectorXd bl, Eigen::VectorXd bu);

    // ALGLIB variables
    alglib::real_1d_array ALGLIBscale = "[1, 1, 1, 1, 1, 1, 1]";
    alglib::real_2d_array ALGLIBH = "[[0.207246189049000,	-1.33746751456000e-07,	0.208829979446000,	-1.74513762106000e-06,	0.0973377820450000,	-2.47168268691000e-06,	-6.31777130540000e-18], [-1.33746751456000e-07,	0.211340320462246,	-8.53101499434000e-08,	-0.163941879513274,	-2.89308655825200e-06,	0.0121817129226451,	-4.20621513539400e-22], [0.208829979446000,	-8.53101499434000e-08,	0.210425873284544,	-2.02301987638900e-06,	0.0980816444250112,	-2.65074641903000e-06,	-6.36605217023341e-18], [-1.74513762106000e-06,	-0.163941879513274,	-2.02301987638900e-06,	0.223205778760695,	3.21279127018000e-06,	0.0597683748925631,	-2.39781124919000e-22], [0.0973377820450000,	-2.89308655825200e-06,	0.0980816444250112,	3.21279127018000e-06,	0.0457168542980401,	1.28545249995723e-12,	-2.96728181321843e-18], [-2.47168268691000e-06,	0.0121817129226451,	-2.65074641903000e-06,	0.0597683748925631,	1.28545249995723e-12,	0.0505931706865406,	-3.97767354655250e-22], [-6.31777130540000e-18,	-4.20621513539400e-22,	-6.36605217023341e-18,	-2.39781124919000e-22,	-2.96728181321843e-18,	-3.97767354655250e-22,	1.92593337727719e-34]]";
    alglib::real_1d_array ALGLIBf = "[-0.227621500000000,0.195628946896000,-0.229360542042500,-0.326755083290000,-0.106913467360000,-0.114859910315000,6.939641152500000e-18]";
    alglib::real_2d_array ALGLIBAb = "[[]]";
    alglib::integer_1d_array ALGLIBconstraintType = "[]";
    alglib::real_1d_array ALGLIBqDot;
    alglib::minqpstate ALGLIBstate;
    alglib::minqpreport ALGLIBrep;
    alglib::real_1d_array ALGLIBbl, ALGLIBbu;


public:
    QPAvoidance();
    ~QPAvoidance();
    Eigen::VectorXd computeJointVelocities(Eigen::VectorXd& q, Eigen::Vector3d& xDot,
                                           std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                           std::vector<KDLSolver::closest_point>& closestPoints);

};