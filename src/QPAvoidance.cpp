#include "QPAvoidance.h"

#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"

using namespace alglib;

void QPAvoidance::algLib()
{
        //
    // This example demonstrates minimization of F(x0,x1) = x0^2 + x1^2 -6*x0 - 4*x1
    // subject to linear constraint x0+x1<=2
    //
    // Exact solution is [x0,x1] = [1.5,0.5]
    //
    // IMPORTANT: this solver minimizes  following  function:
    //     f(x) = 0.5*x'*A*x + b'*x.
    // Note that quadratic term has 0.5 before it. So if you want to minimize
    // quadratic function, you should rewrite it in such way that quadratic term
    // is multiplied by 0.5 too.
    // For example, our function is f(x)=x0^2+x1^2+..., but we rewrite it as
    //     f(x) = 0.5*(2*x0^2+2*x1^2) + ....
    // and pass diag(2,2) as quadratic term - NOT diag(1,1)!
    //


    real_2d_array H = "[[0.207246189049000,	-1.33746751456000e-07,	0.208829979446000,	-1.74513762106000e-06,	0.0973377820450000,	-2.47168268691000e-06,	-6.31777130540000e-18], [-1.33746751456000e-07,	0.211340320462246,	-8.53101499434000e-08,	-0.163941879513274,	-2.89308655825200e-06,	0.0121817129226451,	-4.20621513539400e-22], [0.208829979446000,	-8.53101499434000e-08,	0.210425873284544,	-2.02301987638900e-06,	0.0980816444250112,	-2.65074641903000e-06,	-6.36605217023341e-18], [-1.74513762106000e-06,	-0.163941879513274,	-2.02301987638900e-06,	0.223205778760695,	3.21279127018000e-06,	0.0597683748925631,	-2.39781124919000e-22], [0.0973377820450000,	-2.89308655825200e-06,	0.0980816444250112,	3.21279127018000e-06,	0.0457168542980401,	1.28545249995723e-12,	-2.96728181321843e-18], [-2.47168268691000e-06,	0.0121817129226451,	-2.65074641903000e-06,	0.0597683748925631,	1.28545249995723e-12,	0.0505931706865406,	-3.97767354655250e-22], [-6.31777130540000e-18,	-4.20621513539400e-22,	-6.36605217023341e-18,	-2.39781124919000e-22,	-2.96728181321843e-18,	-3.97767354655250e-22,	1.92593337727719e-34]]";
    real_1d_array f = "[-0.227621500000000,0.195628946896000,-0.229360542042500,-0.326755083290000,-0.106913467360000,-0.114859910315000,6.939641152500000e-18]";
    real_1d_array scale = "[0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]";
    real_2d_array Ab = "[[]]";
    integer_1d_array constraintType = "[]";
    real_1d_array qDot;
    minqpstate state;
    minqpreport rep;

    // create solver, set quadratic/linear terms
    minqpcreate(H.cols(), state);
    minqpsetquadraticterm(state, H);
    minqpsetlinearterm(state, f);
    minqpsetlc(state, Ab, constraintType);

    // Set scale of the parameters.
    // It is strongly recommended that you set scale of your variables.
    // Knowing their scales is essential for evaluation of stopping criteria
    // and for preconditioning of the algorithm steps.
    // You can find more information on scaling at http://www.alglib.net/optimization/scaling.php
    //
    // NOTE: for convex problems you may try using minqpsetscaleautodiag()
    //       which automatically determines variable scales.
    minqpsetscale(state, scale);

    //
    // Solve problem with BLEIC-based QP solver.
    //
    // This solver is intended for problems with moderate (up to 50) number
    // of general linear constraints and unlimited number of box constraints.
    //
    // Default stopping criteria are used.
    //
    minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
    minqpoptimize(state);
    minqpresults(state, qDot, rep);
    printf("%s\n", qDot.tostring(1).c_str()); // EXPECTED: [1.500,0.500]

    //
    // Solve problem with DENSE-AUL solver.
    //
    // This solver is optimized for problems with up to several thousands of
    // variables and large amount of general linear constraints. Problems with
    // less than 50 general linear constraints can be efficiently solved with
    // BLEIC, problems with box-only constraints can be solved with QuickQP.
    // However, DENSE-AUL will work in any (including unconstrained) case.
    //
    // Default stopping criteria are used.
    //
    minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 5);
    minqpoptimize(state);
    minqpresults(state, qDot, rep);
    printf("%s\n", qDot.tostring(1).c_str()); // EXPECTED: [1.500,0.500]

    //
    // Solve problem with QuickQP solver.
    //
    // This solver is intended for medium and large-scale problems with box
    // constraints, and...
    //
    // ...Oops! It does not support general linear constraints, -5 returned as completion code!
    //
    minqpsetalgoquickqp(state, 0.0, 0.0, 0.0, 0, true);
    minqpoptimize(state);
    minqpresults(state, qDot, rep);
    printf("%d\n", int(rep.terminationtype)); // EXPECTED: -5
}

QPAvoidance::QPAvoidance()
{
    jointVelocityLimitsMin << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
    jointVelocityLimitsMax << +2.1750, +2.1750, +2.1750, +2.1750, +2.6100, +2.6100, +2.6100;
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
}

QPAvoidance::~QPAvoidance()
{
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


Eigen::VectorXd QPAvoidance::computeJointVelocities(Eigen::VectorXd q, Eigen::Vector3d xDot,
                                                    std::vector<Eigen::Vector3d> obstaclePositionVectors,
                                                    int numberControlPoints,
                                                    std::unique_ptr<Eigen::Vector3d[]> &controlPointPositionVectors)
{
    Eigen::VectorXd qDot(7);

    // Equation #3
    Eigen::VectorXd bl{jointLimitsMax.size()}, bu{jointLimitsMax.size()};
    for (int i = 0; i < jointLimitsMax.size(); i++)
    {
        if (q(i) <= jointLimitsMin(i))
        {
            bl(i) = 0;
            bu(i) = jointVelocityLimitsMax(i);
        }
        else if (q(i) >= jointLimitsMax(i))
        {
            bl(i) = jointVelocityLimitsMin(i);
            bu(i) = 0;
        }
        else
        {
            bl(i) = jointVelocityLimitsMin(i);
            bu(i) = jointVelocityLimitsMax(i);
        }
    }


    Eigen::MatrixXd J = kdlSolver.computeJacobian(std::string ("end_effector"), q).block(0,0,3,7);
    Eigen::MatrixXd Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd qGroundTruth = Jpinv * xDot ;

    Eigen::MatrixXd H = J.transpose() * J; - computeDampingFactor(std::sqrt((J*J.transpose()).determinant())) * Eigen::MatrixXd::Identity(7,7);
    Eigen::VectorXd f = - xDot.transpose() * J;

    //////
    obstaclePositionVectors.resize(2);
    obstaclePositionVectors[0] = (Eigen::Vector3d::Constant(0.35));
    obstaclePositionVectors[1] = (Eigen::Vector3d::Constant(0.55));
    ///////
    // Being m the number of obstacle points detected:
    // See equation #5 in ding paper for first m rows, then an additonal 1 row for equation #7
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(obstaclePositionVectors.size() + 1, 7);
    // b is from equation #8, must have the same number of rows as A
    Eigen::VectorXd b(obstaclePositionVectors.size() + 1);
    // Will set w dependent on the distance value
    Eigen::VectorXd w = Eigen::VectorXd::Ones(obstaclePositionVectors.size());
    // C comes from equation #6, intermediate value
    Eigen::MatrixXd C(obstaclePositionVectors.size(), 7), Jpc, JpcNormalized = Eigen::MatrixXd::Zero(3, 7);
    // temp variable used to hold distance
    Eigen::Vector3d d;
    // temp variable used to pick the control point closest to each of obstacle points
    std::vector<double> distancesToControlPoints, distanceNorms;
    distancesToControlPoints.resize(numberControlPoints);
    distanceNorms.resize(obstaclePositionVectors.size());

    int assignedIndex{0};
    double distanceNormsSum{0};

    // 1) select the control point closes to each obstacle
    // 2) Calculate the distance d from the obstacle point to chosen control point
    // 3) Get Jacobian to selected control point. This is changing is size dependent on what control point we have selected
    // 4) Alter the size of the Jacobian so that it's always 7x3
    // 5) Add extra row from equation 5
    // 6) d norm calculation done in Fig. 5
    // 7) Take the gradient os the norm distance with
    for (int i = 0; i < obstaclePositionVectors.size(); i++)
    {
        for (int j = 0; j < numberControlPoints; j++)
            distancesToControlPoints[j] = (obstaclePositionVectors[i] - controlPointPositionVectors[j]).norm();
        // assignedIndex = std::distance(distancesToControlPoints.begin(), std::min_element(distancesToControlPoints.begin(), distancesToControlPoints.end()));
        assignedIndex = 3;
        d = obstaclePositionVectors[i] - kdlSolver.forwardKinematics(std::string("control_point") + std::to_string(assignedIndex), q);
        distanceNorms[i] = d.norm();
        distanceNormsSum = distanceNormsSum + distanceNorms[i];
        Jpc = kdlSolver.computeJacobian(std::string("control_point") + std::to_string(assignedIndex), q);
        JpcNormalized.block(0, 0, 3, Jpc.cols()) = Jpc.block(0, 0, 3, Jpc.cols());
        A.row(i) = d.normalized().transpose() * JpcNormalized;
        b[i] = computebvalue(distanceNorms[i]); // From fig. 5
        C.row(i) = gradientOfDistanceNorm(obstaclePositionVectors[i], std::string("control_point") + std::to_string(assignedIndex), q);

        ///////
        // std::cout << assignedIndex << std::endl;
        // std::cout << Jpc.rows() << "," << Jpc.cols() << std::endl;
        ////////
    }

    // Calculate the weights to complet equation #7
    for (int i = 0; i < obstaclePositionVectors.size(); i++)
    {
        w[i] = distanceNorms[i] / distanceNormsSum;
    }
    // Last row in A, equation #7
    A.row(obstaclePositionVectors.size()) = - w.transpose() * C;
    b[obstaclePositionVectors.size()] = 0;

    //////
    // std::cout << b << std::endl;
    // std::cout << "------" << std::endl;
    /////

    // Send to Matlab H, f, A, b, bl, bu;

    return qDot;
}

Eigen::VectorXd QPAvoidance::gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, std::string controlPointName, Eigen::VectorXd q)
{
    // The value of the derivative is dependent on h, generally the smaller the better the aproximation.
    Eigen::VectorXd qplus(7), qminus(7), result(7);
    double h{0.001};
    for (int i = 0; i < 7; i++)
    {
        qplus = q;
        qminus = q;
        qplus[i] = qplus[i] + h;
        qminus[i] = qminus[i] - h;
        result[i] = ((obstaclePositionVector - kdlSolver.forwardKinematics(controlPointName, qplus)).norm() -
             (obstaclePositionVector - kdlSolver.forwardKinematics(controlPointName, qminus)).norm()) / (2*h);
    }
    return result;
}

double QPAvoidance::computebvalue(double distanceNorm)
{
    // equation #13
    double dr{0.17}, d0l{0.22}, d0u{0.25}, da{0.3}, xdota{0.2}, xdotr{-0.2};
    if (distanceNorm < dr)
    {
        return xdotr;
    }
    else if (distanceNorm < d0l)
    {
        return xdotr + (0 - xdotr) / (d0l - dr) * (distanceNorm - dr);
    }
    else if (distanceNorm < d0u)
    {
        return 0;
    }
    else if (distanceNorm < da)
    {
        return 0 + (xdota - 0) / (da - d0u) * (distanceNorm - d0u);
    }
    else
    {
        return 1000.0;
    }
}