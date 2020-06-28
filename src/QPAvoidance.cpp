#include "QPAvoidance.h"

Eigen::VectorXd QPAvoidance::algLib(Eigen::MatrixXd H, Eigen::VectorXd f, Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::VectorXd bl, Eigen::VectorXd bu)
{
    try
    {
        ALGLIBAb = "[[]]";
        ALGLIBconstraintType = "[]";

        for (int i = 0; i < H.rows(); i++)
        {
            for (int j = 0; j < H.cols(); j++)
            {
                ALGLIBH(i,j) = H(i,j);

            }
            ALGLIBf(i) = f(i);
            ALGLIBbl(i) = bl(i);
            ALGLIBbu(i) = bu(i);
        }

        if (A.rows() == 0)
        {
            ALGLIBAb = "[[]]";
        }
        else
        {
            ALGLIBAb.setlength(A.rows(), A.cols() + 1);
            ALGLIBconstraintType.setlength(A.rows());
            for (int i = 0; i < A.rows(); i++)
            {
                for (int j = 0; j < A.cols(); j++)
                {
                    ALGLIBAb(i,j) = A(i,j);
                }
                ALGLIBAb(i, A.cols()) = b(i);
                ALGLIBconstraintType(i) = -1;
            }
        }

        // create solver, set quadratic/linear terms
        alglib::minqpcreate(ALGLIBH.cols(), ALGLIBstate);
        alglib::minqpsetquadraticterm(ALGLIBstate, ALGLIBH);
        alglib::minqpsetlinearterm(ALGLIBstate, ALGLIBf);
        alglib::minqpsetlc(ALGLIBstate, ALGLIBAb, ALGLIBconstraintType);
        alglib::minqpsetbc(ALGLIBstate, ALGLIBbl, ALGLIBbu);

        // Set scale of the parameters.
        // It is strongly recommended that you set scale of your variables.
        // Knowing their scales is essential for evaluation of stopping criteria
        // and for preconditioning of the algorithm steps.
        // You can find more information on scaling at http://www.alglib.net/optimization/scaling.php
        //
        // NOTE: for convex problems you may try using minqpsetscaleautodiag()
        //       which automatically determines variable scales.
        alglib::minqpsetscale(ALGLIBstate, ALGLIBscale);
        // alglib::minqpsetscaleautodiag(ALGLIBstate);


        // SOLVE: 3 options: BLEIC-based, DENSE-AUL, QUICKQP

        // BLEIC-based QP solver is intended for problems with moderate (up to 50) number
        // of general linear constraints and unlimited number of box constraints.
        //
        // Default stopping criteria are used.
        //
        alglib::minqpsetalgobleic(ALGLIBstate, 0.0, 0.0, 0.0, 70);
        alglib::minqpoptimize(ALGLIBstate);
        alglib::minqpresults(ALGLIBstate, ALGLIBqDot, ALGLIBrep);
        // printf("%s\n", ALGLIBqDot.tostring(1).c_str()); // EXPECTED: [1.500,0.500]


        // DENSE-AUL solver is optimized for problems with up to several thousands of
        // variables and large amount of general linear constraints. Problems with
        // less than 50 general linear constraints can be efficiently solved with
        // BLEIC, problems with box-only constraints can be solved with QuickQP.
        // However, DENSE-AUL will work in any (including unconstrained) case.
        //
        // Default stopping criteria are used.
        //
        // alglib::minqpsetalgodenseaul(ALGLIBstate, 0.0, 1.0e+4, 15);
        // alglib::minqpoptimize(ALGLIBstate);
        // alglib::minqpresults(ALGLIBstate, ALGLIBqDot, ALGLIBrep);
        // printf("%s\n", ALGLIBqDot.tostring(1).c_str()); // EXPECTED: [1.500,0.500]
    }
    catch(alglib::ap_error e)
    {
        printf("error msg: %s\n", e.msg.c_str());
        printf("A: \n");
        std::cout << A << std::endl;
        printf("b: \n");
        std::cout << b << std::endl;

    }


    for (int i = 0; i < ALGLIBqDot.length(); i++)
    {
        qDot(i) = ALGLIBqDot(i);
    }

    return qDot;
}

QPAvoidance::QPAvoidance()
{
    jointVelocityLimitsMin << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
    jointVelocityLimitsMax << +2.1750, +2.1750, +2.1750, +2.1750, +2.6100, +2.6100, +2.6100;
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
    ALGLIBH.setlength(7, 7);
    ALGLIBf.setlength(7);
    ALGLIBbl.setlength(7);
    ALGLIBbu.setlength(7);
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

    Eigen::MatrixXd H = J.transpose() * J + computeDampingFactor(std::sqrt((J*J.transpose()).determinant())) * Eigen::MatrixXd::Identity(7,7);
    Eigen::VectorXd f = - xDot.transpose() * J;

    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    if (obstaclePositionVectors.size() == 0)
    {
        A = Eigen::MatrixXd(0,0);
        b = Eigen::VectorXd(0);
    }
    else
    {
        // Being m the number of obstacle points detected:
        // See equation #5 in ding paper for first m rows, then an additonal 1 row for equation #7
        A = Eigen::MatrixXd(obstaclePositionVectors.size() + 1, 7);
        // b is from equation #8, must have the same number of rows as A
        b = Eigen::VectorXd(obstaclePositionVectors.size() + 1);
        // Will set w dependent on the distance value
        Eigen::VectorXd w = Eigen::VectorXd::Ones(obstaclePositionVectors.size());
        Eigen::VectorXd distanceNorms{obstaclePositionVectors.size()};
        // C comes from equation #6, intermediate value
        Eigen::MatrixXd C(obstaclePositionVectors.size(), 7), Jpc, JpcNormalized = Eigen::MatrixXd::Zero(3, 7);
        // temp variable used to hold distance
        Eigen::Vector3d d;
        // temp variable used to pick the control point closest to each of obstacle points
        std::vector<double> distancesToControlPoints;
        distancesToControlPoints.resize(numberControlPoints);

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
            assignedIndex = std::distance(distancesToControlPoints.begin(), std::min_element(distancesToControlPoints.begin(), distancesToControlPoints.end()));
            d = obstaclePositionVectors[i] - kdlSolver.forwardKinematics(std::string("control_point") + std::to_string(assignedIndex), q);
            distanceNorms[i] = d.norm();
            w[i] = 1 / distanceNorms[i];
            Jpc = kdlSolver.computeJacobian(std::string("control_point") + std::to_string(assignedIndex), q);
            JpcNormalized.block(0, 0, 3, Jpc.cols()) = Jpc.block(0, 0, 3, Jpc.cols());
            A.row(i) = d.normalized().transpose() * JpcNormalized;
            b[i] = computebvalue(distanceNorms[i]); // From fig. 5
            C.row(i) = gradientOfDistanceNorm(obstaclePositionVectors[i], std::string("control_point") + std::to_string(assignedIndex), q);
        }

        // Last row in A, equation #7, if obstacles are close enough
        if (distanceNorms.minCoeff() > 0.5)
        {
            A.conservativeResize(obstaclePositionVectors.size(), 7);
            b.conservativeResize(obstaclePositionVectors.size());
        }
        else
        {
            double weightsum = w.sum();
            for (int i = 0; i < w.size(); i++)
            {
                w[i] = w[i] / weightsum;
            }
            std::cout << w << std::endl;

            A.row(obstaclePositionVectors.size()) = - w.transpose() * C;
            b[obstaclePositionVectors.size()] = 0;
        }
    }

    algLib(H, f, A, b, bl, bu);
    return qDot;
}

Eigen::VectorXd QPAvoidance::gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, std::string controlPointName, Eigen::VectorXd q)
{
    // The value of the derivative is dependent on h, generally the smaller the better the aproximation.
    Eigen::VectorXd qplus(7), qminus(7), result(7);
    double h{0.0001};
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