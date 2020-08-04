// Link to Ding Paper Presentation:
// https://docs.google.com/presentation/d/1LrW7mna1wRgHsIzw3wXOrvIg3xlkNpIfmVRfGyxG_v0/edit?usp=sharing

// Link to Overleaf file with the math in the presentation
// https://www.overleaf.com/read/hwndqxxqtvds

#include "QPAvoidance.h"

Eigen::VectorXd QPAvoidance::algLib(Eigen::MatrixXd H, Eigen::VectorXd f, Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::VectorXd bl, Eigen::VectorXd bu)
{
    bool successfulOptimization = false;
    bool numberOfAttemptsExceeded = false;
    while (!successfulOptimization)
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
            if (ALGLIBrep.terminationtype < 0)
            {
                if (numberOfAttemptsExceeded)
                {
                    std::cout << "-------------------------------" << std::endl;
                    ROS_WARN("Error in the optimization process");
                    ROS_WARN("Optimization exit code: %s", std::to_string(ALGLIBrep.terminationtype).c_str());
                    ROS_WARN("Number of attemps exceeded");
                    ROS_WARN("Returning zero velocities");
                    qDot = Eigen::VectorXd::Zero(7);
                    return qDot;
                }
                else
                {
                    successfulOptimization = false;
                    numberOfAttemptsExceeded = true;
                    std::cout << "-------------------------------" << std::endl;
                    ROS_WARN("Error in the optimization process");
                    ROS_WARN("Optimization exit code: %s", std::to_string(ALGLIBrep.terminationtype).c_str());
                    ROS_WARN("Trying easier constraints. Repulsive actions set to 0.");
                    for (int i = 0; i < b.size(); i++)
                    {
                        if (b(i) < 0)
                        {
                            b(i) = 0;
                        }
                    }
                }
            }
            else
            {
                successfulOptimization = true;
                for (int i = 0; i < ALGLIBqDot.length(); i++)
                {
                    qDot(i) = ALGLIBqDot(i);
                }
            }

        }
        catch(alglib::ap_error e)
        {
            ROS_WARN("Alglib exception. Shutting down...");
            printf("error msg: %s\n", e.msg.c_str());
            printf("A: \n");
            std::cout << A << std::endl;
            printf("b: \n");
            std::cout << b << std::endl;
            ros::shutdown();
        }
    }
    return qDot;
}

QPAvoidance::QPAvoidance()
{
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


Eigen::VectorXd QPAvoidance::computeJointVelocities(Eigen::VectorXd& q, Eigen::Vector3d& xDot,
                                                    std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                                    std::vector<KDLSolver::closest_point>& closestPoints,
                                                    ros::Rate& r)
{
    // Equation #3
    Eigen::VectorXd jointLimitsMin{7}, jointLimitsMax{7}, jointVelocityMax{7}, jointAccelerationMax{7};
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
    jointVelocityMax << 2.1750, 2.1750, 2.1750 , 2.1750, 2.6100 , 2.6100 , 2.6100;
    jointAccelerationMax << 15, 7.5, 10, 12.5, 15, 20, 20;
    jointLimitsMax = jointLimitsMin * 0.97;
    jointLimitsMin = jointLimitsMax * 0.97;
    jointVelocityMax = jointVelocityMax * 0.98;
    jointAccelerationMax = jointAccelerationMax * 0.01;
    Eigen::Vector3d candidates;
    Eigen::VectorXd bl{jointLimitsMax.size()}, bu{jointLimitsMax.size()};
    for (int i = 0; i < jointLimitsMax.size(); i++)
    {
        candidates(0) = (jointLimitsMin(i) - q(i)) / r.expectedCycleTime().toSec();
        candidates(1) = - jointVelocityMax(i);
        candidates(2) = - std::sqrt(2 * jointAccelerationMax(i) * (q(i) - jointLimitsMin(i)));
        bl(i) = candidates.maxCoeff();

        candidates(0) = (jointLimitsMax(i) - q(i)) / r.expectedCycleTime().toSec();
        candidates(1) = + jointVelocityMax(i);
        candidates(2) = + std::sqrt(2 * jointAccelerationMax(i) * (q(i) - jointLimitsMin(i)));
        bu(i) = candidates.minCoeff();
    }

    Eigen::MatrixXd J = kdlSolver.computeJacobian(std::string ("panda_EE"), q).block(0,0,3,7);
    Eigen::MatrixXd Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::MatrixXd qGroundTruth = Jpinv * xDot ;

    Eigen::MatrixXd H = J.transpose() * J + computeDampingFactor(std::sqrt((J*J.transpose()).determinant())) * Eigen::MatrixXd::Identity(7,7);
    Eigen::VectorXd f = - xDot.transpose() * J;

    Eigen::MatrixXd A, newA;
    Eigen::VectorXd b, newb;
    if (obstaclePositionVectors.size() == 0)
    {
        A = Eigen::MatrixXd(0,0);
        b = Eigen::VectorXd(0);
    }
    else
    {
        // Initialize variables
        // Being m the number of obstacle points detected:
        int m = obstaclePositionVectors.size();
        A = Eigen::MatrixXd(m + 1, 7);
        b = Eigen::VectorXd(m + 1);
        Eigen::VectorXd w = Eigen::VectorXd::Ones(m);
        Eigen::MatrixXd C(m, 7), Ji, JiResized = Eigen::MatrixXd::Zero(3, 7);

        // 1) select the control point closes to each obstacle
        // 2) Calculate the distance d from the obstacle point to chosen control point
        // 3) Get Jacobian to selected control point. This is changing is size dependent on what control point we have selected
        // 4) Alter the size of the Jacobian so that it's always 7x3
        // 5) Add extra row from equation 5
        // 6) d norm calculation done in Fig. 5
        // 7) Take the gradient os the norm distance with
        int numberOfRestrictions = 0;
        for (int i = 0; i < m; i++)
        {
            w[i] = 1 / closestPoints[i].distance_to_obs;
            b[i] = computebvalue(closestPoints[i].distance_to_obs); // From fig. 5
            if (!std::isnan(b(i)))
            {
                numberOfRestrictions++;
            }
            Ji = kdlSolver.computeJacobian2(closestPoints[i], q);
            JiResized.block(0, 0, 3, Ji.cols()) = Ji.block(0, 0, 3, Ji.cols());
            A.row(i) = (obstaclePositionVectors[i] - closestPoints[i].control_point).normalized().transpose() * JiResized;
            C.row(i) = gradientOfDistanceNorm(obstaclePositionVectors[i], closestPoints[i], q);
        }
        // A.conservativeResize(m, A.cols());
        // b.conservativeResize(m);
        newA.resize(numberOfRestrictions + 1, 7);
        newb.resize(numberOfRestrictions + 1);
        for (int i = 0, j = 0; i < m; i++)
        {
            if (!std::isnan(b(i)))
            {
                newA.row(j) = A.row(i);
                newb(j) = b(i);
                j++;
            }
        }
        newA.row(numberOfRestrictions) = - w.transpose() * C;
        newb(numberOfRestrictions) = 0;
    }
    algLib(H, f, newA, newb, bl, bu);
    return qDot;
}

Eigen::VectorXd QPAvoidance::gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, KDLSolver::closest_point closestPoint, Eigen::VectorXd q)
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
        result[i] = ((obstaclePositionVector - kdlSolver.forwardKinematics(closestPoint, qplus)).norm() -
             (obstaclePositionVector - kdlSolver.forwardKinematics(closestPoint, qminus)).norm()) / (2*h);
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
        return NAN;
    }
}
