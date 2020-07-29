// Link to Ding Paper Presentation:
// https://docs.google.com/presentation/d/1LrW7mna1wRgHsIzw3wXOrvIg3xlkNpIfmVRfGyxG_v0/edit?usp=sharing

// Link to Overleaf file with the math in the presentation
// https://www.overleaf.com/read/hwndqxxqtvds

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
        ros::shutdown();
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


Eigen::VectorXd QPAvoidance::computeJointVelocities(Eigen::VectorXd& q, Eigen::Vector3d& xDot,
                                                    std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                                    std::vector<KDLSolver::closest_point>& closestPoints)
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
        for (int i = 0; i < m; i++)
        {
            w[i] = 1 / closestPoints[i].distance_to_obs;
            b[i] = computebvalue(closestPoints[i].distance_to_obs); // From fig. 5
            Ji = kdlSolver.computeJacobian2(closestPoints[i], q);
            JiResized.block(0, 0, 3, Ji.cols()) = Ji.block(0, 0, 3, Ji.cols());
            A.row(i) = (obstaclePositionVectors[i] - closestPoints[i].control_point).normalized().transpose() * JiResized;
            C.row(i) = gradientOfDistanceNorm(obstaclePositionVectors[i], closestPoints[i], q);
        }
        // A.conservativeResize(m, A.cols());
        // b.conservativeResize(m);
        A.row(m) = - w.transpose() * C;
        b(m) = 0;
    }
    algLib(H, f, A, b, bl, bu);
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
        return 5000.0;
    }
}
