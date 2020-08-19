// Link to Ding Paper Presentation:
// https://docs.google.com/presentation/d/1LrW7mna1wRgHsIzw3wXOrvIg3xlkNpIfmVRfGyxG_v0/edit?usp=sharing

// Link to Overleaf file with the math in the presentation
// https://www.overleaf.com/read/hwndqxxqtvds

#include "FlaccoAvoidance.h"

Eigen::VectorXd FlaccoAvoidance::algLib(Eigen::MatrixXd H, Eigen::VectorXd f, Eigen::MatrixXd A, Eigen::VectorXd b, Eigen::VectorXd bl, Eigen::VectorXd bu)
{
    // std::cout << "A: " << std::endl;
    // std::cout << A << std::endl;
    // std::cout << "b: "  << std::endl;
    // std::cout << b << std::endl;

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

FlaccoAvoidance::FlaccoAvoidance()
{
    ALGLIBH.setlength(7, 7);
    ALGLIBf.setlength(7);
    ALGLIBbl.setlength(7);
    ALGLIBbu.setlength(7);
}

double FlaccoAvoidance::computeDampingFactor(double omega)
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

Eigen::Vector3d FlaccoAvoidance::computeRepulsiveVector(Eigen::MatrixXd& jointPositions, std::vector<Eigen::Vector3d>& obstaclePositionVectors)
{
        // 2, 3, 4, 6, 7, 9
    int m = obstaclePositionVectors.size();

    if (m > 0)
    {
        Eigen::VectorXd endEffectorPositionVector = jointPositions.col(5);
        std::vector<Eigen::VectorXd> distancesToEndEffector(m);
        Eigen::VectorXd distancesToEndEffectorNorms(m);
        for (int i = 0; i < m; i++)
        {
            distancesToEndEffector[i] = obstaclePositionVectors[i] - endEffectorPositionVector;
            distancesToEndEffectorNorms[i] = distancesToEndEffector[i].norm();
        }

        int minIndex;
        double minDistance;
        minDistance = distancesToEndEffectorNorms.minCoeff(&minIndex);
        return distancesToEndEffector[minIndex].normalized() * 0.4 * 1 / (1 + std::exp(6*(2*minDistance/0.4 - 1)));
    }
    else
    {
        return Eigen::Vector3d::Zero();
    }
}

Eigen::MatrixXd FlaccoAvoidance::selectMostRestrictiveRestrictions(std::vector<Eigen::MatrixXd>& bodyRestrictionsCandidates)
{
    Eigen::MatrixXd bodyRestrictions(bodyRestrictionsCandidates[0].rows(), 2);

    // Select most restrictive values
    for (int i = 0; i < bodyRestrictions.rows(); i++)
    {
        Eigen::VectorXd minCandidates(bodyRestrictionsCandidates.size());
        Eigen::VectorXd maxCandidates(bodyRestrictionsCandidates.size());
        for (int j = 0; j < bodyRestrictionsCandidates.size(); j++)
        {

            minCandidates[j] = bodyRestrictionsCandidates[j](i, 0);
            maxCandidates[j] = bodyRestrictionsCandidates[j](i, 1);
        }

        // Min
        bodyRestrictions(i, 0) = minCandidates.maxCoeff();
        // Max
        bodyRestrictions(i, 1) = maxCandidates.minCoeff();
    }
    return bodyRestrictions;
}

Eigen::MatrixXd FlaccoAvoidance::computeBodyRestrictions(Eigen::MatrixXd& jointPositions,
                                                         std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                                         Eigen::VectorXd& jointVelocityMax,
                                                         Eigen::VectorXd& q)
{

    std::vector<KDLSolver::closest_point> controlPoints;
    controlPoints.resize(7);

    controlPoints[0].segmentId = 2;
    controlPoints[0].segmentPointA = jointPositions.col(0);
    controlPoints[0].segmentPointB = jointPositions.col(1);
    controlPoints[0].t = 0.33;

    controlPoints[1].segmentId = 2;
    controlPoints[1].segmentPointA = jointPositions.col(0);
    controlPoints[1].segmentPointB = jointPositions.col(1);
    controlPoints[1].t = 0.66;

    controlPoints[2].segmentId = 3;
    controlPoints[2].segmentPointA = jointPositions.col(1);
    controlPoints[2].segmentPointB = jointPositions.col(2);
    controlPoints[2].t = 0.5;

    controlPoints[3].segmentId = 4;
    controlPoints[3].segmentPointA = jointPositions.col(2);
    controlPoints[3].segmentPointB = jointPositions.col(3);
    controlPoints[3].t = 0.33;

    controlPoints[4].segmentId = 4;
    controlPoints[4].segmentPointA = jointPositions.col(2);
    controlPoints[4].segmentPointB = jointPositions.col(3);
    controlPoints[4].t = 0.66;

    controlPoints[5].segmentId = 6;
    controlPoints[5].segmentPointA = jointPositions.col(3);
    controlPoints[5].segmentPointB = jointPositions.col(4);
    controlPoints[5].t = 0.5;

    controlPoints[6].segmentId = 7;
    controlPoints[6].segmentPointA = jointPositions.col(4);
    controlPoints[6].segmentPointB = jointPositions.col(5);
    controlPoints[6].t = 0.5;

    for (int i = 0; i < controlPoints.size(); i++)
    {
        controlPoints[i].control_point = controlPoints[i].segmentPointA + controlPoints[i].t * controlPoints[i].segmentPointB;
    }


    // 2, 3, 4, 6, 7, 9

    std::vector<Eigen::MatrixXd> bodyRestrictionsCandidates(controlPoints.size());

    int m = obstaclePositionVectors.size();

    if (m > 0)
    {
        for (int controlPointId = 0; controlPointId < controlPoints.size(); controlPointId++)
        {
            std::vector<Eigen::VectorXd> distancesToControlPoint(m);
            Eigen::VectorXd distancesToControlPointNorms(m);
            for (int obstacleId = 0; obstacleId < m; obstacleId++)
            {
                distancesToControlPoint[obstacleId] = obstaclePositionVectors[obstacleId] - controlPoints[controlPointId].control_point;
                distancesToControlPointNorms[obstacleId] = distancesToControlPoint[obstacleId].norm();
            }
            int minIndex;
            double minDistance;
            minDistance = distancesToControlPointNorms.minCoeff(&minIndex);

            double f = 1 / (1 + std::exp(6*(2*minDistance/0.4 - 1)));
            Eigen::MatrixXd Jc = kdlSolver.computeJacobian2(controlPoints[controlPointId], q);
            Jc = Jc.block(0,0,3,Jc.cols());
            Eigen::MatrixXd Jcp = Jc.completeOrthogonalDecomposition().pseudoInverse();
            Eigen::VectorXd s = Jcp * distancesToControlPoint[minIndex].normalized() * f;
            // Initialize candidate to max values
            bodyRestrictionsCandidates[controlPointId].resize(jointVelocityMax.size(), 2);
            bodyRestrictionsCandidates[controlPointId].col(0) = - jointVelocityMax;
            bodyRestrictionsCandidates[controlPointId].col(1) = jointVelocityMax;
            for (int i = 0; i < s.size(); i++)
            {
                if (s[i] >= 0)
                {
                    // Edit max velocities, in the second column
                    bodyRestrictionsCandidates[controlPointId](i, 1) = (1 - f) * jointVelocityMax[i];
                }
                else
                {
                    // Edit min velocities, in the first column
                    bodyRestrictionsCandidates[controlPointId](i, 0) = (1 - f) * - jointVelocityMax[i];
                }
            }
        }

        Eigen::MatrixXd bodyRestrictions = selectMostRestrictiveRestrictions(bodyRestrictionsCandidates);
        return bodyRestrictions;
    }
    else
    {
        Eigen::MatrixXd bodyRestrictions(jointVelocityMax.size(), 2);
        bodyRestrictions.col(0) = - jointVelocityMax;
        bodyRestrictions.col(1) = jointVelocityMax;
        return bodyRestrictions;
    }
}

Eigen::VectorXd FlaccoAvoidance::computeJointVelocities(Eigen::VectorXd& q, Eigen::Vector3d& xDot,
                                                    std::vector<Eigen::Vector3d>& obstaclePositionVectors,
                                                    Eigen::MatrixXd& jointPositions,
                                                    ros::Rate& r)
{


    Eigen::Vector3d repulsiveVector = computeRepulsiveVector(jointPositions, obstaclePositionVectors);

    Eigen::MatrixXd J = kdlSolver.computeJacobian(std::string ("end_effector"), q).block(0,0,3,7);
    Eigen::MatrixXd H = J.transpose() * J + computeDampingFactor(std::sqrt((J*J.transpose()).determinant())) * Eigen::MatrixXd::Identity(7,7);
    Eigen::VectorXd f = - (xDot - repulsiveVector).transpose() * J;


    Eigen::VectorXd jointLimitsMin{7}, jointLimitsMax{7}, jointVelocityMax{7}, jointAccelerationMax{7};
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
    jointVelocityMax << 2.1750, 2.1750, 2.1750 , 2.1750, 2.6100 , 2.6100 , 2.6100;
    jointAccelerationMax << 15, 7.5, 10, 12.5, 15, 20, 20;
    jointAccelerationMax = 0.01 * jointAccelerationMax;
    Eigen::MatrixXd bodyRestrictions = computeBodyRestrictions(jointPositions, obstaclePositionVectors, jointVelocityMax, q);

    Eigen::VectorXd candidates(4);
    Eigen::VectorXd bl{jointLimitsMax.size()}, bu{jointLimitsMax.size()};
    for (int i = 0; i < jointLimitsMax.size(); i++)
    {
        candidates(0) = (jointLimitsMin(i) - q(i)) / r.expectedCycleTime().toSec();
        candidates(1) = - jointVelocityMax(i);
        candidates(2) = - std::sqrt(2 * jointAccelerationMax(i) * (q(i) - jointLimitsMin(i)));
        candidates(3) = bodyRestrictions(i, 0);
        bl(i) = candidates.maxCoeff();

        candidates(0) = (jointLimitsMax(i) - q(i)) / r.expectedCycleTime().toSec();
        candidates(1) = + jointVelocityMax(i);
        candidates(2) = + std::sqrt(2 * jointAccelerationMax(i) * (q(i) - jointLimitsMin(i)));
        candidates(3) = bodyRestrictions(i, 1);
        bu(i) = candidates.minCoeff();
    }

    algLib(H, f, Eigen::MatrixXd(0, 0), Eigen::VectorXd(0), bl, bu);
    return qDot;
}
