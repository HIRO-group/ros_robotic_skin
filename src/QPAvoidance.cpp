#include "QPAvoidance.h"

QPAvoidance::QPAvoidance(/* args */)
{
    jointVelocityLimitsMin << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
    jointVelocityLimitsMax << +2.1750, +2.1750, +2.1750, +2.1750, +2.6100, +2.6100, +2.6100;
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
    for (int i = 0; i < obstaclePositionVectors.size(); i++)
    {
        w[i] = distanceNorms[i] / distanceNormsSum;
    }
    // Last row in A
    A.row(obstaclePositionVectors.size()) = - w.transpose() * C;
    b[obstaclePositionVectors.size()] = 0;

    //////
    std::cout << b << std::endl;
    std::cout << "------" << std::endl;
    /////

    return qDot;
}

Eigen::VectorXd QPAvoidance::gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, std::string controlPointName, Eigen::VectorXd q)
{
    Eigen::VectorXd qplus(7), qminus(7), result(7);
    double h{0.001}; // The value of the derivative is dependent on the size of h
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