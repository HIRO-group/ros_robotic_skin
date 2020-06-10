#include <iostream>
#include <math.h>
#include "Eigen/Dense"
#include "KDLSolver.h"

class QPAvoidance
{
private:
    KDLSolver kdlSolver;
    double dampingFactor0{0.1}, omega0{0.001};
    double computeDampingFactor(double omega);

    Eigen::VectorXd jointVelocityLimitsMin{7};
    Eigen::VectorXd jointVelocityLimitsMax{7};
    //void computeAandbMatrices(Eigen::MatrixXd& A, Eigen::MatrixXd& b);
    Eigen::VectorXd gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, std::string controlPointName, Eigen::VectorXd q);


public:
    QPAvoidance();
    ~QPAvoidance();
    Eigen::VectorXd computeJointVelocities(Eigen::VectorXd q, Eigen::Vector3d xDot,
                                           std::vector<Eigen::Vector3d> obstaclePositionVectors,
                                           int numberControlPoints,
                                           std::unique_ptr<Eigen::Vector3d[]> &controlPointPositionVectors);
};