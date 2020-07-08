#include <iostream>
#include <math.h>
#include "Eigen/Dense"

#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"
#include <math.h>
#include "KDLSolver.h"


class HIROAvoidance
{
private:
    // Class Variables
    Eigen::VectorXd jointVelocityLimitsMin{7};
    Eigen::VectorXd jointVelocityLimitsMax{7};
    Eigen::VectorXd jointLimitsMin{7};
    Eigen::VectorXd jointLimitsMax{7};
    Eigen::VectorXd qDot{7};

    // Class Methods
    Eigen::Vector3d getClosestPointOnLine(Eigen::Vector3d & a, Eigen::Vector3d & b, Eigen::Vector3d & p, double & t);


public:
    //Constructor
    HIROAvoidance();
    // Class Variables
    // Class Methods
    void getClosestControlPoint(KDLSolver & kdlSolver,  Eigen::VectorXd & q, std::vector<Eigen::Vector3d> & obstaclePoints);

    struct closest_point{
        int id;
        double t;
        float distance_to_obs = FLT_MAX;
        Eigen::Vector3d control_point;
    };

};