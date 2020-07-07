#include <iostream>
#include <math.h>
#include "Eigen/Dense"

#include "stdafx.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "optimization.h"
#include <math.h>


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
    Eigen::Vector3d getClosestPointOnLine(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d p);


public:
    //Constructor
    HIROAvoidance();
    // Class Variables
    // Class Methods
    void getClosestControlPoint();



};