#include "HIROAvoidance.h"

//#include "Eigen/Dense"

HIROAvoidance::HIROAvoidance()
{
    jointVelocityLimitsMin << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
    jointVelocityLimitsMax << +2.1750, +2.1750, +2.1750, +2.1750, +2.6100, +2.6100, +2.6100;
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
}

HIROAvoidance::~HIROAvoidance()
{
}

Eigen::Vector3d getClosestPointOnLine(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d p){
    //https://math.stackexchange.com/a/2193733/801563
    Eigen::Vector3d v = b - a;
    Eigen::Vector3d u = a - p;

    double top = (v.transpose() * u);
    double bottom = (v.transpose() * v);

    double t = -top/bottom;

    double d_a = (p - a).norm();
    double d_b = (p - b).norm();

    Eigen::Vector3d c;

    double d = 0;

    if (0 < t && t < 1){
        c = a + t * (b - a);
    } else {
        if (d_a < d_b){
            c = a;
        } else {
            c = b;
        }
    }

    return c;
}

//TODO: check that this is the correct data type passed in
void HIROAvoidance::getClosestControlPoint(){

    Eigen::Vector3d obstacle(0.5, 0, 0.6);
    // TODO: Get all potential joints that create lines from Anders function
    Eigen::Vector3d joint_one(0, 0, 0);
    Eigen::Vector3d joint_two(0, 0, 1);

    Eigen::Vector3d closest_point = HIROAvoidance::getClosestPointOnLine(joint_one, joint_two, obstacle);
    double distance_to_obstacle = (obstacle - closest_point).norm();


}


// % Closest point to P in segment AB
// % https://math.stackexchange.com/a/2193733/801563
// clear all
// A = [0, 0, 0]';
// B = [0, 0, 1]';
// P = [0.5, 0, 0.6]';
// v = B - A;
// u = A - P;
// t = - (v' * u) / (v' * v);
// d_A = norm(P-A);
// d_B = norm(P-B);
// if 0<t && t<1
//     C = (1-t) * A + t * B;
//     d = norm(P-C);
// else
//     if d_A < d_B
//         C = A;
//     else
//         C = B;
//     end
// end


