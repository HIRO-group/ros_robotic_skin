#include "HIROAvoidance.h"

HIROAvoidance::HIROAvoidance()
{
}


Eigen::Vector3d HIROAvoidance::getClosestPointOnLine(Eigen::Vector3d & a, Eigen::Vector3d & b, Eigen::Vector3d & p, double & t)
{
    //https://math.stackexchange.com/a/2193733/801563
    Eigen::Vector3d v = b - a;
    Eigen::Vector3d u = a - p;

    double top = (v.transpose() * u);
    double bottom = (v.transpose() * v);

    // I alter this value here because I want to use it
    t = -top/bottom;

    double d_a = (p - a).norm();
    double d_b = (p - b).norm();

    Eigen::Vector3d c;


    if (0 < t && t < 1){
        c = a + t * (b - a);
    } else {
        if (d_a < d_b){
            c = a;
            t = 0;
        } else {
            c = b;
            t = 1;
        }
    }

    return c;
}

//TODO: check that this is the correct data type passed in
void HIROAvoidance::getClosestControlPoint(KDLSolver & kdlSolver,  Eigen::VectorXd & q, std::vector<Eigen::Vector3d> & obstaclePoints)
{

    Eigen::MatrixXd joint_positions = kdlSolver.forwardKinematicsJoints(q);

    // Initalize a list to hold the starting segemnt of the closest line
    std::vector<HIROAvoidance::closest_point> control_points(obstaclePoints.size());

    //double distance_to_obstacle = (obstacle - closest_point).norm();
    float cur_dist;
    double cur_t;
    Eigen::Vector3d cur_control_point;

    Eigen::Vector3d starting_point;
    Eigen::Vector3d ending_point;
    std::vector<int> joints = {2, 3, 4, 6, 7, 9};


    // Loop though all obstical points to find the closest position on the robot to each
    for (int obs = 0; obs < obstaclePoints.size(); obs++){

        for (int i = 0; i < joint_positions.cols() - 1; i++){
            starting_point = joint_positions.col(i);
            ending_point = joint_positions.col(i+1);

            cur_control_point = getClosestPointOnLine(starting_point, ending_point, obstaclePoints[obs], cur_t);
            cur_dist = (obstaclePoints[obs] - cur_control_point).norm();

            // This is a better potential segment and we should make it the closest point
            if (cur_dist < control_points[obs].distance_to_obs){
                control_points[obs].distance_to_obs = cur_dist;
                control_points[obs].control_point = cur_control_point;
                control_points[obs].id = joints[i];
                control_points[obs].t = cur_t;
            }
        }
    }

    for (int i = 0; i < control_points.size(); i++)
    {
        std::cout << "id:" << control_points[i].id << std::endl;
        std::cout << "dist:" << control_points[i].distance_to_obs << std::endl;
        std::cout << "t:" << control_points[i].t << std::endl;
        std::cout << "point:" << control_points[i].control_point << std::endl;
        std::cout << "---------------------------------" << std::endl;
    }

    ros::shutdown();

    //std::cout << "HIRO Avoidance" << std::endl;
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
