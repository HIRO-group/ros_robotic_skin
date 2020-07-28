#include <string>
#include <vector>
#include <algorithm>
#include <signal.h>
#include <math.h>
#include "ros/ros.h"
#include "JointVelocityController.h"
#include "QPAvoidance.h"
#include "sensor_msgs/JointState.h"
#include "ros_robotic_skin/PointArray.h"
#include "tf/transform_listener.h"
#include "KDLSolver.h"
#include "Eigen/Dense"
#include "HIROAvoidance.h"


enum AvoidanceMode {noAvoidance, Flacco, QP, HIRO};

class CartesianPositionController
{
private:
    AvoidanceMode avoidanceMode{noAvoidance};
    int numberControlPoints;
    double position_error_threshold{0.01}, pGain {2.5}, secondaryTaskGain{5.0};
    ros::NodeHandle n;
    ros::Rate rate{100.0};
    ros::Subscriber subscriberJointStates;
    ros::Subscriber subscriberObstaclePoints;
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;
    Eigen::VectorXd q, qDot{7}, jointLimitsMin{7}, jointLimitsMax{7}, jointMiddleValues{7}, jointRanges{7}, jointVelocityMax{7}, jointAccelerationMax{7};
    Eigen::Vector3d endEffectorPositionVector, positionErrorVector, desiredEEVelocity;
    std::vector<Eigen::Vector3d> obstaclePositionVectors;
    std::unique_ptr<Eigen::Vector3d[]> controlPointPositionVectors;
    Eigen::MatrixXd J, Jpinv, joint_positions;
    KDLSolver kdlSolver;
    std::vector<KDLSolver::closest_point> closestPoints;

    void JointStateCallback(const sensor_msgs::JointState::ConstPtr& scan);
    void ObstaclePointsCallback(const ros_robotic_skin::PointArray::ConstPtr& msg);
    void readEndEffectorPosition();
    void readControlPointPositions();
    Eigen::Vector3d getClosestPointOnLine(Eigen::Vector3d & a, Eigen::Vector3d & b, Eigen::Vector3d & p, double & t);
    void getClosestControlPoints();
    Eigen::VectorXd EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity);

public:
    CartesianPositionController();
    JointVelocityController jointVelocityController;
    QPAvoidance qpAvoidance;
    Eigen::VectorXd secondaryTaskFunctionGradient(Eigen::VectorXd q);
    void setMode(AvoidanceMode avoidanceModeName);
    void moveToPosition(Eigen::Vector3d position_vector);
    HIROAvoidance hiroAvoidance;
};

CartesianPositionController::CartesianPositionController()
{
    numberControlPoints = kdlSolver.getNumberControlPoints();
    this->controlPointPositionVectors = std::make_unique<Eigen::Vector3d[]>(numberControlPoints);
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
    jointVelocityMax << 2.1750, 2.1750, 2.1750 , 2.1750, 2.6100 , 2.6100 , 2.6100;
    jointAccelerationMax << 15, 7.5, 10, 12.5, 15, 20, 20;
    jointMiddleValues = 0.5 * (jointLimitsMax + jointLimitsMin);
    jointRanges = jointLimitsMax - jointLimitsMin;
    q.resize(7);
    subscriberJointStates = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, &CartesianPositionController::JointStateCallback, this);
    subscriberObstaclePoints = n.subscribe<ros_robotic_skin::PointArray>("/live_points", 1, &CartesianPositionController::ObstaclePointsCallback, this);
    readEndEffectorPosition();
    readControlPointPositions();
}

void CartesianPositionController::readEndEffectorPosition()
{
    while (ros::ok())
    {
        try
        {
            transform_listener.lookupTransform("/world", "/end_effector",
                                    ros::Time(0), transform);
            endEffectorPositionVector << transform.getOrigin().getX(),
                                         transform.getOrigin().getY(),
                                         transform.getOrigin().getZ();
            break;
        }
        catch (tf::TransformException ex)
        {
            ros::Duration(0.3).sleep();
        }
    }
}

void CartesianPositionController::readControlPointPositions()
{
    for (int i = 0; i < numberControlPoints; i++)
        controlPointPositionVectors[i] = kdlSolver.forwardKinematicsControlPoints(std::string("control_point") + std::to_string(i), q);
}

void CartesianPositionController::JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    q << msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6], msg->position[7], msg->position[8];
}

void CartesianPositionController::ObstaclePointsCallback(const ros_robotic_skin::PointArray::ConstPtr& msg)
{
    obstaclePositionVectors.clear();
    for (std::vector<geometry_msgs::Point, std::allocator<geometry_msgs::Point>>::const_iterator it = msg->points.begin();
         it != msg->points.end(); it++)
    {
        obstaclePositionVectors.push_back(Eigen::Vector3d(it->x, it->y, it->z));
    }
    ///////////////////////////
    obstaclePositionVectors.clear();
    obstaclePositionVectors.push_back(Eigen::Vector3d(-0.2, 0, 0.2));
    /////////////////////////////

    // Obtain the control point associated to each obstacle
    // Save the values in the class member closestPoints (of type custom struct)
    getClosestControlPoints();
}

Eigen::VectorXd CartesianPositionController::secondaryTaskFunctionGradient(Eigen::VectorXd q)
{
    return 2.0/7.0 * (q - jointMiddleValues).cwiseQuotient(jointRanges);
}

Eigen::VectorXd CartesianPositionController::EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity)
{
    // Function description
    J = kdlSolver.computeJacobian(std::string ("end_effector"), q).block(0,0,3,7);
    Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
    return Jpinv * desiredEEVelocity - secondaryTaskGain * ((Eigen::MatrixXd::Identity(7,7) - Jpinv*J) * secondaryTaskFunctionGradient(q));
}

void CartesianPositionController::setMode(AvoidanceMode avoidanceModeName)
{
    avoidanceMode = avoidanceModeName;
}

Eigen::Vector3d CartesianPositionController::getClosestPointOnLine(Eigen::Vector3d & a, Eigen::Vector3d & b, Eigen::Vector3d & p, double & t)
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

void CartesianPositionController::getClosestControlPoints()
{

    joint_positions = kdlSolver.forwardKinematicsJoints(q);

    // Initalize a list to hold the starting segemnt of the closest line
    closestPoints.clear();
    closestPoints.resize(obstaclePositionVectors.size());

    // Temporary variables
    float cur_dist;
    double cur_t;
    Eigen::Vector3d cur_control_point;

    // Loop though all obstical points to find the closest position on the robot to each
    Eigen::Vector3d starting_point, ending_point;
    std::vector<int> joints = {2, 3, 4, 6, 7, 9};
    for (int obs = 0; obs < obstaclePositionVectors.size(); obs++)
    {
        for (int i = 0; i < joint_positions.cols() - 1; i++)
        {
            starting_point = joint_positions.col(i);
            ending_point = joint_positions.col(i+1);
            cur_control_point = getClosestPointOnLine(starting_point, ending_point, obstaclePositionVectors[obs], cur_t);
            cur_dist = (obstaclePositionVectors[obs] - cur_control_point).norm();

            if (cur_dist < closestPoints[obs].distance_to_obs)
            {
                // This is a better potential segment and we should make it the closest point
                closestPoints[obs].segmentId = joints[i];
                closestPoints[obs].segmentPointA = starting_point;
                closestPoints[obs].segmentPointB = ending_point;
                closestPoints[obs].distance_to_obs = cur_dist;
                closestPoints[obs].control_point = cur_control_point;
                closestPoints[obs].t = cur_t;
            }
        }
    }
    ////////////////////////////////////////////////////
    std::cout << "closestPoints.size():" << closestPoints.size() << std::endl;
    for (int i = 0; i < closestPoints.size(); i++)
    {
        std::cout << "id:" << closestPoints[i].segmentId << std::endl;
        std::cout << "dist:" << closestPoints[i].distance_to_obs << std::endl;
        std::cout << "t:" << closestPoints[i].t << std::endl;
        std::cout << "point:" << closestPoints[i].control_point << std::endl;
        std::cout << "---------------------------------" << std::endl;
    }
    ////////////////////////////////////////////////////
}

void CartesianPositionController::moveToPosition(const Eigen::Vector3d desiredPositionVector)
{
    positionErrorVector = desiredPositionVector - endEffectorPositionVector;

    while (positionErrorVector.norm() > position_error_threshold && ros::ok())
    {
        readEndEffectorPosition();
        readControlPointPositions();
        positionErrorVector = desiredPositionVector - endEffectorPositionVector;
        desiredEEVelocity = pGain * positionErrorVector;
        ros::spinOnce();
        switch (avoidanceMode)
        {
            case noAvoidance:
                jointVelocityController.sendVelocities(EEVelocityToQDot(desiredEEVelocity));
                break;
            case Flacco:
            {
                // Eigen::MatrixXd Jreal = kdlSolver.computeJacobian("control_point5", q);

                // Eigen::Vector3d c = controlPointPositionVectors[5];

                // Eigen::Vector3d v1, v3;
                // transform_listener.lookupTransform("/world", "/panda_link1",
                //                         ros::Time(0), transform);
                // v1 << transform.getOrigin().getX(),
                //       transform.getOrigin().getY(),
                //       transform.getOrigin().getZ();
                // transform_listener.lookupTransform("/world", "/panda_link3",
                //                         ros::Time(0), transform);
                // v3 << transform.getOrigin().getX(),
                //       transform.getOrigin().getY(),
                //       transform.getOrigin().getZ();
                // double t;
                // t = (c-v1)[0] / (v3 - v1)[0];

                // Eigen::MatrixXd J2 = kdlSolver.computeJacobian2("panda_link4", q, t, 0);
                // Eigen::VectorXd va(3); va << 1,2,4;
                // std::cout << J2 << std::endl;
                // std::cout << "----------------" << std::endl;
                // std::cout << Jreal << std::endl;
                // std::cout << "----------------" << std::endl;
                // std::cout << "----------------" << std::endl;

                //std::cout << kdlSolver.forwardKinematicsJoints(q) << std::endl;

                //jointVelocityController.sendVelocities(EEVelocityToQDot(desiredEEVelocity));
                break;
            }

            case QP:
            {
                qDot = qpAvoidance.computeJointVelocities(q, desiredEEVelocity, obstaclePositionVectors, closestPoints);
                jointVelocityController.sendVelocities(qDot);
                break;
            }
            case HIRO:
            {
                std::cout << "-------------------------" << std::endl;
                closestPoints[0].t = 1;
                std::cout << kdlSolver.computeJacobian2(closestPoints[0], q) - kdlSolver.computeJacobian("panda_link2", q)<< std::endl;
                std::cout << "-------------------------" << std::endl;
                std::cout <<  kdlSolver.computeJacobian2(closestPoints[0], q) - kdlSolver.computeJacobian("panda_link3", q) << std::endl;
                std::cout << "-------------------------" << std::endl;
                break;
            }
        }
        // std::cout << rate.cycleTime() << std::endl; Let's us the actual run time of a cycle from start to sleep
        // std::cout << rate.expectedCycleTime() << std::endl;
        rate.sleep();
    }
    jointVelocityController.sendVelocities(Eigen::VectorXd::Constant(7, 0.0));
}

void on_shutdown(int sig)
{
    CartesianPositionController endController;
    endController.jointVelocityController.sendVelocities(Eigen::VectorXd::Constant(7, 0.0));
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CartesianPositionController", ros::init_options::NoSigintHandler);
    signal(SIGINT, on_shutdown);
    CartesianPositionController controller;

    if (argc == 1)
    {
        ROS_INFO("Running in no avoidance mode");
        controller.setMode(noAvoidance);
    }
    else
    {
        std::cout << argv[1] << std::endl;
        if (std::string(argv[1]) == "Flacco")
        {
            ROS_INFO("Flacco selected");
            controller.setMode(Flacco);
        }
        else if (std::string(argv[1]) == "QP")
        {
            ROS_INFO("QP selected");
            controller.setMode(QP);
        }
        else if (std::string(argv[1]) == "HIRO")
        {
            ROS_INFO("HIRO selected");
            controller.setMode(HIRO);
        }
        else
        {
            ROS_ERROR("Mode entered is not valid");
            ros::shutdown();
        }
    }

    while (ros::ok())
    {
        controller.moveToPosition(Eigen::Vector3d {0.7, 0.0, 0.4});
        controller.moveToPosition(Eigen::Vector3d {0.4, 0.0, 0.4});
    }
    return 0;
}
