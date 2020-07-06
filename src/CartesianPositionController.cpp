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


enum AvoidanceMode {noAvoidance, Flacco, QP};

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
    Eigen::VectorXd q, qDot{7}, jointLimitsMin{7}, jointLimitsMax{7}, jointMiddleValues{7}, jointRanges{7};
    Eigen::Vector3d endEffectorPositionVector, positionErrorVector, desiredEEVelocity;
    std::vector<Eigen::Vector3d> obstaclePositionVectors;
    std::unique_ptr<Eigen::Vector3d[]> controlPointPositionVectors;
    Eigen::MatrixXd J, Jpinv;
    KDLSolver kdlSolver;

    void JointStateCallback(const sensor_msgs::JointState::ConstPtr& scan);
    void ObstaclePointsCallback(const ros_robotic_skin::PointArray::ConstPtr& msg);
    void readEndEffectorPosition();
    void readControlPointPositions();
    Eigen::VectorXd EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity);

public:
    CartesianPositionController();
    ~CartesianPositionController();
    JointVelocityController jointVelocityController;
    QPAvoidance qpAvoidance;
    Eigen::VectorXd secondaryTaskFunctionGradient(Eigen::VectorXd q);
    void setMode(AvoidanceMode avoidanceModeName);
    void moveToPosition(Eigen::Vector3d position_vector);
};

CartesianPositionController::CartesianPositionController()
{
    numberControlPoints = kdlSolver.getNumberControlPoints();
    this->controlPointPositionVectors = std::make_unique<Eigen::Vector3d[]>(numberControlPoints);
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
    jointMiddleValues = 0.5 * (jointLimitsMax + jointLimitsMin);
    jointRanges = jointLimitsMax - jointLimitsMin;
    q.resize(7);
    subscriberJointStates = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, &CartesianPositionController::JointStateCallback, this);
    subscriberObstaclePoints = n.subscribe<ros_robotic_skin::PointArray>("/live_points", 1, &CartesianPositionController::ObstaclePointsCallback, this);
    readEndEffectorPosition();
    readControlPointPositions();
}

CartesianPositionController::~CartesianPositionController()
{
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

                std::cout << kdlSolver.forwardKinematicsJoints(q) << std::endl;

                jointVelocityController.sendVelocities(EEVelocityToQDot(desiredEEVelocity));

                break;
            }

            case QP:
            {
                qDot = qpAvoidance.computeJointVelocities(q, desiredEEVelocity, obstaclePositionVectors, numberControlPoints, controlPointPositionVectors);
                jointVelocityController.sendVelocities(qDot);
                break;
            }
            default:
                jointVelocityController.sendVelocities(EEVelocityToQDot(desiredEEVelocity));
        }
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
    if (argc == 2)
    {
        controller.setMode(Flacco);
    }
    else
    {
        controller.setMode(noAvoidance);
    }
    Eigen::MatrixXd empt(0,0);
    while (ros::ok())
    {
        controller.moveToPosition(Eigen::Vector3d {0.7, 0.0, 0.4});
        controller.moveToPosition(Eigen::Vector3d {0.4, 0.0, 0.4});
    }
    return 0;
}
