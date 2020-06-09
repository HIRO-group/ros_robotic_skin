#include <string>
#include <vector>
#include <signal.h>
#include <math.h>
#include "ros/ros.h"
#include "JointVelocityController.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_listener.h"
#include "KDLSolver.h"
#include "Eigen/Dense"


enum AvoidanceMode {noAvoidance, Flacco, QP};

class CartesianPositionController
{
private:
    AvoidanceMode avoidanceMode{noAvoidance};
    double position_error_threshold{0.01}, pGain {2.5}, secondaryTaskGain{5.0};
    ros::NodeHandle n;
    ros::Rate rate{100.0};
    ros::Subscriber subscriberJointStates;
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;
    Eigen::VectorXd q, qDot{7}, jointLimitsMin{7}, jointLimitsMax{7}, jointMiddleValues{7}, jointRanges{7};
    Eigen::Vector3d endEffectorPositionVector, positionErrorVector, desiredEEVelocity;
    Eigen::MatrixXd J, Jpinv;
    KDLSolver kdlSolver;

    void JointStateCallback(const sensor_msgs::JointState::ConstPtr& scan);
    void readEndEffectorPosition();
    Eigen::VectorXd EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity);

public:
    CartesianPositionController();
    ~CartesianPositionController();
    JointVelocityController jointVelocityController;
    Eigen::VectorXd secondaryTaskFunctionGradient(Eigen::VectorXd q);
    void setMode(AvoidanceMode avoidanceModeName);
    void moveToPosition(Eigen::Vector3d position_vector);
    Eigen::VectorXd gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, std::string controlPointName, Eigen::VectorXd q);
};

CartesianPositionController::CartesianPositionController()
{
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
    jointMiddleValues = 0.5 * (jointLimitsMax + jointLimitsMin);
    jointRanges = jointLimitsMax - jointLimitsMin;
    q.resize(7);
    subscriberJointStates = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, &CartesianPositionController::JointStateCallback, this);
    readEndEffectorPosition();
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

void CartesianPositionController::JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    q << msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6], msg->position[7], msg->position[8];
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
        positionErrorVector = desiredPositionVector - endEffectorPositionVector;
        desiredEEVelocity = pGain * positionErrorVector;
        ros::spinOnce();
        switch (avoidanceMode)
        {
            case Flacco:
                ROS_INFO("Flacco selected");
                ros::shutdown();
                break;
            case QP:
            {
                Eigen::MatrixXd C(2,7);
                C.row(0) = gradientOfDistanceNorm(Eigen::Vector3d(1.0, 1.0, 1.0), "end_effector", q);
                C.row(1) = gradientOfDistanceNorm(Eigen::Vector3d(0.0, 1.0, 1.0), "end_effector", q);
                std::cout << C << std::endl;
                ros::shutdown();
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

Eigen::VectorXd CartesianPositionController::gradientOfDistanceNorm(Eigen::Vector3d obstaclePositionVector, std::string controlPointName, Eigen::VectorXd q)
{
    Eigen::VectorXd qplus(7), qminus(7), result(7);
    double h{0.001};
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "QPController", ros::init_options::NoSigintHandler);
    signal(SIGINT, on_shutdown);
    CartesianPositionController controller;
    if (argc == 2)
    {
        controller.setMode(QP);
    }
    else
    {
        controller.setMode(noAvoidance);
    }

    while (ros::ok())
    {
        controller.moveToPosition(Eigen::Vector3d {0.7, 0.0, 0.4});
        controller.moveToPosition(Eigen::Vector3d {0.4, 0.0, 0.4});
    }
    return 0;
}
