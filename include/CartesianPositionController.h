#include <string>
#include <vector>
#include <algorithm>
#include <signal.h>
#include <math.h>
#include <cmath>

#include "ros/ros.h"
#include "JointVelocityController.h"
#include "QPAvoidance.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"
#include "ros_robotic_skin/PointArray.h"
#include "tf/transform_listener.h"
#include "KDLSolver.h"
#include "Eigen/Dense"
#include "HIROAvoidance.h"

enum AvoidanceMode {noAvoidance, Flacco, QP, HIRO};
class CartesianPositionController {
 private:
   //TODO: make isSim a variable set on the command line
    bool isSim = true;
    AvoidanceMode avoidanceMode{noAvoidance};
    int numberControlPoints;
    double position_error_threshold{0.01}, pGain {2.5}, secondaryTaskGain{5.0};
    ros::NodeHandle n;
    ros::Rate rate{100.0};
    ros::Subscriber subscriberJointStates;
    ros::Subscriber subscriberObstaclePoints;

    ros::Publisher ee_xyz_publisher;
    geometry_msgs::Point ee_xyz;
    tf::TransformListener transform_listener;
    tf::StampedTransform transform;
    std::vector<Eigen::Vector3d> obstaclePositionVectors;
    std::unique_ptr<Eigen::Vector3d[]> controlPointPositionVectors;
    Eigen::Vector3d endEffectorPositionVector, positionErrorVector, desiredEEVelocity;
    Eigen::VectorXd q, qDot{7}, jointLimitsMin{7}, jointLimitsMax{7}, jointMiddleValues{7}, jointRanges{7}, jointVelocityMax{7}, jointAccelerationMax{7};
    Eigen::MatrixXd J, Jpinv, joint_positions;
    std::vector<KDLSolver::closest_point> closestPoints;
    KDLSolver kdlSolver;


    void JointStateCallback(const sensor_msgs::JointState::ConstPtr& scan);
    void ObstaclePointsCallback(const ros_robotic_skin::PointArray::ConstPtr& msg);

    void getClosestControlPoints();
    Eigen::Vector3d getClosestPointOnLine(Eigen::Vector3d & a, Eigen::Vector3d & b, Eigen::Vector3d & p, double & t);
    Eigen::VectorXd EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity);

    // Helper functions
    //void



 public:

    CartesianPositionController();
    void setMode(AvoidanceMode avoidanceModeName);
    AvoidanceMode getMode();
    ros::Publisher graph_start_publisher;

    void moveToPosition(Eigen::Vector3d position_vector);
    void moveToTime(Eigen::Vector3d position_vector);
    void setVelocitiesToZero();
    void moveToStart();
    void readEndEffectorPosition();
    void moveInCircle(double radius, double timeToComplete);

    JointVelocityController jointVelocityController;
    QPAvoidance qpAvoidance;
    HIROAvoidance hiroAvoidance;

};
