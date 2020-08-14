#include "CartesianPositionController.h"



CartesianPositionController::CartesianPositionController()
{
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
    jointVelocityMax << 2.1750, 2.1750, 2.1750 , 2.1750, 2.6100 , 2.6100 , 2.6100;
    jointAccelerationMax << 15, 7.5, 10, 12.5, 15, 20, 20;

    jointMiddleValues = 0.5 * (jointLimitsMax + jointLimitsMin);
    jointRanges = jointLimitsMax - jointLimitsMin;

    q.resize(7);

    subscriberJointStates = n.subscribe<sensor_msgs::JointState>("/joint_states", 1,
                                                                 &CartesianPositionController::JointStateCallback,
                                                                 this);
    subscriberObstaclePoints = n.subscribe<ros_robotic_skin::PointArray>("/live_points", 1,
                                                                         &CartesianPositionController::ObstaclePointsCallback,
                                                                        this);

    readEndEffectorPosition();
}

void CartesianPositionController::readEndEffectorPosition()
{
    while (ros::ok())
    {
        try
        {
            transform_listener.lookupTransform("/world", "/panda_EE",
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
    if (isSim == true)
        q << msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6], msg->position[7], msg->position[8];
    else
        q << msg->position[2-2], msg->position[3-2], msg->position[4-2], msg->position[5-2], msg->position[6-2], msg->position[7-2], msg->position[8-2];



}

void CartesianPositionController::ObstaclePointsCallback(const ros_robotic_skin::PointArray::ConstPtr& msg)
{
    obstaclePositionVectors.clear();
    for (std::vector<geometry_msgs::Point, std::allocator<geometry_msgs::Point>>::const_iterator it = msg->points.begin();
         it != msg->points.end(); it++)
    {
        obstaclePositionVectors.push_back(Eigen::Vector3d(it->x, it->y, it->z));
    }
    getClosestControlPoints();
}

Eigen::VectorXd CartesianPositionController::EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity)
{
    J = kdlSolver.computeJacobian(std::string("panda_EE"), q);
    J = J.block(0,0,3,7);
    Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd secondaryTaskFunctionGradient =  2.0/7.0 * (q - jointMiddleValues).cwiseQuotient(jointRanges);
    return Jpinv * desiredEEVelocity - secondaryTaskGain * ((Eigen::MatrixXd::Identity(7, 7) - Jpinv*J) * secondaryTaskFunctionGradient);
}

void CartesianPositionController::setMode(AvoidanceMode avoidanceModeName) {
    avoidanceMode = avoidanceModeName;
}

Eigen::Vector3d CartesianPositionController::getClosestPointOnLine(Eigen::Vector3d & a, Eigen::Vector3d & b, Eigen::Vector3d & p, double & t)
{
    //TODO: change variable names
    //https://math.stackexchange.com/a/2193733/801563
    Eigen::Vector3d v = b - a;
    Eigen::Vector3d u = a - p;

    double top = (v.transpose() * u);
    double bottom = (v.transpose() * v);

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
    // TODO: Break down
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
}

void CartesianPositionController::moveToPosition(const Eigen::Vector3d desiredPositionVector)
{
    positionErrorVector = desiredPositionVector - endEffectorPositionVector;

    while (positionErrorVector.norm() > position_error_threshold && ros::ok()) {

        readEndEffectorPosition();

        positionErrorVector = desiredPositionVector - endEffectorPositionVector;

        // TODO: change this from a constant velocity, to a velocity dependent on distance
        desiredEEVelocity = 0.25 * positionErrorVector.normalized();

        ros::spinOnce();

        switch (avoidanceMode)
        {
            case noAvoidance:
                jointVelocityController.sendVelocities(EEVelocityToQDot(desiredEEVelocity));
                break;
            case Flacco:
            {
                // TODO: insert Flacco implementation here
                break;
            }
            case QP:
            {
                qDot = qpAvoidance.computeJointVelocities(q, desiredEEVelocity,
                                                          obstaclePositionVectors,
                                                          closestPoints, rate);

                jointVelocityController.sendVelocities(qDot);
                break;
            }
            case HIRO:
            {
                qDot = hiroAvoidance.computeJointVelocities(q, desiredEEVelocity,
                                                            obstaclePositionVectors,
                                                            closestPoints, rate);
                jointVelocityController.sendVelocities(qDot);
                break;
            }
        }
        rate.sleep();
    }
    // If no velocity is sent then the sim will continue at the last velocity sent
    if (isSim)
    {
        jointVelocityController.sendVelocities(Eigen::VectorXd::Constant(7, 0.0));
    }
}

void on_shutdown(int sig) {
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

    //Set up a circular trajectory
    // TODO: make function to send this path
    Eigen::Vector3d trajectory;
    double theta = 0;
    double radius = 0.25;
    double x = 0.5;
    double y = radius * std::cos(theta);
    double z = 0.5 + radius * std::sin(theta);
    double inc = 0.005;
    trajectory = Eigen::Vector3d(x, y, z);

    while (ros::ok())
    {
        controller.moveToPosition(trajectory);
        theta = fmod(theta + inc, M_PI * 2.0);
        y = radius * std::cos(theta);
        z = 0.5 + radius * std::sin(theta);
        trajectory = Eigen::Vector3d(x, y, z);
        ros::Duration(0.001).sleep();
    }

    return 0;
}
