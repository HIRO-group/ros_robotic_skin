#include "AvoidanceControl.h"


AvoidanceControl::AvoidanceControl(bool isSim)
{
    this.isSim = isSim;
    jointVelocityController = new JointVelocityController(isSim);

    // Initialize Constants

    // Joint Limits
    Eigen::VectorXd jointLimitsMin{7}, jointLimitsMax{7}, jointVelocityMax{7};
    jointLimitsMin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    jointLimitsMax << +2.8973, +1.7628, +2.8973, -0.0698, +2.8973, +3.7525, +2.8973;
    jointVelocityMax << 2.1750, 2.1750, 2.1750 , 2.1750, 2.6100 , 2.6100 , 2.6100;
    jointMiddleValues = 0.5 * (jointLimitsMax + jointLimitsMin);
    jointRanges = jointLimitsMax - jointLimitsMin;

    // desiredPosition << 0.3, -0.5, 0.5;
    // startPosition << 0.3, 0.5, 0.5;
    desiredPosition << 1.0, 0.0, 0.6;
    startPosition << 0.25, 0.0, 0.6;

    // Joint Positions
    q.resize(7);
    // Distance (give large value to start with to avoid any stupid control)
    distance.range = 1000000000;

    distanceSubscriber = nh.subscribe("/proximity_data6", 10, &AvoidanceControl::distanceCallBack, this);
    jointStateSubscriber = nh.subscribe("/joint_states", 10, &AvoidanceControl::jointStateCallback, this);
    distancePublisher = nh.advertise<sensor_msgs::Range>("/proximity_data6", 10);

    readEndEffectorPosition();

}


AvoidanceControl::~AvoidanceControl() {}


void AvoidanceControl::distanceCallBack(const sensor_msgs::Range::ConstPtr& msg)
{
    distance.header = msg->header;
    distance.radiation_type = msg->radiation_type;
    distance.field_of_view = msg->field_of_view;
    distance.min_range = msg->min_range;
    distance.max_range = msg->max_range;
    distance.range = msg->range;
}


void AvoidanceControl::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (isSim == true)
        q << msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6], msg->position[7], msg->position[8];
    else
        q << msg->position[2-2], msg->position[3-2], msg->position[4-2], msg->position[5-2], msg->position[6-2], msg->position[7-2], msg->position[8-2];
}


void AvoidanceControl::readEndEffectorPosition()
{
    while (ros::ok())
    {
        try
        {
            transform_listener.lookupTransform("/world", "/panda_link8",
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


Eigen::VectorXd AvoidanceControl::EEVelocityToQDot(Eigen::Vector3d desiredEEVelocity)
{
    Eigen::MatrixXd J = kdlSolver.computeJacobian(std::string("panda_EE"), q);
    J = J.block(0, 0, 3, 7);
    Eigen::MatrixXd Jpinv = J.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd secondaryTaskFunctionGradient =  2.0/7.0 * (q - jointMiddleValues).cwiseQuotient(jointRanges);
    return Jpinv * desiredEEVelocity - secondaryTaskGain * ((Eigen::MatrixXd::Identity(7, 7) - Jpinv*J) * secondaryTaskFunctionGradient);
}


void AvoidanceControl::moveToPosition(const Eigen::Vector3d desiredPositionVector)
{
    positionErrorVector = desiredPositionVector - endEffectorPositionVector;

    while (positionErrorVector.norm() > positionErrorThreshold && ros::ok()) {

        readEndEffectorPosition();

        positionErrorVector = desiredPositionVector - endEffectorPositionVector;
        desiredEEVelocity = desiredVelocity * positionErrorVector.normalized();

        Eigen::Vector3d repulsiveErrorVector = -positionErrorVector.normalized();
        Eigen::Vector3d repulsiveEEVelocity = maxRepulsiveVelocity / (1 + std::exp(6 * (2 * distance.range / 0.4 - 1))) * repulsiveErrorVector;

        Eigen::Vector3d commandVelocity = desiredEEVelocity + repulsiveEEVelocity;

        jointVelocityController.sendVelocities(EEVelocityToQDot(commandVelocity));

        ros::spinOnce();
        rate.sleep();
    }
    // If no velocity is sent then the sim will continue at the last velocity sent
    if (isSim)
    {
        jointVelocityController.sendVelocities(Eigen::VectorXd::Constant(7, 0.0));
    }
}


void AvoidanceControl::moveToStart()
{
    ROS_INFO_STREAM("startPosition");
    ROS_INFO_STREAM(startPosition);
    moveToPosition(startPosition);
}


void AvoidanceControl::moveToGoal()
{
    moveToPosition(desiredPosition);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "AvoidanceControl");
    cout << "spawning AvoidanceControl Object" << endl;
    bool isSim = false;
    AvoidanceControl controller = new AvoidanceControl(isSim);

    ros::Duration(10).sleep();

    while(true){
        cout << "Start Moving to a start point" << endl;
        controller.moveToStart();
        ros::Duration(1).sleep();

        cout << "Start Moving to a goal" << endl;
        controller.moveToGoal();
        ros::Duration(1).sleep();
    }
    return 0;
}
