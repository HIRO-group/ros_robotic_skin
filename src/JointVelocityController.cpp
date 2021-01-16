#include "JointVelocityController.h"


JointVelocityController::JointVelocityController(bool isSim) {
    this.isSim = isSim;

    // Switches to a proper controller depending on sim / real robot
    _initializeSwitchController(isSim);

    // Set up Velocity Publishers for Sim and Real
    // Sim
    _prepareSimVelocityPublisher();
    // Real
    _prepareRealVelocityPublisher();

    msgarray.data.resize(7);
}


JointVelocityController::~JointVelocityController() {}


/*
    Switches to a proper controller depending on sim / real robot
*/
void JointVelocityController::_initializeSwitchController(bool isSim){
    // Append 7 different joint controllers
    if (isSim) {
        // Prepare Empty Vectors
        vector<string> velocityControllerNames;
        vector<string> positionControllerNames;
        velocityControllerNames.reserve(7);
        positionControllerNames.reserve(7);
        // Append controllers to the vectors
        for (int i = 0; i < 7; i++) {
            string velocityControllerName = "panda_joint" + to_string(i+1) + "_velocity_controller";
            string positionControllerName = "panda_joint" + to_string(i+1) + "_position_controller";
            velocityControllerNames.push_back(velocityControllerName);
            positionControllerNames.push_back(positionControllerName);
        }
        position_controller_names.push_back("panda_joint_trajectory_controller");
    // Only append 1 joint controller
    } else {
        // Prepare Empty Vectors
        vector<string> velocityControllerNames;
        vector<string> positionControllerNames;
        velocityControllerNames.reserve(1);
        positionControllerNames.reserve(1);
        // Append controllers to the vectors
        for (int i = 0; i < 1; i++) {
            velocity_controller_names.push_back("panda_joint_velocity_controller");
            position_controller_names.push_back("panda_joint_position_controller");
        }
    }

    _switchController(position_controller_names, velocity_controller_names);
}

/*
    Switches to the provided controllers
*/
void JointVelocityController::_switchController(
    vector<string> position_controller_names,
    vector<string> velocity_controller_names) {
        // Prepare a ROS Service to switch controllers
        ros::ServiceClient switch_controller = n.serviceClient<controller_manager_msgs::SwitchController>(
            "/controller_manager/switch_controller");
        // Msg
        controller_manager_msgs::SwitchController srv;
        srv.request.start_controllers = velocity_controller_names;
        srv.request.stop_controllers = position_controller_names;
        srv.request.strictness = 1;
        srv.request.start_asap = true;
        srv.request.timeout = 10.0;
        // Send Request
        switch_controller.call(srv);
    }


void JointVelocityController::_prepareSimVelocityPublisher(){
    velocityPublishers.reserve(7);
    for (int i = 0; i < 7; i++) {
        string topic_name = "panda_joint" + to_string(i+1) + "_velocity_controller/command";
        velocityPublishers.push_back(n.advertise<std_msgs::Float64>(topic_name, 10));
    }
}


void JointVelocityController::_prepareRealVelocityPublisher(){
    realVelocityPublisher = n.advertise<std_msgs::Float64MultiArray>(
        "panda_joint_velocity_controller/command", 1);
}

/*
    Send Joint Velocity Commands
*/
void JointVelocityController::sendVelocities(const Eigen::VectorXd velocities){
    if (velocities.size() != 7) {
        ROS_ERROR_ONCE("The published vector must contain 7 elements");
    }

    if (isSim){
        _sendSimVelocities(velocities);
    } else {
        _sendRealVelocities(velocities);
    }
}


void JointVelocityController::_sendSimVelocities(const Eigen::VectorXd velocities){
    for (int i = 0; i < 7; i++){
        // double velocity = velocities[i];
        if (isnan(velocities[i]))
        {
            ROS_ERROR("NaN values can't be published as joint velocities");
            ROS_ERROR("Shutting down");
            ros::shutdown();
        }

        msg.data = velocities[i];
        velocityPublishers[i].publish(msg);
    }
}


void JointVelocityController::_sendRealVelocities(const Eigen::VectorXd velocities){
    for (int i = 0; i < 7; i++){
        // double velocity = velocities[i];
        if (isnan(velocities[i]))
        {
            ROS_ERROR("NaN values can't be published as joint velocities");
            ROS_ERROR("Shutting down");
            ros::shutdown();
        }
        msgarray.data[i] = velocities[i];
    }
    realVelocityPublisher.publish(msgarray);
}
