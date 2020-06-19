#include "JointVelocityController.h"

JointVelocityController::JointVelocityController() {
    // Switch to velocity controller
    std::vector<std::string> velocity_controller_names; velocity_controller_names.reserve(7);
    std::vector<std::string> position_controller_names; position_controller_names.reserve(7);
    for (int i = 0; i < 7; i++) {
        velocity_controller_names.push_back("panda_joint" + std::to_string(i+1) + "_velocity_controller");
        position_controller_names.push_back("panda_joint" + std::to_string(i+1) + "_position_controller");
    }
    ros::ServiceClient switch_controller = n.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = velocity_controller_names;
    srv.request.stop_controllers = std::vector<std::string> {"panda_joint_trajectory_controller"};
    srv.request.strictness = 1;
    srv.request.start_asap = true;
    srv.request.timeout = 10.0;
    switch_controller.call(srv);

    // Set up publishers
    publishers.reserve(7);
    for (int i = 0; i < 7; i++) {
        publishers.push_back(n.advertise<std_msgs::Float64>("panda_joint" +
                                                        std::to_string(i+1) +
                                                        "_velocity_controller/command", 1));
    }
}

JointVelocityController::~JointVelocityController() {
}

void JointVelocityController::sendVelocities(const Eigen::VectorXd vel) {
    if (vel.size() == 7) {
        for (int i = 0; i < 7; i++) {
            msg.data = vel[i];
            publishers[i].publish(msg);
        }
    } else {
        ROS_ERROR_ONCE("The published vector must contain 7 elements");
    }
}