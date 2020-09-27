#include "JointVelocityController.h"

JointVelocityController::JointVelocityController() {
    // Switch to velocity controller
    // std::vector<std::string> velocity_controller_names; velocity_controller_names.reserve(7);
    // std::vector<std::string> position_controller_names; position_controller_names.reserve(7);
    // for (int i = 0; i < 7; i++) {
    //     velocity_controller_names.push_back("panda_joint" + std::to_string(i+1) + "_velocity_controller");
    //     position_controller_names.push_back("panda_joint" + std::to_string(i+1) + "_position_controller");
    // }
    // position_controller_names.push_back("panda_joint_trajectory_controller");

    std::vector<std::string> velocity_controller_names; velocity_controller_names.reserve(1);
    std::vector<std::string> position_controller_names; position_controller_names.reserve(1);
    for (int i = 0; i < 1; i++) {
        velocity_controller_names.push_back("panda_joint_velocity_controller");
        position_controller_names.push_back("panda_joint_position_controller");
    }

    ros::ServiceClient switch_controller = n.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController srv;
    srv.request.start_controllers = velocity_controller_names;
    srv.request.stop_controllers = position_controller_names;
    srv.request.strictness = 1;
    srv.request.start_asap = true;
    srv.request.timeout = 10.0;
    switch_controller.call(srv);

    // Set up publishers
    publishers.reserve(7);
    for (int i = 0; i < 7; i++) {
        publishers.push_back(n.advertise<std_msgs::Float64>("panda_joint" +
                                                        std::to_string(i+1) +
                                                        "_velocity_controller/command", 10));
    }
    realPublisher = n.advertise<std_msgs::Float64MultiArray>("panda_joint_velocity_controller/command", 1);
    msgarray.data.resize(7);
}

JointVelocityController::~JointVelocityController() {
}

void JointVelocityController::sendVelocities(const Eigen::VectorXd vel)
{
    if (vel.size() == 7)
    {
        for (int i = 0; i < 7; i++){
            if (!std::isnan(vel[i]))
            {
                msg.data = vel[i];
                msgarray.data[i] = vel[i];
                publishers[i].publish(msg);
            }
            else
            {
                ROS_ERROR("NaN values can't be published as joint velocities");
                ROS_ERROR("Shutting down");
                ros::shutdown();
            }
        }
        realPublisher.publish(msgarray);
    }
    else
    {
        ROS_ERROR_ONCE("The published vector must contain 7 elements");
    }
}