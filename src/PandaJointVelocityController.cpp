// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <PandaJointVelocityController.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace hiro_panda {

bool PandaJointVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                        ros::NodeHandle& node_handle) {
    velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
        ROS_ERROR(
            "PandaJointVelocityController: Error getting velocity joint interface from hardware!");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names)) {
        ROS_ERROR("PandaJointVelocityController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
        ROS_ERROR_STREAM("PandaJointVelocityController: Wrong number of joint names, got "
                        << joint_names.size() << " instead of 7 names!");
        return false;
    }

    velocity_joint_handles_.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        try {
        velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "PandaJointVelocityController: Exception getting joint handles: " << ex.what());
        return false;
        }
    }

    for (int i = 0; i < 7; i++) {
        joint_velocities[i] = 0.0;
    }

    last_time_called = ros::Time::now().toSec();

    sub_command_ = node_handle.subscribe<std_msgs::Float64MultiArray>("command", 10, &PandaJointVelocityController::jointCommandCb, this);

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("PandaJointVelocityController: Could not read parameter arm_id");
        return false;
    }
    auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM("PandaJointVelocityController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "PandaJointVelocityController: Exception getting state handle from interface: " << ex.what());
        return false;
    }

    return true;
}

void PandaJointVelocityController::starting(const ros::Time& /* time */) {
}

void PandaJointVelocityController::update(const ros::Time& time,
                                          const ros::Duration& period) {
    // Get current Franka::RobotState
    franka::RobotState robot_state = state_handle_->getRobotState();

    // If there is no command for more than 0.1 sec, set velocity to 0.0
    if (ros::Time::now().toSec() - last_time_called > 0.1) {
        for (int i = 0; i < 7; i++) velocity_joint_handles_[i].setCommand(0.0);
    } else {  // If command recieved, send the command to the controller
        for (int i = 0; i < 7; i++) {
            // Print out to the terminal just for the 1st joint for debugging.
            // Order: Commanded Joint Velocity << Acutal Commanded Joint Velocity << Current Joint Velocity
            // if (i == 0) ROS_INFO_STREAM("Panda_joint" << i+1 << " " << joint_velocities[i] << " " << robot_state.dq_d[i] << " " << robot_state.dq[i]);
            // Send command
            velocity_joint_handles_[i].setCommand(joint_velocities[i]);
        }
    }
}

void PandaJointVelocityController::stopping(const ros::Time& /*time*/) {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void PandaJointVelocityController::jointCommandCb(const std_msgs::Float64MultiArray::ConstPtr& joint_velocity_commands) {
    if (joint_velocity_commands->data.size() != 7) {
        ROS_ERROR_STREAM("PandaJointVelocityController: Wrong number of joint velocity commands, got "
                        << joint_velocity_commands->data.size() << " instead of 7 commands!");
    }

    // Receive Joint Velocity Commands from a topic and save them in joint_velocities.
    for (int i = 0; i < 7; i++) joint_velocities[i] = joint_velocity_commands->data[i];
    // Save the time when the last topic was published
    last_time_called = ros::Time::now().toSec();
}


}  // namespace hiro_panda

PLUGINLIB_EXPORT_CLASS(hiro_panda::PandaJointVelocityController,
                       controller_interface::ControllerBase)
