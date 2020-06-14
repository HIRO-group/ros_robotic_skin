// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <PandaJointPositionController.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace hiro_panda {

bool PandaJointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) 
{
    position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();

    if (position_joint_interface_ == nullptr) {
        ROS_ERROR(
            "PandaJointPositionController: Error getting position joint interface from hardware!");
        return false;
    }

    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names)) {
        ROS_ERROR("PandaJointPositionController: Could not parse joint names");
    }

    if (joint_names.size() != 7) {
        ROS_ERROR_STREAM("PandaJointPositionController: Wrong number of joint names, got "
                        << joint_names.size() << " instead of 7 names!");
        return false;
    }

    position_joint_handles_.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        try {
        position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
        } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "PandaJointPositionController: Exception getting joint handles: " << e.what());
        return false;
        }
    }

    for (int i = 0; i < 7; i++)
    {
        joint_positions[i] = position_joint_handles_[i].getPosition();
    }

    sub_command_ = node_handle.subscribe<std_msgs::Float64MultiArray>("command", 10, &PandaJointPositionController::jointCommandCb, this);


    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("PandaJointPositionController: Could not read parameter arm_id");
        return false;
    }
    auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM("PandaJointPositionController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "PandaJointPositionController: Exception getting state handle from interface: " << ex.what());
        return false;
    }
    
    return true;
}

void PandaJointPositionController::starting(const ros::Time& /* time */) 
{
  for (size_t i = 0; i < 7; ++i) {
    joint_positions[i] = position_joint_handles_[i].getPosition();
  }
}

void PandaJointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) 
{
    // Get current Franka::RobotState
    franka::RobotState robot_state = state_handle_->getRobotState();

    for (int i = 0; i < 7; i++)
    {
        franka::limitRate(franka::kMaxJointVelocity,
                          franka::kMaxJointAcceleration,
                          franka::kMaxJointJerk,
                          joint_positions,
                          joint_velocities,
                          joint_accelerations);

        position_joint_handles_[i].setCommand(joint_positions[i]);
    }
}

void PandaJointPositionController::jointCommandCb(const std_msgs::Float64MultiArray::ConstPtr& joint_position_commands)
{
    if (joint_position_commands->data.size() != 7)
    {
        ROS_ERROR_STREAM("PandaJointPositionController: Wrong number of joint position commands, got "
                        << joint_position_commands->data.size() << " instead of 7 commands!");
    }

    for (int i = 0; i < 7; i++) joint_positions[i] = joint_position_commands->data[i];
}


}  // namespace hiro_panda

PLUGINLIB_EXPORT_CLASS(hiro_panda::PandaJointPositionController,
                       controller_interface::ControllerBase)
