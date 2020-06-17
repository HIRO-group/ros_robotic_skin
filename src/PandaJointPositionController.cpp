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
                                          ros::NodeHandle& node_handle) {
    position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();

    if (position_joint_interface_ == nullptr) {
        ROS_ERROR(
            "PandaJointPositionController: Error getting position joint interface from hardware!");
        return false;
    }

    if (!node_handle.getParam("joint_names", joint_names)) {
        ROS_ERROR("PandaJointPositionController: Could not parse joint names");
    }

    if (joint_names.size() != 7) {
        ROS_ERROR_STREAM("PandaJointPositionController: Wrong number of joint names, got "
                        << joint_names.size() << " instead of 7 names!");
        return false;
    }

    if (!node_handle.getParam("joint_velocities", joint_velocities)) {
        ROS_ERROR("PandaJointPositionController: Could not parse joint velocities");
    }

    if (joint_velocities.size() != 7) {
        ROS_ERROR_STREAM("PandaJointPositionController: Wrong number of joint velocities, got "
                        << joint_velocities.size() << " instead of 7!");
        return false;
    }

    position_joint_handles_.resize(7);
    commanded_joint_positions.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        try {
        position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
        } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
            "PandaJointPositionController: Exception getting joint handles: " << e.what());
        return false;
        }
    }

    // Get URDF info about joint
    if (!urdf_model.initParamWithNodeHandle("robot_description", node_handle)) {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }

    bool passed = checkJointVelocityLimits(joint_velocities);

    for (int i = 0; i < 7; i++) {
        commanded_joint_positions[i] = position_joint_handles_[i].getPosition();
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

void PandaJointPositionController::starting(const ros::Time& /* time */) {
  count = 0;
}

void PandaJointPositionController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
    // Get current Franka::RobotState
    franka::RobotState robot_state = state_handle_->getRobotState();

    // Compute the postition errors
    std::array<double, 7> position_diff{};
    std::array<int, 7> sign{0, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> new_velocity{};


    for (int i = 0; i < 7; i++) {
        position_diff[i] = commanded_joint_positions[i] - position_joint_handles_[i].getPosition();

        // if ((count % 1000 == 0) && (joint_names[i] == "panda_joint1"))
        // {
        //     ROS_INFO_STREAM(commanded_joint_positions[i] << " " <<  position_joint_handles_[i].getPosition());
        // }
        
        if (position_diff[i] < 0)
        {
            sign[i] = -1;
        } else if (position_diff[i] > 0) {
            sign[i] = 1;
        }

        double gain = 1 / (1 + (0.2 * pow(abs(position_diff[i]), -1)));
        new_velocity[i] = sign[i] * gain * joint_velocities[i];

        double delta_angle = new_velocity[i] * period.toSec();
        double next_position = position_joint_handles_[i].getPosition() + delta_angle;
        position_joint_handles_[i].setCommand(next_position);

        // if ((count % 1000 == 0) && (joint_names[i] == "panda_joint1"))
        // {
        //     ROS_INFO_STREAM(joint_names[i] << " " << position_diff[i] << " " << gain << " " << delta_angle << " " << position_joint_handles_[i].getPosition() << " " << robot_state.q[i] << " " << next_position);
        // }
    }

    count += 1;
}

void PandaJointPositionController::jointCommandCb(const std_msgs::Float64MultiArray::ConstPtr& joint_position_commands) {
    if (joint_position_commands->data.size() != 7) {
        ROS_ERROR_STREAM("PandaJointPositionController: Wrong number of joint position commands, got "
                        << joint_position_commands->data.size() << " instead of 7 commands!");
    }

    for (int i = 0; i < 7; i++) {
        commanded_joint_positions[i] = joint_position_commands->data[i];
        ROS_INFO_STREAM("PandaJointPositionController::jointCommandCb::Joint " << i << " " << commanded_joint_positions[i]);
    }
}

bool PandaJointPositionController::checkJointVelocityLimits(std::vector<double>& joint_velocities) {
    for (int i=0; i < joint_names.size(); i++) {
        urdf::JointConstSharedPtr joint_urdf_ = urdf_model.getJoint(joint_names[i]);
        if (!joint_urdf_) {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_names[i].c_str());
            return false;
        }

        // Check that this joint has applicable limits
        if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC) {
            if (joint_velocities[i] > joint_urdf_->limits->velocity) {  // above upper limnit
                ROS_ERROR_STREAM("Given Joint Velocity is higher than the upper limit: " << joint_urdf_->limits->velocity);
                return false;
            } else if (joint_velocities[i] < -joint_urdf_->limits->velocity) {  // below lower limit
                ROS_ERROR_STREAM("Given Joint Velocity is lower than the lower limit: " << -joint_urdf_->limits->velocity);
                return false;
            }
        }
    }

    return true;
}

}  // namespace hiro_panda

PLUGINLIB_EXPORT_CLASS(hiro_panda::PandaJointPositionController,
                       controller_interface::ControllerBase)
