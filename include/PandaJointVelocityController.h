// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>


namespace hiro_panda {

class PandaJointVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
  public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void stopping(const ros::Time&) override;

  private:
    hardware_interface::VelocityJointInterface* velocity_joint_interface_;
    hardware_interface::JointHandle velocity_joint_handle_;
    hardware_interface::PositionJointInterface* position_joint_interface_;
    hardware_interface::JointHandle position_joint_handle_;
    urdf::JointConstSharedPtr joint_urdf_;
    ros::Subscriber sub_command_;

    void commandCb(const std_msgs::Float64ConstPtr& msg);
    void enforceJointVelocityLimit(double &command);
    bool enforceJointPositionLimit(double &position);

    double command;
    double position;
    std::string joint_name;
    // If joint reaches its limit, it should halt.
    // margin will be subtracted/added to the limit
    double joint_margin;
};

}  // namespace hiro_panda

PLUGINLIB_EXPORT_CLASS(hiro_panda::PandaJointVelocityController,
                       controller_interface::ControllerBase)
