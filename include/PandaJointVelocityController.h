// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <urdf/model.h>


namespace hiro_panda {

class PandaJointVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
  public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& nh) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void stopping(const ros::Time&) override;

  private:
    hardware_interface::VelocityJointInterface* velocity_joint_interface_; // not really necessary in .h
    hardware_interface::JointHandle velocity_joint_handle_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

    urdf::JointConstSharedPtr joint_urdf_;
    ros::Subscriber sub_command_;

    void commandCb(const std_msgs::Float64ConstPtr& msg);
    void enforceJointVelocityLimit(double &command);
    bool enforceJointPositionSoftLimit(double &position);

    int i_joint;
    double command;
    double position;
    std::string joint_name;
};

}  // namespace hiro_panda
