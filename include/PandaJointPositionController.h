// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <franka/rate_limiting.h>

namespace hiro_panda {

class PandaJointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface,
                                           franka_hw::FrankaStateInterface> 
{
  public:
      bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
      void starting(const ros::Time&) override;
      void update(const ros::Time&, const ros::Duration& period) override;

  private:
      void jointCommandCb(const std_msgs::Float64MultiArray::ConstPtr& joint_position_commands);
      hardware_interface::PositionJointInterface* position_joint_interface_;
      std::vector<hardware_interface::JointHandle> position_joint_handles_;
      std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
      ros::Subscriber sub_command_;
      
      std::array<double, 7> joint_positions{};
      std::array<double, 7> joint_velocities{0., 0., 0., 0., 0., 0., 0.};
      std::array<double, 7> joint_accelerations{0., 0., 0., 0., 0., 0., 0.};

};

}  // namespace hiro_panda
