#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "include/PandaJointPositionController.h"

namespace hiro_panda
{
bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &nh)
{
}

void update(const ros::Time &time, const ros::Duration &period)
{

}
void starting(const ros::Time &time)
{

}
void stopping(const ros::Time &time)
{

}
}
