#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/console.h>

#include "PandaJointVelocityController.h"

namespace hiro_panda
{
bool PandaJointVelocityController::init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &nh)
{
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();

  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "PandaJointVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "PandaJointVelocityController: Error getting position joint interface from hardware!");
    return false;
  }

  // Get joint name from parameter server
  if (!nh.getParam("joint", joint_name)){
    ROS_ERROR("No joint given (namespace: %s)", nh.getNamespace().c_str());
    return false;
  }

  // Get joint margin from parameter server
  if (!nh.getParam("margin", joint_margin)){
    ROS_INFO("Joint Margin of 0.1 [rad/s] will be used");
    joint_margin = 0.1;
  }

  std::string topic = joint_name + "_veocity_controller";
  sub_command_ = nh.subscribe<std_msgs::Float64>(topic, 1000, &PandaJointVelocityController::commandCb, this);

  try {
    velocity_joint_handle_ = velocity_joint_interface_->getHandle(joint_name);
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PandaJointVelocityController: Exception getting velocity joint handles: " << ex.what());
    return false;
  }
  try {
    position_joint_handle_ = position_joint_interface_->getHandle(joint_name);
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PandaJointVelocityController: Exception getting position joint handles: " << ex.what());
    return false;
  }

  position = position_joint_handle_.getPosition();
  bool enforced = enforceJointPositionLimit(position);

  // Get URDF info about joint
  urdf::Model urdf;
  if (!urdf.initParamWithNodeHandle("robot_description", nh))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }
  joint_urdf_ = urdf.getJoint(joint_name);
  if (!joint_urdf_)
  {
    ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
    return false;
  }

  return true;
}


void PandaJointVelocityController::starting(const ros::Time &time)
{
  // Start controller with 0.0 velocity
  command_buffer_.writeFromNonRT(0.0);
}


void PandaJointVelocityController::update(const ros::Time&, const ros::Duration& period)
{
  // Now get the published command
  command = *(command_buffer_.readFromRT());
  // Make sure joint velocity is within the limit
  enforceJointVelocityLimit(command);

  // Check if the joint positionreached the limit or not
  position = position_joint_handle_.getPosition();
  double next_position = position + command * period.toSec();
  if (!enforceJointPositionLimit(next_position))
  {
    velocity_joint_handle_.setCommand(command);
  }
}


void PandaJointVelocityController::stopping(const ros::Time &time)
{

}


void PandaJointVelocityController::commandCb(const std_msgs::Float64ConstPtr& msg){
  command_buffer_.writeFromNonRT(msg->data);
}


// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
void PandaJointVelocityController::enforceJointVelocityLimit(double &command)
{
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
  {
    if( command > joint_urdf_->limits->velocity ) // above upper limnit
    {
      command = joint_urdf_->limits->velocity;
    }
    else if( command < -joint_urdf_->limits->velocity ) // below lower limit
    {
      command = -joint_urdf_->limits->velocity;
    }
  }
}


bool PandaJointVelocityController::enforceJointPositionLimit(double &position)
{
  // Check that this joint has applicable limits
  if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
  {
    if( position > joint_urdf_->limits->upper - joint_margin ) // above upper limnit
    {
      velocity_joint_handle_.setCommand(0.0);
      ROS_DEBUG_STREAM(joint_name + " reached the joint position upper limit of " + std::to_string(joint_urdf_->limits->upper));
      return true;
    }
    else if( position < joint_urdf_->limits->lower + joint_margin ) // below lower limit
    {
      velocity_joint_handle_.setCommand(0.0);
      ROS_DEBUG_STREAM(joint_name + " reached the joint position lower limit of " + std::to_string(joint_urdf_->limits->lower));
      return true;
    }
  }

  return false;
}

}

PLUGINLIB_EXPORT_CLASS(hiro_panda::PandaJointVelocityController,
                       controller_interface::ControllerBase)
