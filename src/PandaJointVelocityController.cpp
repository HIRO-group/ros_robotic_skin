#include "PandaJointVelocityController.h"

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <franka/robot_state.h>

namespace hiro_panda
{

bool PandaJointVelocityController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh)
{
    velocity_joint_interface_ = robot_hw->get<hardware_interface::VelocityJointInterface>();

    if (velocity_joint_interface_ == nullptr)
    {
        ROS_ERROR(
        "PandaJointVelocityController: Error getting velocity joint interface from hardware!");
        return false;
    }

    // Get joint name from parameter server
    if (!nh.getParam("joint", joint_name))
    {
        ROS_ERROR("No joint given (namespace: %s)", nh.getNamespace().c_str());
        return false;
    }

    i_joint = std::stoi(joint_name.back());
    std::string topic = joint_name + "_veocity_controller";
    sub_command_ = nh.subscribe<std_msgs::Float64>(topic, 10, &PandaJointVelocityController::commandCb, this);

    // Get Velocity Joint Handle from Velocity Joint Interface
    try {
        velocity_joint_handle_ = std::make_unique<hardware_interface::JointHandle>(
            velocity_joint_interface_->getHandle(joint_name));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
        "PandaJointVelocityController: Exception getting velocity joint handles: " << ex.what());
        return false;
    }

    // Get State Interface and Handle
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
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

    franka::RobotState robot_state = state_handle_->getRobotState();
    bool enforced = enforceJointPositionSoftLimit(robot_state.q[i_joint]);

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
    command = 0.0;
}


void PandaJointVelocityController::update(const ros::Time&, const ros::Duration& period)
{
    // Make sure joint velocity is within the limit
    // If it doesn't work, replace with
    // velocity_joint_limit_interface.enforceLimits(period);
    // enforceJointVelocityLimit(command);

    // Check if the joint positionreached the limit or not
    franka::RobotState robot_state = state_handle_->getRobotState();
    double next_position = robot_state.q[i_joint] + command * period.toSec();

    //TODO:
    // Send command if next position is not close to the limit
    // else Slow down the joint velocity to 0
    if (!enforceJointPositionSoftLimit(next_position))
    {
        velocity_joint_handle_->setCommand(command);
    }
}


void PandaJointVelocityController::stopping(const ros::Time &time)
{
  // can't send immediate commands for 0 velocity to robot
}


void PandaJointVelocityController::commandCb(const std_msgs::Float64ConstPtr& msg){
    command = msg-> data;
}


// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
void PandaJointVelocityController::enforceJointVelocityLimit(double &command)
{
    // Check that this joint has applicable limits
    if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
    {
        if (command > joint_urdf_->limits->velocity) // above upper limnit
        {
            command = joint_urdf_->limits->velocity;
        }
        else if (command < -joint_urdf_->limits->velocity) // below lower limit
        {
            command = -joint_urdf_->limits->velocity;
        }
    }
}


bool PandaJointVelocityController::enforceJointPositionSoftLimit(double &position)
{
    // Check that this joint has applicable limits
    if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
    {
        if (position > joint_urdf_->safety->soft_upper_limit) // above upper limnit
        {
            velocity_joint_handle_->setCommand(0.0);
            ROS_DEBUG_STREAM(joint_name + " reached the joint position soft upper limit of " + std::to_string(joint_urdf_->limits->upper));
            return true;
        }
        else if (position < joint_urdf_->safety->soft_lower_limit) // below lower limit
        {
            velocity_joint_handle_->setCommand(0.0);
            ROS_DEBUG_STREAM(joint_name + " reached the joint position soft lower limit of " + std::to_string(joint_urdf_->limits->lower));
            return true;
        }
    }

    return false;
}

}

PLUGINLIB_EXPORT_CLASS(hiro_panda::PandaJointVelocityController,
                       controller_interface::ControllerBase)
