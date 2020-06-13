#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/console.h>

#include "PandaJointPositionController.h"

namespace hiro_panda
{

bool PandaJointPositionController::init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &nh)
{
    position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();

    if (position_joint_interface_ == nullptr)
    {
        ROS_ERROR(
        "PandaJointPositionController: Error getting position joint interface from hardware!");
        return false;
    }

    // Get joint name from parameter server
    if (!nh.getParam("joint_names", joint_name))
    {
        ROS_ERROR("No joint given (namespace: %s)", nh.getNamespace().c_str());
        return false;
    }

    // Get joint margin from parameter server
    if(!nh.getParam("margin", joint_margin))
    {
        ROS_INFO("Joint Margin of 0.1 [rad/s] will be used");
        joint_margin = 0.1;
    }

    std::string topic = joint_name + "_position_controller";
    sub_command_ = nh.subscribe<std_msgs::Float64>(topic, 10, &PandaJointPositionController::commandCb, this);

    try{
        position_joint_handle_ = position_joint_interface_->getHandle(joint_name);
    }catch (const hardware_interface::HardwareInterfaceException& ex){
        ROS_ERROR_STREAM(
            "PandaJointPositionController: Exception getting position joint handles: " << ex.what());
        return false;
    }

    curr_position = position_joint_handle_.getPosition();
    desired_position = curr_position;
    // enforced = enforceJointPositionLimit(desired_position);

    // Get URDF info about joint
    urdf::Model urdf;
    if(!urdf.initParamWithNodeHandle("robot_description", nh))
    {
        ROS_ERROR("Failed to parse urdf file");
        return false;
    }
    joint_urdf_ = urdf.getJoint(joint_name);
    if(!joint_urdf_)
    {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
    }

    return true;
}

void PandaJointPositionController::starting(const ros::Time &time)
{
    initial_pose_ = position_joint_handle_.getPosition();
}

void PandaJointPositionController::update(const ros::Time&, const ros::Duration& period) {
    double position_diff = desired_position - curr_position;
    bool threshold = (abs(position_diff) <= 1);
    float divisor = threshold ? ((1 - position_diff) * 10) : 1;
    float sign = (position_diff > 0) ? 1 : -1;
    float command_ = (position_diff == 0) ? 0.0 : ((delta_angle * sign) / divisor);
    // enforced = enforceJointPositionLimit(desired_position);
    position_joint_handle_.setCommand(command_);
}

void PandaJointPositionController::stopping(const ros::Time &time)
{
  // can't send immediate commands for 0 velocity to robot
}

void PandaJointPositionController::commandCb(const std_msgs::Float64ConstPtr& msg)
{
    // if (enforceJointPositionLimit(msg->data))
    // {
    //     ROS_ERROR("Position '%s' is out of the allowed bounds!", msg->data);
    // }
    // else
    // {
    //     desired_position = msg->data;
    // }
    desired_position = msg->data;
}


// Note: we may want to remove this function once issue https://github.com/ros/angles/issues/2 is resolved
// void PandaJointPositionController::enforceJointVelocityLimit(double &command)
// {
//     // Check that this joint has applicable limits
//     if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
//     {
//         if (command > joint_urdf_->limits->velocity) // above upper limnit
//         {
//             command = joint_urdf_->limits->velocity;
//         }
//         else if (command < -joint_urdf_->limits->velocity) // below lower limit
//         {
//             command = -joint_urdf_->limits->velocity;
//         }
//     }
// }


// bool PandaJointPositionController::enforceJointPositionLimit(double &position)
// {
//     // Check that this joint has applicable limits
//     if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC)
//     {
//         if (position > joint_urdf_->limits->upper - joint_margin) // above upper limnit
//         {
//             position_joint_handle_.setCommand(0.0);
//             ROS_DEBUG_STREAM(joint_name + " reached the joint position upper limit of " + std::to_string(joint_urdf_->limits->upper));
//             return true;
//         }
//         else if (position < joint_urdf_->limits->lower + joint_margin) // below lower limit
//         {
//             position_joint_handle_.setCommand(0.0);
//             ROS_DEBUG_STREAM(joint_name + " reached the joint position lower limit of " + std::to_string(joint_urdf_->limits->lower));
//             return true;
//         }
//     }

//     return false;
// }

}  // namespace hiro_panda

PLUGINLIB_EXPORT_CLASS(hiro_panda::PandaJointPositionController,
                       controller_interface::ControllerBase)