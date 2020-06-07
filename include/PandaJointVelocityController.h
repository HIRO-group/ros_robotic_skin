#ifndef PANDA_JOINT_VELOCITY_CONTROLLER_H
#define PANDA_JOINT_VELOCITY_CONTROLLER_H

#include <iostream>
#include <vector>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>


namespace hiro_panda
{
class PandaJointVelocityController : public controller_interface::MultiInterfaceController<
                                hardware_interface::VelocityJointInterface,
                                franka_hw::FrankaStateInterface>
{
    public:
        // required functions for panda fci
        bool init(hardware_interface::RobotHW *hw, ros::NodeHandle &nh) override;
        void update(const ros::Time &time, const ros::Duration &period) override;
        void starting(const ros::Time &time) override;
        void stopping(const ros::Time &time) override;

    private:
        // hardware_interface and corresponding joint handles
        hardware_interface::VelocityJointInterface *_velocityJointInterface;
        std::vector<hardware_interface::JointHandle> _velocityJointHandles;

        geometry_msgs::Pose _targetPose;
        std::vector<ros::Subscriber> _targetSubscribers;

    public:
        void targetVelocityCb(const geometry_msgs::Pose &targetPose);
        JointVelocityController();
        ~JointVelocityController();
};
}

#endif // PANDA_JOINT_VELOCITY_CONTROLLER_H
