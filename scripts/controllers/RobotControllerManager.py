#!/usr/bin/env python
from controller_manager_msgs.srv import SwitchController, ListControllers
import rospy
from enum import Enum


class ControllerType(Enum):
    POSITION = 1
    VELOCITY = 2
    TRAJECTORY = 3


class RobotControllerManager():
    def __init__(self, position_controller_names=None,
                 velocity_controller_names=None,
                 trajectory_controller_names=None):
        """
        Creates the RobotControllerManager object and
        determines the correct mode to be in - position,
        velocity, or trajectory.
        When the controller names are not provided, they default to the franka
        panda (which is the standard robot used in the repo - and pretty good, too)
        """
        self.switch_controller_service_name = "/controller_manager/switch_controller"
        self.list_controller_service_name = "/controller_manager/list_controllers"
        if position_controller_names is None:
            position_controller_names = ["panda_joint{}_position_controller".format(i) for i in range(1, 8)]
        if velocity_controller_names is None:
            velocity_controller_names = ["panda_joint{}_velocity_controller".format(i) for i in range(1, 8)]
        if trajectory_controller_names is None:
            trajectory_controller_names = ["panda_joint_trajectory_controller"]
        self.controller_names = {
            ControllerType.POSITION: position_controller_names,
            ControllerType.VELOCITY: velocity_controller_names,
            ControllerType.TRAJECTORY: trajectory_controller_names
        }
        try:
            rospy.wait_for_service(self.list_controller_service_name)
            list_controllers = rospy.ServiceProxy(self.list_controller_service_name, ListControllers)
            controller_list = list_controllers().controller
            self.mode = ControllerType.TRAJECTORY
            for controller in controller_list:
                if controller.state == "running":
                    if controller.name in position_controller_names:
                        self.mode = ControllerType.POSITION
                    elif controller.name in velocity_controller_names:
                        self.mode = ControllerType.VELOCITY
                    elif controller.name in trajectory_controller_names:
                        self.mode = ControllerType.TRAJECTORY
        except rospy.ServiceException as e:
            rospy.logerr("Controller Manager service exception:", e)

    def switch_mode(self, desired_mode):
        """
        Based on mode type desired by user, switch controllers from current one.

        Desired mode should be a ControllerType enum
        """
        # check if mode is running

        if desired_mode == self.mode:
            pass
        else:
            try:
                rospy.wait_for_service(self.switch_controller_service_name)
                switch_controller = rospy.ServiceProxy(self.switch_controller_service_name, SwitchController)
                # switch the controllers
                switch_controller(self.controller_names[desired_mode],
                                  self.controller_names[self.mode], 1, True, 10)

                self.mode = desired_mode
            except rospy.ServiceException as e:
                rospy.logerr("Controller Manager service exception:", e)
