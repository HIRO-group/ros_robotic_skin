#!/usr/bin/env python
from controller_manager_msgs.srv import SwitchController, ListControllers
import rospy
from enum import Enum


class ControllerType(Enum):
    POSITION = 1
    VELOCITY = 2
    TRAJECTORY = 3


class PandaControllerManager():
    def __init__(self):
        """
        Creates the PandaControllerManager object and
        determines the correct mode to be in - position,
        velocity, or trajectory.
        """
        self.controller_service_name = "/controller_manager/switch_controller"
        velocity_controller_names = ["panda_joint{}_velocity_controller".format(i) for i in range(1, 8)]
        position_controller_names = ["panda_joint{}_position_controller".format(i) for i in range(1, 8)]
        self.controller_names = {
            ControllerType.POSITION: position_controller_names,
            ControllerType.VELOCITY: velocity_controller_names,
            ControllerType.TRAJECTORY: ["panda_joint_trajectory_controller"]
        }
        try:
            list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
            controller_list = list_controllers().controller
            self.mode = ControllerType.TRAJECTORY
            for controller in controller_list:
                if controller.state == "running":
                    if controller.name in position_controller_names:
                        self.mode = ControllerType.POSITION
                    elif controller.name in velocity_controller_names:
                        self.mode = ControllerType.VELOCITY
                    elif controller.name == "panda_joint_trajectory_controller":
                        self.mode = ControllerType.TRAJECTORY
        except rospy.ServiceException as e:
            print("Controller Manager Service exception", e)

    def switch_mode(self, desired_mode):
        """
        Based on mode type desired by user, switch controllers from current one.

        Desired mode should be a ControllerType enum
        """
        # check if mode is running

        if desired_mode == self.mode:
            print("Desired controller already running.")
        else:
            rospy.wait_for_service(self.controller_service_name)

            try:
                switch_controller = rospy.ServiceProxy(self.controller_service_name, SwitchController)
                # switch the controllers
                switch_controller(self.controller_names[desired_mode],
                                  self.controller_names[self.mode], 2, True, 10)
                print("Mode successfully changed!")
                self.mode = desired_mode
            except rospy.ServiceException as e:
                print("Controller Manager Service exception", e)
