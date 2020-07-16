#!/usr/bin/env python
from controller_manager_msgs.srv import SwitchController, ListControllers
import rospy
from enum import Enum


class ControllerType(Enum):
    POSITION = 1
    VELOCITY = 2
    TRAJECTORY = 3


class RobotControllerManager():
    def __init__(self, controller_names=None):
        """
        Creates the RobotControllerManager object and
        determines the correct mode to be in - position,
        velocity, or trajectory.
        When the controller names are not provided, they default to the franka
        panda (which is the standard robot used in the repo - and pretty good, too)

        structure of controller_names:

        `controller_names[0]` - position controller names - `List[str]`

        `controller_names[1]` - velocity controller names - `List[str]`

        `controller_names[2]` - trajectory controller name* - `List[str]`

        """
        self.switch_controller_service_name = "/controller_manager/switch_controller"
        self.list_controller_service_name = "/controller_manager/list_controllers"
        if controller_names is None:
            position_controller_names = ["panda_joint{}_position_controller".format(i) for i in range(1, 8)]
            velocity_controller_names = ["panda_joint{}_velocity_controller".format(i) for i in range(1, 8)]
            trajectory_controller_names = ["panda_joint_trajectory_controller"]
        else:
            position_controller_names = controller_names[0]
            velocity_controller_names = controller_names[1]
            trajectory_controller_names = controller_names[2]

        self.controller_names = {

            ControllerType.POSITION: position_controller_names,
            ControllerType.VELOCITY: velocity_controller_names,
            ControllerType.TRAJECTORY: trajectory_controller_names

        }
        try:
            rospy.wait_for_service(self.list_controller_service_name)
            list_controllers = rospy.ServiceProxy(self.list_controller_service_name, ListControllers)
            controller_list = list_controllers().controller
            print(controller_list)
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
                while 1:
                    rospy.wait_for_service(self.switch_controller_service_name)
                    switch_controller = rospy.ServiceProxy(self.switch_controller_service_name, SwitchController)
                    # switch the controllers
                    resp = switch_controller(self.controller_names[desired_mode],
                                             self.controller_names[self.mode], 1, True, 10)
                    # Break from while loop is response is Okay meaning the switching was successful
                    if resp.ok:
                        break

                self.mode = desired_mode
            except rospy.ServiceException as e:
                rospy.logerr("Controller Manager service exception:", e)
