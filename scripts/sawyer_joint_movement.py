#!/usr/bin/env python
import math
import rospy
import numpy as np
import intera_interface
DEG2RAD = math.pi / 180.0


class SawyerJointControl():
    def __init__(self, limb="right"):
        """
        Sawyer Joint control class.

        Arguments
        ----------
        `limb`: `str`
            The limb used to initialize `self._limb`.
        """
        # initialize sawyer joints
        self._limb = intera_interface.Limb(limb)
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print(self._limb.joint_angles())

        self.velocities = {name: 0.0 for name in self._limb.joint_names()}
        self.positions = {name: -0.2*i for i, name in enumerate(self._limb.joint_names())}

    def spin(self):
        """
        Spin for Sawyer. Oscillates the robot.

        """
        dt = 0.01
        t = 0.0
        freq = 2.0

        while not rospy.is_shutdown():
            for joint_name in self._limb.joint_names():
                self.velocities[joint_name] = math.sin(2*math.pi*freq*t)

            try:
                self._limb.set_joint_velocities(self.velocities)
            except rospy.ROSInterruptException:
                print('Set Joint Velocities Failed')

            t += dt
            rospy.sleep(dt)

    def set_random_pose(self):
        """
        Moves the robot to semi-random poses.

        """
        def random_joint(low, high):
            return (high - low) * np.random.rand() + low

        limits = np.array([[0, -180, 0, 0, 0, 0, 0],
                           [350, 0, 350, 350, 340, 340, 540]])
        positions = [DEG2RAD*random_joint(low=limits[0, i], high=limits[1, i]) for i in range(7)]
        for joint_name, position in zip(self._limb.joint_names(), positions):
            print(joint_name, position*180.0/np.pi)
            self.positions[joint_name] = position

        if not rospy.is_shutdown():
            try:
                print(' '.join(['%.2f' % (position) for position in positions]))
                self._limb.move_to_joint_positions(self.positions, timeout=5)
            except rospy.ROSInterruptException:
                print('Set Joint Positions Failed')


if __name__ == '__main__':
    rospy.init_node('sawyer_joint_movement')
    try:
        sawer_control = SawyerJointControl()
        # sawer_control.spin()
        sawer_control.set_random_pose()
    except rospy.ROSInterruptException:
        print('Exiting Sawyer control process...')
