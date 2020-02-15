#!/usr/bin/env python

import math
import rospy
import intera_interface


class SawyerJointControl():
    def __init__(self, limb="right"):
        # initialize sawyer joints
        self._limb = intera_interface.Limb(limb)
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        self.velocities = {name: 0.0 for name in self._limb.joint_names()}
        self.positions = {name: -0.2*i for i, name in enumerate(self._limb.joint_names())}

    def spin(self):
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

if __name__ == '__main__':
    rospy.init_node('sawyer_joint_movement')
    try:
        sawer_control = SawyerJointControl()
        sawer_control.spin()
    except rospy.ROSInterruptException:
        print('Exciting Sawyer control process...')