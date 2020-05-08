#!/usr/bin/env python

import numpy as np
import rospkg
import os
"""
module for spawning any number of imus, anywhere, on
the robot.
"""

"""
<xacro:imu imu_id="0" xyz="0.05 0 -0.15" rpy="0 1.57 0" gravity="true" connected_to="panda_link1"/>
<xacro:imu imu_id="1" xyz="0.06 0 0.06" rpy="0 1.57 0" gravity="true" connected_to="panda_link2"/>
<xacro:imu imu_id="2" xyz="0 -0.05 -0.08" rpy="1.57 0 0" gravity="true" connected_to="panda_link3"/>
<xacro:imu imu_id="3" xyz="-0.06 0 0.08" rpy="0 -1.57 0" gravity="true" connected_to="panda_link4"/>
<xacro:imu imu_id="4" xyz="0 0.1 -0.1" rpy="-1.57 0 0" gravity="true" connected_to="panda_link5"/>
<xacro:imu imu_id="5" xyz="-0.05 0 0.03" rpy="0 -1.57 0" gravity="true" connected_to="panda_link6"/>
"""


class RealIMUSpawner():
    """
    Real IMU spawner for a robot.
    Each imu:
        has a certain pose. xyz rpy in xacro file.
        link connected to in the robot.
    """
    def __init__(self, robot_standard_link="panda_link"):
        self.pkg_path = rospkg.RosPack().get_path('ros_robotic_skin')
        real_imu_pose_path = os.path.join(self.pkg_path, 'config/imu_poses.txt')
        if not os.path.exists(real_imu_pose_path):
            raise EnvironmentError('imu_poses.txt file not found in config!')
        else:
            self.imu_arr = np.loadtxt(real_imu_pose_path)
            rows = self.imu_arr.shape[0]
            assert self.imu_arr.shape == (rows, 7), 'Wrong shape provided!'
            print(self.imu_arr.shape)

    def construct_imu_string(self):
        """
        example: <xacro:imu imu_id="5" xyz="-0.05 0 0.03" rpy="0 -1.57 0" gravity="true" connected_to="panda_link6"/>
        """
        self.xacro_strings = []
        for idx, imu in enumerate(self.imu_arr):
            x, y, z = imu[0], imu[1], imu[2]
            r, p, y = imu[3], imu[4], imu[5]
            link_no = int(imu[6])
            xacro_string = '  <xacro:imu imu_id="{}" xyz="{} {} {}" rpy="{} {} {}" gravity="true" connected_to="panda_link{}"/>\n'.format(
                idx, x, y, z, r, p, y, link_no
            )
            self.xacro_strings.append(xacro_string)

    def write(self, path='panda_arm_hand.urdf.xacro'):
        print(self.xacro_strings)
        robot_path = 'robots/{}'.format(path)
        full_path = os.path.join(self.pkg_path, robot_path)
        with open(full_path, "r") as xacro_file:
            lines = xacro_file.readlines()
        # now we're writing it out.
        is_comment = False
        with open(full_path, "w") as out_xacro_file:
            for line in lines:
                if not is_comment and "xacro:imu" in line:
                    # don't write out
                    pass
                else:
                    if "<!--" in line and "-->" not in line:
                        is_comment = True
                        out_xacro_file.write(line)
                    elif "-->" in line:
                        is_comment = False
                        out_xacro_file.write(line)
                    elif "xacro:hand" in line:
                        out_xacro_file.write(line)
                        for xacro_string in self.xacro_strings:
                            out_xacro_file.write(xacro_string)
                    else:
                        out_xacro_file.write(line)


if __name__ == '__main__':
    ris = RealIMUSpawner()
    ris.construct_imu_string()
    ris.write()
