#!/usr/bin/env python

#import numpy as np
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


class AddIMUToSU():
    """
    Adding n number of IMUs to Skin Unit (SU)
    Each imu:
        has a certain pose. xyz rpy in xacro file.
        link connected to in the robot.
    """
    def __init__(self, robot="panda", n=2):
        """
        initializes the add imu to su.
        """
        self.pkg_path = rospkg.RosPack().get_path('ros_robotic_skin')
        if robot == "panda":
            self.robot_standard_link = "panda_link"
        else:
            self.robot_standard_link = "right_l"

        # n is just the number of imus on a skin unit
        self.n = n

    def construct_imu_string(self):
        """
        example: <xacro:imu imu_id="5" xyz="-0.05 0 0.03" rpy="0 -1.57 0" gravity="true" connected_to="panda_link6"/>
        """
        rpy_per_link = np.array([[0.0, 1.57, 0.0], 
                                [0.0, 1.57, 0.0], [1.57, 0.0, 0.0], 
                                [0.0, -1.57, 0.0], [-1.57, 0.0, 0.0], 
                                [0.0, -1.57, 0.0], [0.0, 1.57, 0.0]])
        self.xacro_strings = []
        imu_id = 0
        x = 0
        y = 0
        z = 0
        for link, rpy in enumerate(rpy_per_link):
            # This is to start at panda_link1 not link0
            link = link + 1
            for i in self.n:
                xacro_string = '  <xacro:imu imu_id="{}" xyz="{} {} {}" rpy="{} {} {}" gravity="true" connected_to="{}{}"/>\n'.format(
                    imu_id, x, y, z,
                    rpy[0], rpy[1], rpy[2], self.robot_standard_link,
                    link
                )
                self.xacro_strings.append(xacro_string)
                imu_id = imu_id + 1

    def write(self, path='su.xacro'):
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
                    else:
                        out_xacro_file.write(line)


if __name__ == '__main__':
    ris = AddIMUToSU()
    ris.construct_imu_string()
    ris.write()
