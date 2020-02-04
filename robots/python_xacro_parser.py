"""
This script is the output of my frustration with Gazebo, when I want to place reusable blocks, which passing some
strings as parameters. Also to change origin and orientation using quaternion mathematics. Wish me luck.

Usage:
1) Make the required changes in generate_xacro class's __init__ definition constants and just run the python file. This
will take panda_arm_reference.xacro as base, generated required xacro blocks and rewrites the old panda_arm.xacro. So
make a backup of working and stable panda_arm.xacro before attempting to do any changes for peace of your own mind.
2) You don't need to run the python file every time, only when you want to change the xacro settings
3) If you want to change the xacro, you can, but later update the changes in this file's xacro template, so that next
time they are auto-magically generated for you
4) This python file should be one stop destination relating to generating the xacro for all needs
5) The variables are named programmatically and clearly, so I won't comment. Everything is self explanatory
"""
import math
import numpy as np
import os

# XML Blocks
sensor_block = """


    <joint name = "{imu_joint}" type ="fixed">
          <origin rpy="0 0 0" xyz="0 0 0"/>
          <parent link="{imu_parent}"/>
          <child link="{imu_link}"/>
          <axis xyz="0 0 0"/>
    </joint>
    <link name="{imu_link}">
        <inertial>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <mass value="0.0"/>
            <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
        </inertial>
        <visual>
            <origin xyz="{visual_xyz}" rpy="{visual_rpy}" />
            <geometry>
                <box size = "0.05 0.05 0.05"/> 
            </geometry>
        </visual>
    </link>
    <gazebo reference="{imu_link}">
        <gravity>true</gravity>
        <!--      Below is the line to change the material-->
        <material>Gazebo/YellowGlow</material>
        <sensor name="{imu_sensor}" type="imu">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <visualize>true</visualize>
          <topic>imu_data0</topic>
          <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
            <topicName>{imu_data}</topicName>
            <bodyName>{imu_link}</bodyName>
            <gaussianNoise>0.0</gaussianNoise>
            <xyzOffset>{visual_xyz}</xyzOffset>
            <rpyOffset>{visual_rpy}</rpyOffset>
            <frameName>{imu_link}</frameName>
          </plugin>
          <pose>0 0 0 0 0 0</pose>
        </sensor>
    </gazebo>


"""

axes_blocks = """


    <joint name = "{imu_joint}_x_axis" type ="fixed">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <parent link="{imu_parent}"/>
      <child link="{imu_link}_x_axis"/>
      <axis xyz="0 0 0"/>
    </joint>
    <link name="{imu_link}_x_axis">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.0"/>
        <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      </inertial>
      <visual>
        <origin xyz="{visual_xyz}" rpy="{visual_rpy_x}" />
          <geometry>
            <mesh scale="0.007 0.007 0.007" filename="package://${axes_meshes}/meshes/visual/arrow.dae"/>
          </geometry>
        </visual>
    </link>
    <gazebo reference="{imu_link}_x_axis">
    <gravity>true</gravity>
    <material>Gazebo/RedBright</material>
    </gazebo>
     
      
    <joint name = "{imu_joint}_y_axis" type ="fixed">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <parent link="{imu_parent}"/>
      <child link="{imu_link}_y_axis"/>
      <axis xyz="0 0 0"/>
    </joint>
    <link name="{imu_link}_y_axis">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.0"/>
        <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      </inertial>
      <visual>
        <origin xyz="{visual_xyz}" rpy="{visual_rpy_y}" />
          <geometry>
            <mesh scale="0.007 0.007 0.007" filename="package://${axes_meshes}/meshes/visual/arrow.dae"/>
          </geometry>
        </visual>
    </link>
    <gazebo reference="{imu_link}_y_axis">
    <gravity>true</gravity>
    <material>Gazebo/Green</material>
    </gazebo>
    
    
    <joint name = "{imu_joint}_z_axis" type ="fixed">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <parent link="{imu_parent}"/>
      <child link="{imu_link}_z_axis"/>
      <axis xyz="0 0 0"/>
    </joint>
    <link name="{imu_link}_z_axis">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.0"/>
        <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      </inertial>
      <visual>
        <origin xyz="{visual_xyz}" rpy="{visual_rpy_z}" />
          <geometry>
            <mesh scale="0.007 0.007 0.007" filename="package://${axes_meshes}/meshes/visual/arrow.dae"/>
          </geometry>
        </visual>
    </link>
    <gazebo reference="{imu_link}_z_axis">
    <gravity>true</gravity>
    <material>Gazebo/Blue</material>
    </gazebo>


"""


# End XML Blocks

def list_to_string(my_list):
    """
    Input a list, it will output a string delimited by a white space
    :param my_list: list
        list of floats like [0.2 0.3 0.4]
    :return: str
        Returns literally 0.2 0.3 0.4 in a string format
    """
    return ' '.join([str(each_item) for each_item in my_list])


class generate_xacro:
    def __init__(self):
        # Below we set all the constants required for generating required config files
        # Sensor Settings
        self.real_imu_joint = "imu_link_joint"
        self.simulated_imu_joint = "simulated_imu_link_joint"
        self.imu_parent = "${arm_id}_link"
        self.real_imu_link = "imu_link"
        self.simulated_imu_link = "simulated_imu_link"
        self.real_imu_sensor = "imu_sensor"
        self.simulated_imu_sensor = "simulated_imu_sensor"
        self.real_imu_data = "imu_data"
        self.simulated_imu_data = "simulated_imu_data"
        # Specific Sensor Configuration
        self.real_visual_imu_xyz_0 = np.array([0.06, 0, 0.05])
        # We will offset to add to the xyz of real IMU to that of simulated IMU
        self.offset = np.array([0.5, 0, 0])
        self.simulated_visual_imu_xyz_0 = self.real_visual_imu_xyz_0 + self.offset

        self.real_visual_imu_xyz_1 = np.array([0.06, 0, -0.15])
        self.simulated_visual_imu_xyz_1 = self.real_visual_imu_xyz_1 + self.offset

        self.real_visual_imu_xyz_2 = np.array([0.06, -0.15, 0])
        self.simulated_visual_imu_xyz_2 = self.real_visual_imu_xyz_2 + self.offset

        self.real_visual_imu_xyz_3 = np.array([-0.06, 0, -0.06])
        self.simulated_visual_imu_xyz_3 = self.real_visual_imu_xyz_3 + self.offset

        self.real_visual_imu_xyz_4 = np.array([-0.14, 0.09, 0])
        self.simulated_visual_imu_xyz_4 = self.real_visual_imu_xyz_4 + self.offset

        self.real_visual_imu_xyz_5 = np.array([0, 0.10, -0.11])
        self.simulated_visual_imu_xyz_5 = self.real_visual_imu_xyz_5 + self.offset

        self.real_visual_imu_xyz_6 = np.array([0.13, 0.02, 0])
        self.simulated_visual_imu_xyz_6 = self.real_visual_imu_xyz_6 + self.offset

        self.real_imu_visual_rpy_0 = self.simulated_imu_visual_rpy_0 = [0, 0, 0]

        self.real_imu_visual_rpy_1 = self.simulated_imu_visual_rpy_1 = [0, 0, 0]

        self.real_imu_visual_rpy_2 = self.simulated_imu_visual_rpy_2 = [0, 0, 0]

        self.real_imu_visual_rpy_3 = self.simulated_imu_visual_rpy_3 = [0, 0, 0]

        self.real_imu_visual_rpy_4 = self.simulated_imu_visual_rpy_4 = [0, 0, 0]

        self.real_imu_visual_rpy_5 = self.simulated_imu_visual_rpy_5 = [0, 0, 0]

        self.real_imu_visual_rpy_6 = self.simulated_imu_visual_rpy_6 = [0, 0, 0]

        # Axes Settings
        self.real_imu_rpy_x_0 = self.simulated_imu_rpy_x_0 = [1, 0, 0]
        self.real_imu_rpy_y_0 = self.simulated_imu_rpy_y_0 = [0, 1, 0]
        self.real_imu_rpy_z_0 = self.simulated_imu_rpy_z_0 = [0, 0, 1]

        self.real_imu_rpy_x_1 = self.simulated_imu_rpy_x_1 = [1, 0, 0]
        self.real_imu_rpy_y_1 = self.simulated_imu_rpy_y_1 = [0, 1, 0]
        self.real_imu_rpy_z_1 = self.simulated_imu_rpy_z_1 = [0, 0, 1]

        self.real_imu_rpy_x_2 = self.simulated_imu_rpy_x_2 = [1, 0, 0]
        self.real_imu_rpy_y_2 = self.simulated_imu_rpy_y_2 = [0, 1, 0]
        self.real_imu_rpy_z_2 = self.simulated_imu_rpy_z_2 = [0, 0, 1]

        self.real_imu_rpy_x_3 = self.simulated_imu_rpy_x_3 = [1, 0, 0]
        self.real_imu_rpy_y_3 = self.simulated_imu_rpy_y_3 = [0, 1, 0]
        self.real_imu_rpy_z_3 = self.simulated_imu_rpy_z_3 = [0, 0, 1]

        self.real_imu_rpy_x_4 = self.simulated_imu_rpy_x_4 = [1, 0, 0]
        self.real_imu_rpy_y_4 = self.simulated_imu_rpy_y_4 = [0, 1, 0]
        self.real_imu_rpy_z_4 = self.simulated_imu_rpy_z_4 = [0, 0, 1]

        self.real_imu_rpy_x_5 = self.simulated_imu_rpy_x_5 = [1, 0, 0]
        self.real_imu_rpy_y_5 = self.simulated_imu_rpy_y_5 = [0, 1, 0]
        self.real_imu_rpy_z_5 = self.simulated_imu_rpy_z_5 = [0, 0, 1]

        self.real_imu_rpy_x_6 = self.simulated_imu_rpy_x_6 = [1, 0, 0]
        self.real_imu_rpy_y_6 = self.simulated_imu_rpy_y_6 = [0, 1, 0]
        self.real_imu_rpy_z_6 = self.simulated_imu_rpy_z_6 = [0, 0, 1]

        self.starting_string = ""

    def make_sensor(self, imu_joint, imu_parent, imu_link, imu_sensor, imu_data, visual_xyz, visual_rpy):
        global sensor_block
        return sensor_block.format(
            imu_joint=imu_joint,
            imu_parent=imu_parent,
            imu_link=imu_link,
            imu_sensor=imu_sensor,
            imu_data=imu_data,
            visual_xyz=list_to_string(visual_xyz),
            visual_rpy=list_to_string(visual_rpy)
        )

    def make_axes(self, imu_joint, imu_parent, imu_link, visual_xyz, visual_rpy_x, visual_rpy_y, visual_rpy_z):
        global axes_blocks
        return axes_blocks.format(
            imu_joint=imu_joint,
            imu_parent=imu_parent,
            imu_link=imu_link,
            visual_xyz=list_to_string(visual_xyz),
            visual_rpy_x=list_to_string(visual_rpy_x),
            visual_rpy_y=list_to_string(visual_rpy_y),
            visual_rpy_z=list_to_string(visual_rpy_z),
            description_pkg='{description_pkg}',
            axes_meshes='{axes_meshes}'
        )

    def gen_config(self):
        # Generating all sensors
        for type_of_imu in ['real_', 'simulated_']:
            for sensor_int in ['0', '1', '2', '3', '4', '5', '6']:
                self.starting_string += self.make_sensor(
                    imu_joint=getattr(self, type_of_imu + 'imu_joint') + sensor_int,
                    imu_parent=getattr(self, 'imu_parent') + sensor_int,
                    imu_link=getattr(self, type_of_imu + 'imu_link') + sensor_int,
                    imu_sensor=getattr(self, type_of_imu + 'imu_sensor') + sensor_int,
                    imu_data=getattr(self, type_of_imu + 'imu_data') + sensor_int,
                    visual_xyz=getattr(self, type_of_imu + 'visual_imu_xyz_' + sensor_int),
                    visual_rpy=getattr(self, type_of_imu + 'imu_visual_rpy_' + sensor_int)
                )
        # Generating all Axes
        for type_of_imu in ['real_', 'simulated_']:
            for sensor_int in ['0', '1', '2', '3', '4', '5', '6']:
                self.starting_string += self.make_axes(
                    imu_joint=getattr(self, type_of_imu + 'imu_joint') + sensor_int,
                    imu_parent=getattr(self, 'imu_parent') + sensor_int,
                    imu_link=getattr(self, type_of_imu + 'imu_link') + sensor_int,
                    visual_xyz=getattr(self, type_of_imu + 'visual_imu_xyz_' + sensor_int),
                    visual_rpy_x=getattr(self, type_of_imu + 'imu_rpy_x_' + sensor_int),
                    visual_rpy_y=getattr(self, type_of_imu + 'imu_rpy_y_' + sensor_int),
                    visual_rpy_z=getattr(self, type_of_imu + 'imu_rpy_z_' + sensor_int)
                )

    def write_config(self):
        final_closing_tags = """
        </xacro:macro>
        </robot>
        """

        script_directory = os.path.dirname(os.path.realpath(__file__))
        reference_file = open(script_directory + '/panda_arm_reference.xacro', 'r').read()
        final_file = reference_file + self.starting_string + final_closing_tags
        open(script_directory + '/panda_arm.xacro', 'w').close()
        with open(script_directory + '/panda_arm.xacro', 'w') as f:
            f.write(final_file)


if __name__ == "__main__":
    gg = generate_xacro()
    gg.gen_config()
    gg.write_config()
