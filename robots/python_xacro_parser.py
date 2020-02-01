"""
This script is the output of my frustration with Gazebo, when I want to place reusable blocks, which passing some
strings as parameters. Also to change origin and orientation using quaternion mathematics. Wish me luck.
"""
import math
import numpy as np


# XML Blocks
sensor_block = """
<joint name = "{real_imu_joint}" type ="fixed">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <parent link="{imu_parent}"/>
      <child link="{real_imu_link}"/>
      <axis xyz="0 0 0"/>
    </joint>
    <link name="{real_imu_link}">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.0"/>
        <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      </inertial>
      <visual>
        <origin xyz="0.06 0 0.05" rpy="0 0 0" />
        <geometry>
            <box size = "0.05 0.05 0.05"/> 
        </geometry>
      </visual>
    </link>
    <gazebo reference="{real_imu_link}">
    <gravity>true</gravity>
    <!--      Below is the line to change the material-->
    <material>Gazebo/YellowGlow</material>
    <sensor name="imu_sensor0" type="imu">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>true</visualize>
      <topic>imu_data0</topic>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <topicName>imu_data0</topicName>
        <bodyName>{real_imu_link}</bodyName>
        <gaussianNoise>0.0</gaussianNoise>
        <xyzOffset>0.06 0 0.05</xyzOffset>
        <rpyOffset>0 0 0</rpyOffset>
        <frameName>{real_imu_link}</frameName>
      </plugin>
      <pose>0 0 0 0 0 0</pose>
    </sensor>
  </gazebo>

<!--    Joint 0 Axes Start-->
<!--    IMU 0 X Axis link-->
  <joint name = "imu_link_joint0_x_axis" type ="fixed">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <parent link="${arm_id}_link0"/>
      <child link="imu_link0_x_axis"/>
      <axis xyz="0 0 0"/>
  </joint>
  <link name="imu_link0_x_axis">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="0.0"/>
        <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
      </inertial>
      <visual>
        <origin xyz="0.06 0 0.05" rpy="1.57 0 0" />
          <geometry>
            <mesh scale="0.007 0.007 0.007" filename="package://${description_pkg}/meshes/visual/arrow.dae"/>
          </geometry>
        </visual>
  </link>
  <gazebo reference="imu_link0_x_axis">
    <gravity>true</gravity>
    <material>Gazebo/RedBright</material>
  </gazebo>
"""


# End XML Blocks

def list_to_string(my_list: list):
    """
    Input a list, it will output a string delimited by a white space
    :param my_list: list
        list of floats like [0.2 0.3 0.4]
    :return: str
        Returns literally "0.2 0.3 0.4"
    """
    return '"' + ' '.join([str(each_item) for each_item in my_list]) + '"'


class generate_xacro:
    def __init__(self):
        # Below we set all the constants required for generating required config files
        self.real_imu_joint = "imu_link_joint"
        self.simulated_imu_joint = "simulated_imu_link_joint"
        self.imu_parent = "${arm_id}_link"
        self.real_imu_link = "imu_link"
        self.simulated_imu_link = "simulated_imu_link"
        self.real_imu_sensor = "imu_sensor"
        self.simulated_imu_sensor = "simulated_imu_sensor"
        self.real_imu_data = "imu_data"
        self.simulated_imu_link = "simulated_imu_link"
        # Specific Sensor Configuration
        self.visual_imu_xyz_0 = np.array([0.06, 0, 0.05])
        # We will offset to add to the xyz of real IMU to that of simulated IMU
        self.offset = np.array([1, 0, 0])
        self.visual_simulated_imu_xyz_0 = self.visual_imu_xyz_0 + self.offset

        self.visual_imu_xyz_1 = np.array([0.06, 0, -0.15])
        self.visual_simulated_imu_xyz_1 = self.visual_imu_xyz_1 + self.offset

        self.visual_imu_xyz_2 = np.array([0.06, -0.15, 0])
        self.visual_simulated_imu_xyz_2 = self.visual_imu_xyz_2 + self.offset

        self.visual_imu_xyz_3 = np.array([-0.06, 0, -0.06])
        self.visual_simulated_imu_xyz_3 = self.visual_imu_xyz_3 + self.offset

        self.visual_imu_xyz_4 = np.array([-0.14, 0.09, 0])
        self.visual_simulated_imu_xyz_4 = self.visual_imu_xyz_4 + self.offset

        self.visual_imu_xyz_5 = np.array([0, 0.10, -0.11])
        self.visual_simulated_imu_xyz_5 = self.visual_imu_xyz_5 + self.offset

        self.visual_imu_xyz_6 = np.array([0.13, 0.02, 0])
        self.visual_simulated_imu_xyz_6 = self.visual_imu_xyz_6 + self.offset

        self.visual_imu_rpy_0 = [0, 0, 0]
        self.visual_simulated_imu_rpy_0 = [0, 0, 0]

        self.visual_imu_rpy_1 = [0, 0, 0]
        self.visual_simulated_imu_rpy_1 = [0, 0, 0]

        self.visual_imu_rpy_2 = [0, 0, 0]
        self.visual_simulated_imu_rpy_2 = [0, 0, 0]

        self.visual_imu_rpy_3 = [0, 0, 0]
        self.visual_simulated_imu_rpy_3 = [0, 0, 0]

        self.visual_imu_rpy_4 = [0, 0, 0]
        self.visual_simulated_imu_rpy_4 = [0, 0, 0]

        self.visual_imu_rpy_5 = [0, 0, 0]
        self.visual_simulated_imu_rpy_5 = [0, 0, 0]

        self.visual_imu_rpy_6 = [0, 0, 0]
        self.visual_simulated_imu_rpy_6 = [0, 0, 0]

        self.starting_string = ""

