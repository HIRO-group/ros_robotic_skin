#!/usr/bin/env python
import sys
import numpy as np
# ROS packages
import rospy
import rospkg
from sensor_msgs.msg import Imu
# Our packages
sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts import utils  # noqa: E402
from scripts.data_generators.stopwatch import StopWatch  # noqa: E402
from scripts.data_generators.storage import StaticPoseData, DynamicPoseData  # noqa: E402
from scripts.controllers.RobotController import PandaController  # noqa: E402

RATE = rospy.get_param('/dynamic_frequency')
SIM_DT = 1.0 / RATE
STATIC_MOTION_RECORD_TIME = 3.0
DYNAMIC_MOTION_RECORD_TIME = rospy.get_param('/oscillation_time')
FREQS = rospy.get_param('/oscillation_frequency')
AMPLITUDES = rospy.get_param('/oscillation_magnitude')
IS_SIM = rospy.get_param('/is_sim')
REST_TIME = rospy.get_param('/rest_time')


class DataCollector:
    """
    Class for collecting dynamic pose data and save them as a pickle file
    """
    def __init__(self, controller, poses_list, filepath='data/collected_data', is_sim=True):
        """
        Initializes DataCollector class.

        Arguments
        -----------
        controller:
            Wrapped controller to control either a robot.
        poses_list: list
            A list of poses. Each pose is a list.
            It includes 7 joint position, 7 joint velociites, and Pose name
        filepath: str
            File path to save the collected data
        """
        self.controller = controller
        self.poses_list = poses_list
        self.is_sim = is_sim
        # constant
        self.pose_names = [pose[2] for pose in poses_list]
        self.joint_names = map(str, self.controller.joint_names)

        # get imu names and topics through rostopic and xacro.
        self.imu_names, self.imu_topics = utils.get_imu_names_and_topics()

        self.curr_pose_name = self.pose_names[0]
        self.curr_joint_name = self.joint_names[0]
        self.prev_angular_velocity = 0.0

        self.watch_dt = StopWatch()
        self.watch_dynamic_motion = StopWatch()
        self.watch_static_motion = StopWatch()

        self.r = rospy.Rate(RATE)
        self.sim_dt = 1.0/RATE

        # data storage
        self.static_acceleration_storage = StaticPoseData(self.pose_names, self.imu_names, filepath+'_static')
        self.dynamic_acceleration_storage = DynamicPoseData(self.pose_names, self.joint_names, self.imu_names, filepath+'_dynamic')
        # Subscribe to IMUs
        for imu_topic in self.imu_topics:
            rospy.Subscriber(imu_topic, Imu, self.callback)

    def callback(self, data):
        """
        A callback function for IMU topics

        Arguments
        ----------
        data: sensor_msgs.msg.Imu
            IMU data. Please refer to the official documentation.
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
        """
        if self.watch_static_motion.is_started():
            acceleration = utils.Vector3_to_np(data.linear_acceleration)
            quaternion = utils.Quaternion_to_np(data.orientation)
            joint_angles = self.controller.joint_angles

            self.static_acceleration_storage.append(
                pose_name=self.curr_pose_name,     # for each defined initial pose
                imu_name=data.header.frame_id,      # frame id of imu
                data=np.r_[
                    quaternion,
                    acceleration,
                    joint_angles
                ]
            )

        if self.watch_dynamic_motion.is_started():
            acceleration = utils.Vector3_to_np(data.linear_acceleration)

            dt = self.watch_dt.get_elapsed_time()
            self.watch_dt.restart()
            if self.is_sim and dt <= 0.9*self.sim_dt:
                return

            curr_angular_velocity = self.controller.joint_velocity(self.curr_joint_name)
            angular_acceleration = (curr_angular_velocity - self.prev_angular_velocity) / dt
            self.prev_angular_velocity = curr_angular_velocity

            joint_angles = self.controller.joint_angles

            # time
            t = self.watch_dynamic_motion.get_elapsed_time()
            self.dynamic_acceleration_storage.append(
                pose_name=self.curr_pose_name,          # for each defined initial pose
                joint_name=self.curr_joint_name,        # for each excited joint
                imu_name=data.header.frame_id,          # for each imu
                data=np.r_[
                    acceleration,
                    joint_angles,
                    t,
                    angular_acceleration,
                    AMPLITUDES[0],  # Need to remove this later.
                    curr_angular_velocity,
                ]
            )

    def goto_defined_pose(self, pose, rest_time):
        positions, _, pose_name = pose[0], pose[1], pose[2]  # noqa: F841
        # first, move to the position from <robot>_positions.txt
        # TODO: We have to ensure that commanded positions are reached
        # Then REST_TIME should start once it reached the goal.
        self.controller.publish_positions(positions, sleep=rest_time)

        self.curr_pose_name = pose_name
        print('At Position: ' + pose_name,
              map(int, np.rad2deg * np.array(positions)))

    def record_static_motion(self, static_motion_record_time=3):
        self.watch_static_motion.start()
        rospy.sleep(static_motion_record_time)
        self.watch_static_motion.stop()

    def prepare_prev_states(self, joint_name):
        # Prepare for publishing velocities
        self.curr_joint_name = joint_name
        # Get current joint velocity
        self.prev_angular_velocity = self.controller.joint_velocity(self.curr_joint_name)

    def start_watches(self, dynamic_motion_record_time):
        # Start motion and recording
        self.watch_dt.start()
        self.watch_dynamic_motion.set_timer(dynamic_motion_record_time)
        self.watch_dynamic_motion.start()

    def collect_data(self, amplitudes, freqs, rest_time,
                     static_motion_record_time,
                     dynamic_motion_record_time):
        """
        This will move the joint of the robot arm like a sine wave
        for all joints for all defined poses.
        """
        self.controller.set_joint_position_speed(speed=1.0)

        for pose in self.poses_list:
            for i, joint_name in enumerate(self.joint_names):
                # Go to current setting position
                self.goto_defined_pose(pose, rest_time)
                # Record for given time
                self.record_static_motion(static_motion_record_time)

                # Prepare for recording dynamic motion
                self.prepare_prev_states(joint_name)
                self.start_watches(dynamic_motion_record_time)

                velocities = np.zeros(len(self.joint_names))
                while not rospy.is_shutdown():
                    # time within motion
                    t = self.watch_dynamic_motion.get_elapsed_time()

                    # Oscillated Velocity pattern
                    velocities[i] = amplitudes[i] * np.sin(2 * np.pi * freqs[i] * t)
                    self.controller.send_velocities(velocities)

                    if self.watch_dynamic_motion.is_ended():
                        break
                    self.r.sleep()

                self.watch_dynamic_motion.stop()
                rospy.sleep(1)

    def save(self, save=True, verbose=False, clean_static=True, clean_dynamic=False):
        """
        Save data to a pickle file.


        Arguments
        ----------
        `save`: `bool`
            If the data will be saved

        `verbose`: `bool`
        """
        if clean_static:
            static_data = self.static_acceleration_storage.clean_data(verbose)
        else:
            static_data = self.static_acceleration_storage.data

        if clean_dynamic:
            dynamic_data = self.dynamic_acceleration_storage.clean_data(verbose)
        else:
            dynamic_data = self.dynamic_acceleration_storage.data

        if save:
            rospy.loginfo('saving')
            self.static_acceleration_storage.save(static_data)
            self.dynamic_acceleration_storage.save(dynamic_data)


if __name__ == '__main__':
    rospy.init_node('data_collection')

    controller = PandaController(is_sim=IS_SIM)
    filename = 'panda_positions.txt'

    poses_list = utils.get_poses_list_file(filename)
    filepath = 'data/collected_data_panda'

    data_collector = DataCollector(controller, poses_list, filepath, IS_SIM)
    data_collector.collect_data(
        amplitudes=AMPLITUDES,
        freqs=FREQS,
        rest_time=REST_TIME,
        static_motion_record_time=STATIC_MOTION_RECORD_TIME,
        dynamic_motion_record_time=DYNAMIC_MOTION_RECORD_TIME)
    data_collector.save(save=True, verbose=False)
