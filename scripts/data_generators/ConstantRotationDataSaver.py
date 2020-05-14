#!/usr/bin/env python
import os
import sys
from collections import OrderedDict
import copy
import numpy as np
import pickle

import tf
import rospy
import rospkg
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion

# add to sys.path to make sure we can import utils and controllers
sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts import utils  # noqa: E402
from scipts.controllers.RobotController import PandaController, SawyerController  # noqa: E402

RAD2DEG = 180.0/np.pi
DATA_COLLECTION_TIME = 3.0
CONSTANT_VELOCITY = 1.0
JOINT_ROT_TIME = 0.5


class ConstantRotationData():
    """
    Class to store dynamic pose data into a nested dictionary.
    It is stored as in [20 poses][7 excited joints][7 IMUs][data].
    self.data[pose_name][joint_name][imu_name] = np.ndarray
    The data is defined as below.
    data = [acceleration, joint angles, joint velocities].
    """
    def __init__(self, pose_names, joint_names, imu_mappings, filepath):
        """
        Initialize StaticPoseData class.

        Arguments
        ----------
        pose_names: list[str]
            Names of poses
        joint_names: list[str]
            Names of joints
        imu_mappings: Dict[str: str]
            Names of imus, with the corresponding link they are connected to.
        filepath: str
            Path to save the collected data
        """
        self.pose_names = pose_names
        self.joint_names = joint_names
        self.imu_mappings = imu_mappings
        self.full_imu_names = []
        for key in self.imu_mappings:
            self.full_imu_names.append(self.imu_mappings[key])
        # self.imu_names = imu_names
        self.filepath = filepath
        self.data = OrderedDict()

        # Create nested dictionary to store data
        for pose_name in pose_names:
            self.data[pose_name] = OrderedDict()
            for joint_name in joint_names:
                self.data[pose_name][joint_name] = OrderedDict()
                for imu_name in self.full_imu_names:
                    self.data[pose_name][joint_name][imu_name] = np.empty((0, 15), float)

    def append(self, pose_name, joint_name, imu_name, data):
        """
        Append data to a dictionary whose keys are
        [pose_name][joint_name][self.imu_mappings[imu_name]]

        Arguments
        ----------
        pose_name: str
            Names of poses
        joint_name: str
            Names of joints
        imu_name: str
            Names of imus
        data: np.array
            Numpy array of size (1,4).
            Includes an accelerometer measurement and a joint angle.
        """
        imu_val = self.imu_mappings[imu_name]
        self.data[pose_name][joint_name][imu_val] = \
            np.append(self.data[pose_name][joint_name][imu_val], np.array([data]), axis=0)

    def clean_data(self, verbose=False):
        """
        Cleans the data.

        Arguments
        ----------
        `verbose`: `bool`
        """
        # Create nested dictionary to store data
        data = copy.deepcopy(self.data)
        for pose_name in self.pose_names:
            for joint_name in self.joint_names:
                for imu_name in self.full_imu_names:
                    norm = np.linalg.norm(self.data[pose_name][joint_name][imu_name][:, :3], axis=1)
                    norm, in_std = utils.reject_outliers(norm)
                    data[pose_name][joint_name][imu_name] = self.data[pose_name][joint_name][imu_name][in_std, :]

        return data

    def save(self, data, filter=False):
        """
        Saves the data to a pickle file.

        Arguments
        ----------
        `data`: `OrderedDict`
            The data to be saved

        `filter`: `bool`
            Whether the data is filtered of values
            that are not near the constant velocity defined in this file.
        """
        ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
        filepath = os.path.join(ros_robotic_skin_path, self.filepath+'.pickle')
        if filter:
            # filters the data and gets the constant angular velocity values
            data = self.filter_data(data)
        with open(filepath, 'wb') as f:
            pickle.dump(data, f)

    def filter_data(self, data):
        """
        Filters the data along values near `CONSTANT_VELOCITY`

        Arguments
        ----------
        `data`: `OrderedDict`
            The data from the data collection.
        """
        for pose in data.keys():
            poses_dict = data[pose]
            for joint in poses_dict.keys():
                all_joints = poses_dict[joint]
                for imu in all_joints.keys():
                    imu_points = all_joints[imu][0]
                    delete_idxs = []
                    for ind, val in enumerate(imu_points):
                        joint_vel = val[-1]
                        if abs(joint_vel - CONSTANT_VELOCITY) >= 0.2:
                            delete_idxs.append(ind)
                    res = np.delete(imu_points, delete_idxs, 0)
                    y = np.expand_dims(res, axis=0)
                    all_joints[imu] = y
        return data


class ConstantRotationDataSaver():
    """
    Class for collecting dynamic pose data nd save them as a pickle file
    """
    def __init__(self, controller, poses_list, filepath='data/constant_data'):
        """
        Initializes DynamicPoseDataSaver class.

        Arguments
        -----------
        controller:
            Wrapped controller to control either Panda or Sawyer robot.
        poses_list: list
            A list of poses. Each pose is a list.
            It includes 7 joint positiosn, 7 joint velociites, and Pose name
        filepath: str
            File path to save the collected data
        """
        self.controller = controller
        self.poses_list = poses_list

        # constant
        self.pose_names = [pose[2] for pose in poses_list]
        self.joint_names = self.controller.joint_names
        self.imu_names, self.imu_topics, self.imu_mappings = \
            utils.get_imu_names_and_topics()
        """
        to-do get the joints the imus are connected to.
        """
        self.collecting_data = False
        self.curr_pose_name = self.pose_names[0]
        self.curr_joint_name = self.joint_names[0]
        self.pubs = {}
        for imu_name in self.imu_names:
            self.pubs[imu_name] = rospy.Publisher('/Anorm%s' % (list(imu_name)[-1]), Float32, queue_size=10)

        # data storage
        self.data_storage = ConstantRotationData(self.pose_names, self.joint_names, self.imu_mappings, filepath)
        rospy.sleep(1)
        # Subscribe to IMUs
        for imu_topic in self.imu_topics:
            rospy.Subscriber(imu_topic, Imu, self.callback)
        self.tf_listener = tf.TransformListener()
        self.Q = {imu_name: Quaternion() for imu_name in self.imu_names}

    def callback(self, data):
        """
        A callback function for IMU topics

        Arguments
        ----------
        data: sensor_msgs.msg.Imu
            IMU data. Please refer to the official documentation.
            http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html
        """
        accel = data.linear_acceleration
        msg = Float32()
        msg.data = np.linalg.norm(np.array([accel.x, accel.y, accel.z]))
        self.pubs[data.header.frame_id].publish(msg)
        if self.collecting_data:
            # acceleration of skin unit, followed by its acceleration
            accel = data.linear_acceleration
            q = self.Q[data.header.frame_id]
            # get the orientation of the imu
            J = np.array([self.controller.joint_angle(name) for name in self.joint_names])
            joint_velocity = self.controller.joint_velocity(self.curr_joint_name)
            # if self.curr_joint_name == 'right_j0'
            # and data.header.frame_id == 'imu_link0':
            # rospy.loginfo(utils.n2s(np.array([accel.x, accel.y, accel.z])))
            self.data_storage.append(
                self.curr_pose_name,            # for each defined initial pose
                self.curr_joint_name,           # for each excited joint
                data.header.frame_id,           # for each imu
                np.array([
                    q.x, q.y, q.z, q.w,
                    accel.x, accel.y, accel.z,
                    J[0], J[1], J[2], J[3], J[4], J[5], J[6],
                    joint_velocity])
            )

    def rotate_at_constant_vel(self):
        """
        This will move the joint of the robot arm like a sine wave
        for all joints for all defined poses.
        """
        for pose in self.poses_list:

            positions, _, pose_name = pose[0], pose[1], pose[2]  # noqa: F841
            self.curr_pose_name = pose_name
            self.controller.publish_positions(positions, 5)
            print('At Position: ' + pose_name,
                  map(int, RAD2DEG*np.array(positions)))
            for i, joint_name in enumerate(self.joint_names):
                self.curr_joint_name = joint_name
                print(joint_name)

                # Prepare for publishing velocities
                velocities = np.zeros(len(self.joint_names))
                # only one joint - at a time - should move with constant velocity.
                velocities[i] = CONSTANT_VELOCITY

                # stopping time
                self.collecting_data = True
                now = rospy.get_rostime()
                while True:
                    dt = (rospy.get_rostime() - now).to_sec()
                    self.controller.publish_velocities(velocities, JOINT_ROT_TIME)
                    if dt > DATA_COLLECTION_TIME:
                        break

                    for imu_name in self.imu_names:
                        try:
                            # get pose of imu
                            (trans, rot) = self.tf_listener.lookupTransform('/world', imu_name, rospy.Time(0))
                            self.Q[imu_name].x = rot[0]
                            self.Q[imu_name].y = rot[1]
                            self.Q[imu_name].z = rot[2]
                            self.Q[imu_name].w = rot[3]
                        except (tf.LookupException,
                                tf.ConnectivityException,
                                tf.ExtrapolationException):
                            continue

                self.collecting_data = False
                rospy.sleep(1)

    def save(self, verbose=False, filter=False):
        """
        Save data to a pickle file.
        """
        data = self.data_storage.clean_data(verbose)

        self.data_storage.save(data, filter=filter)


if __name__ == "__main__":
    # [Pose, Joint, IMU, x, y, z]* number os samples according to hertz
    robot = sys.argv[1]
    # determine controller to use.
    if robot == 'panda':
        controller = PandaController()
        filename = 'panda_positions.txt'
    elif robot == 'sawyer':
        controller = SawyerController()
        filename = 'sawyer_positions.txt'
    else:
        raise ValueError("Must be either panda or sawyer")

    if len(sys.argv) > 2:
        filename = sys.argv[2]

    poses_list = utils.get_poses_list_file(filename)
    filepath = '_'.join(['data/constant_data', robot])

    cr = ConstantRotationDataSaver(controller, poses_list, filepath)
    # perform data collection
    cr.rotate_at_constant_vel()
    # save data
    cr.save(verbose=True, filter=True)
