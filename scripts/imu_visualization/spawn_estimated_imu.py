#!/usr/bin/env python
import os
import numpy as np
import pickle

import rospy
import rospkg
import tf
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SetModelState, GetLinkState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Quaternion, Point

from hiro_ros_arm_controller.RobotController import PandaController, SawyerController


class EstimatedIMUBoxStateManager():
    def __init__(self, model_names, init_poses=None, sdf=True):
        """
        Spawns the IMU model.

        Arguments
        ---------
        `model_names`: `List[str]`
            The model names for the IMU

        `init_poses`: `List[geometry_msgs.msg.Pose]`
            `[Pose(Position xyz, Orientation quaternion)]`
        """
        self.model_names = model_names
        self.n = len(model_names)
        ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')

        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        model_path = os.path.join(ros_robotic_skin_path, 'robots/imubox/model.sdf')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        with open(model_path, 'r') as f:
            xml_string = f.read().replace('\n', '')

        if init_poses is None:
            q = tf.transformations.quaternion_from_euler(0, 0, 0)
            orientation = Quaternion(q[0], q[1], q[2], q[3])
            position = Point(0.0, 0.0, 0.0)
            init_pose = Pose(position=position, orientation=orientation)
            init_poses = [init_pose for i in range(self.n)]

        self.init_poses = init_poses
        self.req = SpawnModelRequest()
        self.req.model_xml = xml_string

    def spawn(self):
        """
        Spawns the IMU model with the poses defined earlier in
        `__init__`
        """
        for model_name, pose in zip(self.model_names, self.init_poses):
            try:
                self.req.model_name = model_name
                init_pose = Pose()
                init_pose.position.x = pose.position[0]
                init_pose.position.y = pose.position[1]
                init_pose.position.z = pose.position[2]  # + 0.91488
                init_pose.orientation.x = pose.orientation[0]
                init_pose.orientation.y = pose.orientation[1]
                init_pose.orientation.z = pose.orientation[2]
                init_pose.orientation.w = pose.orientation[3]
                self.req.initial_pose = init_pose
                rospy.wait_for_service('/gazebo/spawn_sdf_model')
                self.spawn_model(self.req)
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s" % e)

    def set_poses(self, names, poses):
        """
        Sets the poses of the IMU.

        Arguments
        ---------
        `names`: `List[str]`
            The names of the IMUs

        `poses`: `List[geometry_msgs.msg.Pose]`
            The poses of the IMUs
        """
        for name, pose in zip(names, poses):
            self.set_pose(name, pose)

    def set_pose(self, name, pose):
        """
        Sets a pose of an IMU

        Arguments
        ---------
        `name`: `str`
            The name of the IMU

        `poses`: `geometry_msgs.msg.Pose`
            The pose of the IMU
        """
        init_pose = Pose()
        init_pose.position.x = pose.position[0]
        init_pose.position.y = pose.position[1]
        init_pose.position.z = pose.position[2]  # + 0.91488
        init_pose.orientation.x = pose.orientation[0]
        init_pose.orientation.y = pose.orientation[1]
        init_pose.orientation.z = pose.orientation[2]
        init_pose.orientation.w = pose.orientation[3]
        try:
            state_msg = ModelState()
            state_msg.model_name = name
            state_msg.pose = init_pose
            state_msg.reference_frame = "world"
            rospy.wait_for_service('/gazebo/set_model_state')
            self.set_model_state(state_msg)
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s" % e)


class TrueIMUBoxStateManager():
    def __init__(self, names):
        """
        TrueIMUBoxStateManager class.

        Arguments
        ---------
        `names`: `List[str]`
            The names of the IMUs
        """
        self.names = names
        self.n = len(names)

        self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        rospy.wait_for_service('/gazebo/get_link_state')

    def get_poses(self):
        """
        Gets the poses of the given links

        """
        poses = np.zeros((self.n, 7))
        for i, name in enumerate(self.names):
            resp = self.get_link_state(name, 'world')
            if resp.success:
                pose = resp.link_state.pose
                poses[i, :] = np.r_[pose.position, pose.orientation]

        return poses


def parse_pickle_data(pickle_filename):
    """
    parses pickle data.
    """
    data = pickle.load(open(pickle_filename, 'r'))
    trials = data['trials']
    keys = trials.keys()
    dh_params_data = []
    for key in keys:
        if len(dh_params_data) != trials[key]['imu_num']:
            dh_params_data.append([])
        dh_params_data.append(trials[key]['position'] + trials[key]['orientation'])

    return dh_params_data

def parse_pickle_data_best(pickle_filename):
    """
    parses pickle data.
    """
    data = pickle.load(open(pickle_filename, 'r'))
    best_data = data['best_data']
    keys = best_data['elapsed_time'].keys()
    dh_params_data = []
    for key in keys:
        dh_params_data.append(best_data['position'][key] + best_data['orientation'][key])
    return dh_params_data


if __name__ == '__main__':
    rospy.init_node("set_estimated_imu_positions")
    # parser = argparse.ArgumentParser(description='IMU Spawner')
    # parser.add_argument('-r', '--robot', type=str, default='panda',
    #                     help="Currently only 'panda' and 'sawyer' are supported")
    # parser.add_argument('-p', '--picklepath', type=str, default='panda_OM.pickle',
    #                     help="Path to pickle file from within the data directory.")
    # parser.add_argument('-f', '--frequency', type=float, default=50.0,
    #                     help='Frequency at which the animation occurs.')
    # args = parser.parse_args()
    robot = 'panda'
    # data from corresponding method
    filename = 'panda_OM_jan16.pickle'

    frequency = 50.0
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    load_path = os.path.join(ros_robotic_skin_path, 'data', filename)

    if os.path.exists(load_path):
        # dh_params_data = parse_pickle_data(load_path)
        dh_params_data = parse_pickle_data_best(load_path)

    else:
        raise EnvironmentError("File not found!")
    # dh_params_data shape is (num_su, optimization_steps, dh_params)
    if robot == 'panda':
        controller = PandaController()
    elif robot == 'sawyer':
        controller = SawyerController()
    else:
        raise ValueError("Must be either panda or sawyer")

    positions = np.array([0, 0, 0, -0.0698, 0, 0, 0])
    controller.publish_positions(positions, sleep=1)
    imu_names = ['imu_link0', 'imu_link1', 'imu_link2',
                 'imu_link3', 'imu_link4', 'imu_link5', 'imu_link6']
    link_names = ['panda::' + imu_name for imu_name in imu_names]
    panda_imu_manager = TrueIMUBoxStateManager(link_names)
    defined_poses = panda_imu_manager.get_poses()
    # Set initial poses
    # dh_params_data shape is (num_su, optimization_steps, dh_params)
    r = rospy.Rate(frequency)
    n_imu = len(dh_params_data)
    model_names = ['imu%i' % (i+1) for i in range(n_imu)]

    poses = np.zeros((n_imu, 7))
    init_poses = [Pose(position=pose[:3], orientation=pose[3:]) for pose in poses]
    state_manager = EstimatedIMUBoxStateManager(model_names, init_poses)
    state_manager.spawn()
    for su_idx in range(3):
        pose = dh_params_data[su_idx]
        pose = Pose(position=pose[:3], orientation=pose[3:])
        state_manager.set_pose(model_names[su_idx], pose)
    sleep(3)
    # optimization_lengths = []
    # for su_idx, val in enumerate(dh_params_data):
    #     optimization_lengths.append(len(dh_params_data[su_idx]))

    # optimization_lengths = np.array(optimization_lengths)
    # indices = [0] * 6
    # su_idx = 0
    # done_amt = 0

    # while True:
    #     if indices[su_idx] == optimization_lengths[su_idx]:
    #         # skip su.
    #         pass
    #     else:
    #         pose = dh_params_data[su_idx][indices[su_idx]]
    #         pose = Pose(position=pose[:3], orientation=pose[3:])
    #         state_manager.set_pose(model_names[su_idx], pose)

    #         indices[su_idx] += 1
    #         if indices[su_idx] == optimization_lengths[su_idx]:
    #             done_amt += 1
    #             if done_amt == n_imu:
    #                 break
    #     su_idx += 1
    #     su_idx %= n_imu
    #     print('Optimization steps for imus:', indices)
    #     r.sleep()

    # error = np.linalg.norm(defined_poses - poses)
    # print(error)
