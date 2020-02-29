#!/usr/bin/env python
import os
import math
import numpy as np
import rospy, rospkg, tf
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SetModelState, SetLinkState, GetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import *

import utils
from SawyerController import SawyerController
from PandaController import PandaController

class EstimatedIMUBoxStateManager():
    def __init__(self, model_names, init_poses=None, sdf=True):
        """
        init_poses: List of geometry_msgs.msg.Pose
            [Pose(Position xyz, Orientation quaternion)]
        sdf: bool
            launch from sdf or urdf
        """
        self.model_names = model_names
        self.n = len(model_names)
        self.sdf = sdf
        ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')

        rospy.wait_for_service('/gazebo/set_model_state')
        if sdf:
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            model_path = os.path.join(ros_robotic_skin_path, 'robots/imubox/model.sdf')
            self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            with open(model_path, 'r') as f:
                xml_string = f.read().replace('\n', '')
        else:
            rospy.wait_for_service('/gazebo/spawn_urdf_model')
            self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            model_path = os.path.join(ros_robotic_skin_path, 'robots/imu.urdf')
            self.set_link_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
            with open(model_path, 'r') as f:
                xml_string = f.read()
        
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
        for model_name, pose in zip(self.model_names, self.init_poses):
            try:
                self.req.model_name = model_name
                self.req.initial_pose = pose
                res = self.spawn_model(self.req) 
            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed: %s" % e)

    def set_poses(self, names, poses):
        for name, pose in zip(names, poses):
            self.set_pose(name, pose)

    def set_pose(self, name, pose):
        if self.sdf:
            try:
                state_msg = ModelState()
                state_msg.model_name = name
                state_msg.pose = pose
                state_msg.reference_frame = "world"
                res = self.set_model_state(state_msg) 
            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed: %s" % e)
        else:
            try:
                link_msg = LinkState()
                link_msg.link_name = name 
                link_msg.pose = pose
                link_msg.reference_frame = "world"
                res = self.set_link_state(state_msg) 
                print(res)
            except rospy.ServiceException, e:
                rospy.loginfo("Service call failed: %s" % e)

class TrueIMUBoxStateManager():
    def __init__(self, names):
        self.names = names
        self.n = len(names)

        self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        rospy.wait_for_service('/gazebo/get_link_state')

    def get_poses(self):
        poses = np.zeros((self.n, 7))
        for i, name in enumerate(self.names):
            resp = self.get_link_state(name, 'world')

            if resp.success:
                pose = resp.link_state.pose
                poses[i, :] = np.r_[pose.position, pose.orientation]

        return poses

def load_estimated_poses(filename):
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    return np.loadtxt(os.path.join(ros_robotic_skin_path, 'data', filename))

if __name__ == '__main__':
    rospy.init_node("set_estimated_imu_positions")
    n_joint = 7
    n_imu = 7

    robot = sys.argv[1]
    filename = sys.argv[2]

    if robot == 'panda':
        controller = PandaController()
    elif robot == 'sawyer':
        controller = SawyerController()
    else:
        raise ValueError("Must be either panda or sawyer")
    
    poses = load_estimated_poses(filename)
    positions = np.zeros(len(poses.shape[0]))
    controller.publish_positions(positions, sleep=1)

    imu_names = ['imu_link0', 'imu_link1', 'imu_link2', 'imu_link3', 'imu_link4', 'imu_link5', 'imu_link6']
    link_names = ['panda::'+imu_name for imu_name in imu_names]
    panda_imu_manager = TrueIMUBoxStateManager(link_names)
    defined_poses = panda_imu_manager.get_poses()

    # Set initial poses    
    model_names = ['imu%i'%(i) for i in range(n_imu)]
    init_poses = [Pose(position=pose[:3], orientation=pose[3:]) for pose in poses]
    state_manager = EstimatedIMUBoxStateManager(model_names, init_poses) 
    state_manager.spawn()

    error = np.sum(np.linalg.norm(defined_poses - poses))
    print()
