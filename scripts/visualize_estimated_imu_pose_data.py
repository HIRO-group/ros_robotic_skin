#!/usr/bin/env python
import os
import math
import rospy, rospkg, tf
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SetModelState, SetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import *
from sensor_msgs.msg import Imu

class IMUBoxStateManager():
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
        self.imu_q = Quaternion()
        self.position = Point(1,1,1)
        init_poses = [Pose(position=self.position, orientation=self.imu_q)]
        #rospy.Subscriber('imu0_pose', Quaternion, self.imu_callback)
        rospy.Subscriber('imu/data', Imu, self.imu_callback)

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

    def imu_callback(self, data):
        # self.imu_q = data
        self.imu_q = data.orientation
    
    def spawn(self):
        for model_name, pose in zip(self.model_names, self.init_poses):
            try:
                self.req.model_name = model_name
                self.req.initial_pose = pose
                res = self.spawn_model(self.req) 
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s" % e)

    def set_poses(self, names, poses):
        for name, pose in zip(names, poses):
            self.set_pose(name, pose)

    def set_pose(self, name, pose=None):
        pose = Pose(position=self.position, orientation=self.imu_q)

        if self.sdf:
            try:
                state_msg = ModelState()
                state_msg.model_name = name
                state_msg.pose = pose
                state_msg.reference_frame = "world"
                res = self.set_model_state(state_msg) 
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s" % e)
        else:
            try:
                link_msg = LinkState()
                link_msg.link_name = name 
                link_msg.pose = pose
                link_msg.reference_frame = "world"
                res = self.set_link_state(state_msg) 
                print(res)
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node("set_estimated_imu_positions_data")

    model_names = ['imu6']
    state_manager = IMUBoxStateManager(model_names) 
    state_manager.spawn()

    r = rospy.Rate(30) 

    while not rospy.is_shutdown():
        state_manager.set_pose(model_names[0])
        r.sleep()
