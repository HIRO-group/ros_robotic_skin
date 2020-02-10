#!/usr/bin/env python
import os
import math
import rospy, rospkg, tf
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SetModelState, SetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import *

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


if __name__ == '__main__':
    rospy.init_node("set_estimated_imu_positions")
    n_joint = 7

    # Set initial poses    
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    orientation = Quaternion(q[0], q[1], q[2], q[3])
    positions = [Point(1, 0, 0.2*i) for i in range(n_joint)]

    model_names = ['imu%i'%(i) for i in range(n_joint)]
    init_poses = [Pose(position=pos, orientation=orientation) for pos in positions]
    state_manager = IMUBoxStateManager(model_names, init_poses) 
    state_manager.spawn()

    sim_freq = 100.0
    r = rospy.Rate(sim_freq) 

    t = 0.0
    circle_freq = 1 # round / sec
    pi = math.pi
    while not rospy.is_shutdown():
        th = 2*pi*circle_freq*t
        print(t, 1/sim_freq)
        print(th)
        print(math.cos(th), math.sin(th))
        for i, name in enumerate(model_names):
            pose = Pose(position=Point(math.cos(th), math.sin(th), 0.2*i), orientation=orientation)
            state_manager.set_pose(name, pose)
        t += 1/sim_freq
        r.sleep()
