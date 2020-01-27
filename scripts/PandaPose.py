import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi
import time


class PandaPose:
    def __init__(self):
        # Create Publishers and Init Node
        self.pub = rospy.Publisher('/panda_arm_controller/command', JointTrajectory, queue_size=1)
        self.pub_int = rospy.Publisher('/joint_mvmt_dof', Int16, queue_size=1)
        self.pub_bool = rospy.Publisher('/calibration_complete', Bool, queue_size=1)
        rospy.init_node('calibration_joint_mvmt_node', anonymous=True)
        self.msg = JointTrajectory()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = '/base_link'
        self.msg.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5',
                                'panda_joint6', 'panda_joint7']

        # Set Initial position of the robot
        self.point = JointTrajectoryPoint()
        self.point.positions = [0, 0, 0, 0, 0, 0, 0]
        self.point.velocities = [0, 0, 0, 0, 0, 0, 0]
        self.point.time_from_start.secs = 10
        self.msg.points = [self.point]
        # TODO: Look up do we need to have one message to init the robot?
        # If I only send one message then the franka does not move.
        # TODO: This might not be the best starting position for the robot to be in.
        # Think about if we are losing some information by using a completely vertical position
        # for the start.
        # Move arm to starting position
        self.pub.publish(self.msg)
        rospy.sleep(1)
        self.pub.publish(self.msg)
        rospy.sleep(1)

    def _set_pose(self, pose, pose_string):
        if len(pose) != 7:
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        for index, _ in enumerate(self.point.positions):
            self.point.positions[index] = pose[index]

    def _set_velocity(self, velocity):
        if len(velocity) != 7:
            raise Exception("The length of input list should be 7, as panda has 7 arms")
        for index, _ in enumerate(velocity):
            self.point.velocities[index] = velocity[index]

    def _set_all_values(self):
        self.msg.points = [self.point]
        if not rospy.is_shutdown():
            # publish message to actuate the dof
            self.pub.publish(self.msg)
            time.sleep(5)

    def set_poses(self, poses):
        for each_pose in poses:
            pose_configuration, velocity_configuration, pose_string = each_pose[0], each_pose[1], each_pose[2]
            self._set_pose(pose_configuration, pose_string)
            # self._set_velocity(velocity_configuration)
            self._set_all_values()


if __name__ == "__main__":
    poses_list = [
        [[-1, -pi / 3, -pi / 4, 1, 1, 1, -pi / 4], [0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01], 'Pose_1'],
        [[-1, -1, -1, -1, -1, -1, -1], [0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9], 'Pose_2']
    ]
    pp = PandaPose()
    pp.set_poses(poses_list)
