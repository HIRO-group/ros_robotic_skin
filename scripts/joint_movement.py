#!/usr/bin/python

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Create Publishers and Init Node
pub = rospy.Publisher('/panda_arm_controller/command', JointTrajectory, queue_size=1)
pub_int = rospy.Publisher('/joint_mvmt_dof', Int16, queue_size=1)
pub_bool = rospy.Publisher('/calibration_complete', Bool, queue_size=1)
rospy.init_node('calibration_joint_mvmt_node', anonymous=True)

# Create and set Joint Trajectory msg
msg = JointTrajectory()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = '/base_link'
msg.joint_names = ['panda_joint1', 'panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']

# Set Initial position of the robot
point = JointTrajectoryPoint()
point.positions = [0, 0, 0, 0, 0, 0, 0]
point.time_from_start.secs = 1
msg.points = [point]
joint_int = -1


# TODO: Look up do we need to have one message to init the robot?
# If I only send one message then the franka does not move.
# TODO: This might not be the best starting postion for the robot to be in.
# Think about if we are losing some inforamtion by using a completely vertical postion
# for the start.
# Move arm to starting position


pub.publish(msg)
rospy.sleep(1)
pub.publish(msg)
rospy.sleep(1)


def talker():
    # specify all global variables that were defined outside of this function
    global joint_int
    global point
    global msg
    global pub
    global pub_int
    global pub_bool

    while not rospy.is_shutdown():

        # Increment the Dof we are actuating here
        joint_int = (joint_int + 1)

        # Check if we have actuated every DoF, to end this script
        if joint_int == 7:
            pub_bool.publish(True)
            print('CALIBRATION COMPLETE')
            break

        point.positions[joint_int] = -1
        msg.points = [point]

        # Publish this message so activation_matrix.py knows which Dof what actuated
        pub_int.publish(joint_int)
        rospy.sleep(1)

        #publish message to actuate the dof
        pub.publish(msg)
        rospy.sleep(5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
