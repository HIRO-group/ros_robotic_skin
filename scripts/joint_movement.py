#!/usr/bin/env python

import rospy 
from std_msgs.msg import Int16
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

pub = rospy.Publisher('/panda_arm_controller/command', JointTrajectory, queue_size=1)
pub_int = rospy.Publisher('/joint_mvmt_dof', Int16, queue_size=1)
rospy.init_node('calibration_joint_mvmt_node', anonymous=True)

# Create and set Joint Trajectory msg
msg = JointTrajectory()
msg.header.stamp = rospy.Time.now()
msg.header.frame_id = '/base_link'
msg.joint_names = ['panda_joint1', 'panda_joint2','panda_joint3','panda_joint4','panda_joint5','panda_joint6','panda_joint7']    

point = JointTrajectoryPoint() 
point.positions = [0, 0, 0, 0, 0, 0, 0]
point.time_from_start.secs = 1    
msg.points = [point]

joint_int = -1


def talker():
    # specify all global variables that were defined outside of this function
    global joint_int 
    global point
    global msg
    global pub 
    global pub_int

    # Move the arm to the initial starting position
    print('Move arm to starting position')
    pub.publish(msg)
    rospy.sleep(5)


    while not rospy.is_shutdown():
        joint_int = (joint_int + 1)
        if joint_int == 7:
            break
        point.positions[joint_int] = 1
        msg.points = [point]

        # Publish this message so I know what dof is being actuated
        pub_int.publish(joint_int)
        print('DoF Published')
        rospy.sleep(1) # Is there a better way to send this information?

        # publish message to actuate the dof sent 1 sec earlier
        pub.publish(msg)
        print('Joint Position Published')
        rospy.sleep(5)

    print('Activation Matrix Complete')

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
