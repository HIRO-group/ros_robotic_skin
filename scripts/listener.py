#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def callback(data):
    a = data.linear_acceleration
    print(a.x, a.y, a.z)
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/imu", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()