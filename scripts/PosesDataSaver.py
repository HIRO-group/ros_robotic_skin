from PandaPose import PandaPose
import numpy as np
import random
import rospy
from sensor_msgs.msg import Imu

################################################
###### Poses Configuration #####################
poses_list = [
    [[-1, -1, -1, -3, -1, -1, -1], [], 'Pose_1']
]
################################################

def callback(data):





class PandaPosesDataSaver(PandaPose):
    def __init__(self):
        super(PandaPosesDataSaver, self).__init__()

    def get_imu_data(self):
        rospy.Subscriber('imu_data2', Imu, callback)



# Lets generate poses for review
if __name__ == "__main__":
    pp = PandaPosesDataSaver()
