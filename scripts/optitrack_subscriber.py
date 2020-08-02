import os
import sys
import pickle
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped

ROS_PATH = rospkg.RosPack().get_path('ros_robotic_skin')
FILEPATH = os.path.join(ROS_PATH, 'data/optitrack_data.pickle')


class poseData:
    """
    Class for collecting all pose data for the objects in optitrack
    """
    def __init__(self):
        """
        Initializes the list that stores all data
        """
        self.pose_data = []

    def save_to_pickle(self, pickle_data):
        """
        Adds all optitrack data to a list
        Arguments
        ----------
        `picke_data`: `PoseStamped`
            The data to be added to the list
        """
        self.pose_data.append(pickle_data)

    def writePickleFile(self):
        """
        Saves the data from the list to a pickle file.
        """
        with open(FILEPATH, 'wb') as f:
            pickle.dump(self.pose_data, f)


def optitrack_listener(topic, poseDataClass):
    """
    Subscribes to events from the optitrack and get PoseStamped data for the
    location of the given object and sends callback data to `save_to_pickle`
    Arguments
    ----------
    `obj`: `String`
        The object the optitrack listener will track
    """

    # initializes the subscriber
    rospy.init_node('optitrack_listener', anonymous=True)
    rospy.Subscriber(topic[0], PoseStamped, poseDataClass.save_to_pickle)

    # saves 3 seconds of data
    rospy.sleep(3)


if __name__ == '__main__':

    # accesses command line arguments through rospy
    args = rospy.myargv(argv=sys.argv)

    if len(args) != 2:
        raise ValueError("RigidBody not provided!")

    topic = ["/vrpn_client_node/"+args[1]+"/pose", 'geometry_msgs/PoseStamped']
    if not topic in rospy.get_published_topics():
        raise ValueError("RigidBody does not exist!")

    newSave = poseData()
    optitrack_listener(topic, newSave)

    # calls the write function to save the data
    newSave.writePickleFile()
