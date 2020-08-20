import os
import sys
import pickle
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped

ROS_PATH = rospkg.RosPack().get_path('ros_robotic_skin')
DIRPATH = os.path.join(ROS_PATH, 'data', 'optitrack_data')
FILEPATH = os.path.join(DIRPATH, 'optitrack_data.pickle')


class OptiTrackPoseDataSaver:
    """
    Class for collecting all pose data for the objects in optitrack
    """
    def __init__(self):
        """
        Initializes the list that stores all data
        """
        self.pose_data = []

    def append_pose(self, pickle_data):
        """
        Adds all optitrack data to a list
        Arguments
        ----------
        `picke_data`: `PoseStamped`
            The data to be added to the list
        """
        self.pose_data.append(pickle_data)

    def write_pickle_file(self):
        """
        Saves the data from the list to a pickle file.
        """
        with open(FILEPATH, 'wb') as f:
            pickle.dump(self.pose_data, f)


def optitrack_listener(topic, duration, poseDataClass):
    """
    Subscribes to events from the optitrack and get PoseStamped data for the
    location of the given object and sends callback data to `save_to_pickle`
    Arguments
    ----------
    `obj`: `String`
        The object the optitrack listener will track
    `duration`: `Integer`
        The duration the script will collect data for
    `poseDataClass`: `Class`
        The class that has access to the function append_pose
    """

    # initializes the subscriber
    rospy.init_node('optitrack_listener', anonymous=True)
    rospy.Subscriber(topic[0], PoseStamped, poseDataClass.append_pose)

    # saves n seconds of data
    rospy.sleep(duration)


if __name__ == '__main__':

    # accesses command line arguments through rospy
    args = rospy.myargv(argv=sys.argv)

    # checks if number of arguments are valid
    if len(args) != 3:
        raise ValueError("RigidBody or Duration not provided!")

    # checks if topic is valid
    topic = ["/vrpn_client_node/{}/pose".format(args[1]), 'geometry_msgs/PoseStamped']
    if topic not in rospy.get_published_topics():
        raise ValueError("RigidBody does not exist!")

    # checks if duration is valid
    try:
        duration = int(args[2])
    except ValueError:
        raise ValueError("Duration was not a valid integer.")

    if not os.path.exists(DIRPATH):
        os.makedirs(DIRPATH)

    optitrackPoseData = OptiTrackPoseDataSaver()
    optitrack_listener(topic, duration, optitrackPoseData)

    # calls the write function to save the data
    optitrackPoseData.write_pickle_file()
