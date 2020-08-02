import os
import sys
import pickle
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped

ROS_PATH = rospkg.RosPack().get_path('ros_robotic_skin')
FILEPATH = os.path.join(ROS_PATH, 'data/optitrack_data.pickle')


def save_to_pickle(pickle_data):
    """
    Saves the data to a pickle file.
    Arguments
    ----------
    `data`: `PoseStamped`
        The data to be saved
    """

    with open(FILEPATH, 'ab') as f:
        pickle.dump(pickle_data, f)
        # print(pickle_data)


def optitrack_listener(obj):
    """
    Subscribes to events from the optitrack and get PoseStamped data for the
    location of the given object and sends callback data to `save_to_pickle`
    Arguments
    ----------
    `obj`: `String`
        The object the optitrack listener will track
    """

    # initializes the subscriber and listens to the object provided through the command line and sends callback data
    rospy.init_node('optitrack_listener', anonymous=True)
    rospy.Subscriber("/vrpn_client_node/"+obj+"/pose", PoseStamped, save_to_pickle)

    # saves 3 seconds of data
    rospy.sleep(3)


if __name__ == '__main__':

    # accesses command line arguments through rospy
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 2:
        raise ValueError("RigidBody not provided!")
    elif not args[1] in rospy.get_published_topics():
        raise ValueError("RigidBody does not exist!")

    # deletes the file before starting to get a clean dataset.
    if os.path.exists(FILEPATH):
        os.remove(FILEPATH)

    optitrack_listener(args[1])
