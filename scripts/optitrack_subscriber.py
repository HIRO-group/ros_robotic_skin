import os
import pickle
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped


def save(pickle_data):
    """
    Saves the data to a pickle file.
    Arguments
    ----------
    `data`: `PoseStamped`
        The data to be saved
    """
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    filepath = os.path.join(ros_robotic_skin_path, 'data/optitrack_data.pickle')

    with open(filepath, 'ab') as f:
        pickle.dump(pickle_data, f)


def callback(data):
    """
    A callback function for optitrack topics
    Arguments
    ----------
    data: geometry_msgs.msg.PoseStamped
        Data for the rigid body under optitrack
    """
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s ', data)
    save(data)


def listener():
    rospy.init_node('listener', anonymous=True)

    # saves data for RigidBody01
    topic_name = "/vrpn_client_node/RigidBody01/pose"
    topic_type = PoseStamped
    rospy.Subscriber(topic_name, topic_type, callback)

    # saves 3 seconds of data
    rospy.sleep(3)


if __name__ == '__main__':
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    filepath = os.path.join(ros_robotic_skin_path, 'data/optitrack_data.pickle')

    if os.path.exists(filepath):
        os.remove(filepath)
    listener()
