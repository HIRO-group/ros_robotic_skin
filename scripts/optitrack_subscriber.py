import os
import pickle
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped


def save(pickle_data):
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    filepath = os.path.join(ros_robotic_skin_path, 'data/optitrack_data.pickle')

    with open(filepath, 'ab') as f:
        pickle.dump(pickle_data, f)


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + 'I heard %s ', data)
    save(data)

def listener():
    rospy.init_node('listener', anonymous=True)

    topic_name = "/vrpn_client_node/RigidBody01/pose"
    topic_type = PoseStamped
    rospy.Subscriber(topic_name, topic_type , callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.sleep(3)


if __name__ == '__main__':
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    filepath = os.path.join(ros_robotic_skin_path, 'data/optitrack_data.pickle')

    if os.path.exists(filepath):
        os.remove(filepath)
    listener()

