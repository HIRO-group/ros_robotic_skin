from gazebo_msgs import ModelState
import rospy


rospy.init_node("gazebo_move")
pub = rospy.Publisher('/gazebo/model_states', ModelState, queue_size=100)
msg = ModelState()
