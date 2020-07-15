#!/usr/bin/env python
import sys
import rospy
import rospkg
sys.path.append(rospkg.RosPack().get_path('ros_robotic_skin'))
from scripts.controllers.RobotController import PandaController  # noqa: E402

COMMAND_SWITCHING_TIME = 5
POSITION1 = [0, 0, 0, 0, 0, 0, 0]
POSITION2 = [1, 0, 0, 0, 0, 0, 0]
VELOCITIES = [0.5, 0, 0, 0, 0, 0, 0]
ACCELERATIONS = [0, 0, 0, 0, 0, 0, 0]

if __name__ == '__main__':
    i_joint = 1

    if len(sys.argv) > 1:
        i_joint = sys.argv[1]

    if len(sys.argv) > 2:
        POSITION1 = float(sys.argv[2])
    if len(sys.argv) > 3:
        POSITION2 = float(sys.argv[3])

    # rospy.init_node('periodical_position_publisher', anonymous=True)
    controller = PandaController()

    curr_pos = 1

    now = rospy.get_rostime()
    while not rospy.is_shutdown():
        dt = (rospy.get_rostime() - now).to_sec()

        if dt > COMMAND_SWITCHING_TIME:
            positions = POSITION1 if curr_pos == 1 else POSITION2
            controller.publish_trajectory(positions, VELOCITIES, ACCELERATIONS, None)
            now = rospy.get_rostime()
            curr_pos = 2 if curr_pos == 1 else 1
