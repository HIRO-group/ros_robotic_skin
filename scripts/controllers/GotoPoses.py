import os
import numpy as np
import rospkg
import rospy

from RobotController import PandaController


if __name__ == '__main__':
    rospy.init_node('matt_strong')
    ros_robotic_skin_path = rospkg.RosPack().get_path('ros_robotic_skin')
    panda_positions_path = os.path.join(ros_robotic_skin_path, 'config', 'panda_positions.txt')

    panda_positions_arr = np.loadtxt(panda_positions_path)
    panda_controller = PandaController(is_sim=False)
    # velocities = [-0.2, 0, 0, 0, 0, 0, 0]
    # while True:
    #     panda_controller.send_velocities(list(velocities))
    for pose in panda_positions_arr:
        print(list(pose))
        panda_controller.publish_positions(list(pose), sleep=10.0)
