#!/usr/bin/env python

# This Python node is used to determine which controller will be used when the proximity_sim.launch file is launched

import rospy
import numpy as np
from CartesianPositionController import CartesianPositionController
from ObstacleAvoidanceController import ObstacleAvoidanceController

# Obtains the value inputed by user for which controller to use from proximity_sim.launch file (controller objects currently hardcoded)
controller_val = rospy.get_param("controller")

# Conditionals to test which controller to launch
if controller_val == 0:  # No controller selected
    pass

elif controller_val == 1:  # Obstacle avoidance controller selected
    obstacle_avoidance_controller = ObstacleAvoidanceController()
    x0 = 0.6
    y0 = 0.0
    z0 = 0.3
    r = 0.2
    resolution = 0.4
    trajectory = obstacle_avoidance_controller.get_trajectory_points_in_circle_yz_plane(r, x0, y0, z0, resolution)
    while not rospy.is_shutdown():
        for point in trajectory:
            if not rospy.is_shutdown():
                obstacle_avoidance_controller.go_to_point(point)

elif controller_val == 2:  # Cartesian position controller selected
    x0 = 0.6
    y0 = 0.0
    z0 = 0.5
    r = 0.45
    resolution = np.pi / 2

    # Loop that trajectory
    cartesian_controller = CartesianPositionController()
    trajectory = cartesian_controller.get_trajectory_points_in_circle_yz_plane(r, x0, y0, z0, resolution)
    while not rospy.is_shutdown():
        cartesian_controller.go_to_points_in_trajectory(trajectory)

else:  # Some other option selected; carries to effect
    pass
