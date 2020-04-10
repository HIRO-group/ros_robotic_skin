#!/usr/bin/env python


from CartesianController import CartesianController
import numpy as np
import rospy
import tf


class ObstacleAvoidance(CartesianController):
    def __init__(self):

        super(ObstacleAvoidance, self).__init__()

        self.control_points = []

    def end_effector_algorithm(self, xd):
        np.array(xd)
        V_max = 1 # m/s
        alpha = 6 # shape vector
        rho = 0.4 # m

        D = xd[0:2]
        D_norm = np.linalg.norm(D)
        D_unit_vec = D/D_norm
       
        v_repulse = (V_max/(1+ np.exp((D_norm*(rho/2)-1)*alpha)))*D_unit_vec
        
        xd[0:2] = D + v_repulse
        
        xc = xd
        
        return xc

    def is_array_in_list(self, element, list_of_vectors):
        for vector in list_of_vectors:
            if np.array_equal(element, vector):
                return True
        return False

    def get_obstacle_points(self):
        self.obstacle_points = [np.array([0., 0.,  0.333])]

    def go_to_point(self, position_desired):
        """
        Move to a point an stop when arrived

        Parameters
        ----------
        position_desired : numpy.ndarray
            3x1 end effector desired position vector
            Only position, not orientation
        """
        position_desired = np.array(position_desired)
        self.error = position_desired - self.position
        while np.linalg.norm(self.error) > self.error_threshold:
            self.position = self.get_current_end_effector_position()
            xd_dot = self.compute_command_velocity(position_desired)
            # Add flacco algorithm for end effector to get cartesian velocity xc_dot
            # TODO: Calc |D(P,P)|
            # TODO: Calc v(P,O)
            # TODO: Return xc_dot
            xc_dot = self.end_effector_algorithm(xd_dot)
            # Compute the corresponding joint velocities q_dot
            self.q_dot = self.compute_command_q_dot(xc_dot)
            # Compute the joint velocity restrictions and apply them
            # restrictions = self.body_algorithm()
            # q_restricted = self.apply_restrictions()
            # Publish velocities
            self.panda_controller.send_velocities(self.q_dot)
            self.rate.sleep()
        self.stop()
        return 0


if __name__ == "__main__":
    controller = ObstacleAvoidance()
    #controller.get_control_points()
    while not rospy.is_shutdown():
        controller.go_to_point([0.65, 0, 0.3])
        controller.go_to_point([0.4, 0, 0.3])
        controller.go_to_point([0.65, 0, 0.3])
