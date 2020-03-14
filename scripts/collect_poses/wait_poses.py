#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool, Int16

# a class to capture key poses of the panda


class WaitPose():

    def __init__(self, joints=7):

        """
        Creates a wait pose object.

        Arguments
        ----------
        `joints`: `int`
            Sets the amount of joints in the desired robot

        Returns
        ----------
        returns: None
        """

        # publisher for when the user wants the pose to be captured
        rospy.init_node("wait_poses", anonymous=True)

        self.pose_num_pub = rospy.Publisher("zero_g_pose_num", Int16)
        self.is_in_captured_pose_pub = rospy.Publisher("is_in_captured_pose", Bool)
        self.poses = rospy.get_param("/zero_g_poses", default=11)

        self.pose_num = 0

    def wait_poses(self):
        """
        Waits for all of the poses. Requires a user to press enter
        when the desired pose is reached. Peforms this for
        `self.poses` times.

        Arguments
        ----------
        None

        Returns
        ----------
        returns: None
        """
        for i in range(self.poses):
            self.pose_num = i

            self.pose_num_pub.publish(Int16(self.pose_num))
            self.is_in_captured_pose_pub.publish(Bool(False))
            print("Please move the robot to the desired pose.")
            raw_input("Press enter to capture the pose: ")   # noqa: F821
            self.is_in_captured_pose_pub.publish(Bool(True))
            print("==========================")
            print("Please wait 1 second in this pose...")
            # sleep for 5 seconds
            rospy.sleep(1)


if __name__ == "__main__":
    wp = WaitPose()
    wp.wait_poses()
